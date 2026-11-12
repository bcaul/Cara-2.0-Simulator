"""Generate the CARA 2.0 MJCF model from the STEP assembly + STL files.

    python -m cara.model.build_model          (writes cara/model/cara.xml + assets/)

Design
------
* Every body frame is world-aligned at the reference configuration, which is
  the CAD pose = firmware standing pose getIK(0, e, 280). Joint position q = 0
  there; firmware angle theta = thetaRef + q (sign handled by the hinge axis).
* Visual meshes are the real STLs placed with the STEP instance transforms.
* Collision: boxes for body/carriage, capsules for links, sphere (squash ball) foot.
* The coaxial 5-bar is a true closed chain: upperA-lowerA and upperB-lowerB are
  separate branches joined at the ankle by an <equality connect>.
* Masses/inertias: shell+infill model per printed part + catalogue purchased parts
  at their STEP locations (see docs/ASSUMPTIONS.md, printed by --report).
"""
from __future__ import annotations

import argparse
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import numpy as np
import trimesh

from .. import config as C
from ..firmware.kinematics import getIK, linkage_points
from .mass_props import (MassElement, combine, point_box, point_cylinder,
                         printed_part_element)
from .step_assembly import StepFile

R_S2M = np.array(C.STEP_TO_SIM_ROT, float)
CENTER = np.array(C.STEP_BODY_CENTER, float)
MAX_VIS_FACES = 8000

# STEP product name -> STL base name (when they differ)
PRODUCT_TO_STL = {
    "Side_End_Plate_1": "Side_End_Plate",
    "Side_End_Plate_2": "Side_End_Plate",
    "MKS_XDrive_Mini_Cover": "Controller_Cover",
}

# Leg-subassembly part -> segment
SEGMENT_OF = {
    "ABAD_Plate": "base", "ABAD_Brace_Arm": "base", "ABAD_Plate_Cap": "base",
    "Carriage_Front_Plate": "carriage", "Carriage_Back_Plate": "carriage",
    "Carriage_Front_Spacer": "carriage", "Carriage_Back_Spacer": "carriage",
    "Big_Drum_ABAD": "carriage", "B_Brace_Arm": "carriage",
    "Big_Drum_A": "upperA", "Big_Drum_A_MIR": "upperA", "Upper_Link_A": "upperA",
    "Upper_Link_A_MIR": "upperA", "Big_Drum_A_Cap": "upperA",
    "Big_Drum_B": "upperB", "Big_Drum_B_MIR": "upperB",
    "Lower_Link_A": "lowerA", "Lower_Link_B": "lowerB",
}
SEGMENTS = ["carriage", "upperA", "lowerA", "upperB", "lowerB"]

COLORS = {
    "base": "0.12 0.12 0.13 1", "carriage": "0.15 0.15 0.16 1", "upperA": "0.85 0.65 0.15 1",
    "upperB": "0.85 0.65 0.15 1", "lowerA": "0.15 0.15 0.16 1", "lowerB": "0.15 0.15 0.16 1",
    "motor": "0.55 0.55 0.6 1",
}


# ---------------------------------------------------------------------------
def _cluster_decimate(mesh: trimesh.Trimesh, cell: float) -> trimesh.Trimesh:
    """Vertex-clustering decimation on a `cell`-mm grid (visual meshes only)."""
    key = np.floor(mesh.vertices / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    n = inv.max() + 1
    verts = np.zeros((n, 3))
    np.add.at(verts, inv, mesh.vertices)
    verts /= np.bincount(inv, minlength=n)[:, None]
    faces = inv[mesh.faces]
    ok = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    faces = np.unique(np.sort(faces[ok], axis=1), axis=0, return_index=True)[1]
    faces = inv[mesh.faces][ok][np.sort(faces)]
    return trimesh.Trimesh(verts, faces, process=False)


def step_to_sim(T: np.ndarray) -> np.ndarray:
    S = np.eye(4)
    S[:3, :3] = R_S2M @ T[:3, :3]
    S[:3, 3] = R_S2M @ (T[:3, 3] - CENTER)
    return S


def mat2quat(Rm: np.ndarray) -> np.ndarray:
    import mujoco
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(Rm).flatten())
    return q


def fmt(a, nd=6):
    return " ".join(f"{float(x):.{nd}g}" if abs(float(x)) > 1e-12 else "0" for x in np.ravel(a))


def leg_reference_geometry(i: int) -> dict:
    """Sim-frame (metres) joint locations of leg i at the reference pose."""
    sx, sy = C.LEG_SX[i], C.LEG_SY[i]
    tA, tB, tAB = getIK(0, C.e, C.zDefault)
    lp = linkage_points(tA * 2 * math.pi, tB * 2 * math.pi)
    abad = np.array([sx * C.v / 2, sy * C.w / 2, 0.0]) / 1000
    plane_y = sy * (C.w / 2 + C.e)

    def P(XZ):
        X, Z = XZ
        return np.array([sx * C.v / 2 + sx * X, plane_y, -C.h - Z]) / 1000

    return dict(
        sx=sx, sy=sy, theta_ref=(tA, tB, tAB),
        abad=abad, pivot=P((0.0, 0.0)), kneeA=P(lp["kneeA"]), kneeB=P(lp["kneeB"]),
        ankle=P(lp["ankle"]), foot=P(lp["foot_options"][0]),
    )


# ---------------------------------------------------------------------------
class ModelBuilder:
    def __init__(self, verbose=True):
        self.verbose = verbose
        self.stl: dict[str, trimesh.Trimesh] = {}
        self.vis_assets: dict[str, str] = {}     # asset name -> file
        self.mass_report: list[tuple] = []

    def log(self, *a):
        if self.verbose:
            print(*a)

    # -- meshes ------------------------------------------------------------
    def load_stls(self):
        for f in sorted(C.STL_DIR.glob("*/*.stl")):
            if f.parent.name == "Tools":
                continue
            name = re.sub(r"_\(x\d+\)$", "", f.stem)
            self.stl[name] = trimesh.load(f, force="mesh")
        self.log(f"loaded {len(self.stl)} STLs")

    def visual_asset(self, stl_name: str, mirrored: bool) -> str:
        key = stl_name + ("_mirror" if mirrored else "")
        if key in self.vis_assets:
            return key
        mesh = self.stl[stl_name].copy()
        if mirrored:
            mesh.vertices[:, 0] *= -1
            mesh.invert()
        if len(mesh.faces) > MAX_VIS_FACES:
            import fast_simplification
            red = 1 - MAX_VIS_FACES / len(mesh.faces)
            vs, fs = fast_simplification.simplify(mesh.vertices, mesh.faces, target_reduction=red)
            mesh = trimesh.Trimesh(vs, fs, process=False)
            cell = 0.4  # mm; vertex clustering fallback for meshes QEM can't reduce (rope grooves)
            while len(mesh.faces) > 1.5 * MAX_VIS_FACES and cell < 5:
                mesh = _cluster_decimate(mesh, cell)
                cell *= 1.5
        C.ASSET_DIR.mkdir(parents=True, exist_ok=True)
        path = C.ASSET_DIR / f"{key}.stl"
        mesh.export(path)
        self.vis_assets[key] = path.name
        return key

    # -- classification ---------------------------------------------------
    def classify(self, instances):
        """Return list of dicts: part placements grouped into (body, transform)."""
        parts = []
        motors = []    # (leg, center_sim, axis_sim)
        for inst in instances:
            if not inst.is_leaf:
                continue
            path, prod = inst.path, inst.product
            Ts = step_to_sim(inst.T)
            top = path[1] if len(path) > 1 else None
            leg = C.STEP_LEG_INSTANCES.get(top)
            if prod == "12_Slot_Stator" and leg is not None:
                center = (Ts @ np.array([0, 0, 0.00875, 1]))[:3]
                motors.append((leg, center, Ts[:3, 2]))
                continue
            stl_name = PRODUCT_TO_STL.get(prod, prod)
            if stl_name not in self.stl:
                continue
            if leg is None:
                seg = "base"
            elif prod in SEGMENT_OF:
                seg = SEGMENT_OF[prod]
            else:
                seg = None  # motor-assembly printed parts / controller covers: decided below
            parts.append(dict(leg=leg, prod=prod, stl=stl_name, T=Ts, seg=seg, path=path))

        # A motor whose spin axis is along sim x is the (body-fixed) ABAD motor;
        # A/B motors (axis along y) ride on the carriage.
        def motor_segment(m_axis):
            return "base" if abs(m_axis[0]) > 0.9 else "carriage"

        for p in parts:
            if p["seg"] is not None:
                continue
            centroid = (p["T"] @ np.r_[self.stl[p["stl"]].centroid / 1000, 1])[:3]
            legm = [m for m in motors if m[0] == p["leg"]]
            nearest = min(legm, key=lambda m: np.linalg.norm(m[1] - centroid))
            p["seg"] = motor_segment(nearest[2])
        motor_list = [dict(leg=l, center=cen, axis=ax, seg=motor_segment(ax)) for l, cen, ax in motors]
        return parts, motor_list

    # -- build ---------------------------------------------------------------
    def build(self):
        self.load_stls()
        sf = StepFile(C.STEP_FILE)
        instances = sf.instances()
        parts, motors = self.classify(instances)
        self.log(f"{len(parts)} printed-part instances, {len(motors)} motors")

        legs = [leg_reference_geometry(i) for i in range(4)]
        self.check_cad_consistency(parts, legs)

        def body_key(p_or_m):
            return "base" if p_or_m["seg"] == "base" else (p_or_m["leg"], p_or_m["seg"])

        def body_origin(key):
            if key == "base":
                return np.zeros(3)
            leg, seg = key
            g = legs[leg]
            return {"carriage": g["abad"], "upperA": g["pivot"], "upperB": g["pivot"],
                    "lowerA": g["kneeA"], "lowerB": g["kneeB"]}[seg]

        # ---- mass elements (world/sim frame) ----
        elems = defaultdict(list)
        printed_total = 0.0
        for p in parts:
            el = printed_part_element(self.stl[p["stl"]], p["prod"])
            el = el.transformed(p["T"][:3, :3], p["T"][:3, 3])
            elems[body_key(p)].append(el)
            printed_total += el.mass
        self.printed_raw = printed_total
        if C.PRINTED_MASS_TARGET:
            k_print = C.PRINTED_MASS_TARGET / printed_total
            for els in elems.values():
                for el in els:
                    el.mass *= k_print
                    el.inertia = el.inertia * k_print
            printed_total = C.PRINTED_MASS_TARGET
        PM = C.PURCHASED_MASS
        for mo in motors:
            elems[body_key(mo)].append(point_cylinder(PM["motor"], mo["center"], mo["axis"], 0.028, 0.025, "motor"))
            # XDrive Mini is mounted directly on the motor (reads the shaft magnet)
            elems[body_key(mo)].append(point_box(PM["xdrive"], mo["center"], (0.05, 0.05, 0.01), "xdrive"))
        for i, g in enumerate(legs):
            elems[(i, "lowerB")].append(point_box(PM["squash_ball"], g["foot"], (0.04, 0.04, 0.04), "ball"))
        elems["base"].append(point_box(PM["battery"] + PM["battery_adapter"], np.array([0.005, 0.011, 0.095]),
                                       (0.14, 0.09, 0.06), "battery"))
        elems["base"].append(point_box(PM["electronics"], np.array([0.17, 0.0, 0.09]), (0.12, 0.08, 0.03), "electronics"))
        elems["base"].append(point_box(PM["wiring"], np.array([0.0, 0.0, 0.05]), (0.3, 0.1, 0.05), "wiring"))
        # fasteners & bearings distributed proportionally to printed mass
        printed_by_body = {k: sum(e.mass for e in v if e.label not in
                                  ("motor", "xdrive", "ball", "battery", "electronics", "wiring"))
                           for k, v in elems.items()}
        extra = PM["fasteners"] + PM["bearings"]
        for k, pm in printed_by_body.items():
            if pm <= 0:
                continue
            body_el = combine([e for e in elems[k]])
            elems[k].append(MassElement(extra * pm / printed_total, body_el.com.copy(),
                                        np.zeros((3, 3)), "hardware"))

        inertials = {}
        for k, els in elems.items():
            tot = combine(els)
            tot.mass *= C.MASS_SCALE
            tot.inertia *= C.MASS_SCALE
            tot.com = tot.com - body_origin(k)
            inertials[k] = tot
        self.inertials = inertials
        self.printed_total = printed_total
        self.total_mass = sum(t.mass for t in inertials.values())

        # ---- geoms per body ----
        geoms = defaultdict(list)
        for p in parts:
            k = body_key(p)
            T = p["T"]
            mirrored = np.linalg.det(T[:3, :3]) < 0
            Rm = T[:3, :3].copy()
            if mirrored:
                Rm[:, 0] *= -1
            asset = self.visual_asset(p["stl"], mirrored)
            pos = T[:3, 3] - body_origin(k)
            color = COLORS["motor"] if p["prod"] in ("Short_Small_Drum_Bottom", "Short_Small_Drum_Top",
                                                     "Tall_Small_Drum_Bottom", "Tall_Small_Drum_Top",
                                                     "Small_Drum_Shaft_Adapter") else COLORS[p["seg"]]
            geoms[k].append(f'<geom class="visual" mesh="{asset}" pos="{fmt(pos)}" quat="{fmt(mat2quat(Rm))}" rgba="{color}"/>')
        for mo in motors:
            k = body_key(mo)
            a = mo["axis"] / np.linalg.norm(mo["axis"])
            c0 = mo["center"] - body_origin(k)
            geoms[k].append(f'<geom class="visual" type="cylinder" size="0.0285 0.0125" '
                            f'fromto="{fmt(c0 - a * 0.0125)} {fmt(c0 + a * 0.0125)}" rgba="{COLORS["motor"]}"/>')
        self.parts, self.motors, self.legs = parts, motors, legs
        return self.emit_xml(geoms, legs, inertials, parts)

    # -- consistency check ----------------------------------------------------
    def check_cad_consistency(self, parts, legs):
        """Compare analytic joint locations with the STEP frames of the link parts."""
        worst = 0.0
        for p in parts:
            if p["leg"] is None:
                continue
            g = legs[p["leg"]]
            if p["prod"] == "Lower_Link_A":
                err = np.linalg.norm(p["T"][:3, 3] - g["kneeA"])
            elif p["prod"] == "Lower_Link_B":
                err = np.linalg.norm(p["T"][:3, 3] - g["ankle"])
            else:
                continue
            worst = max(worst, err)
        self.cad_error_mm = worst * 1000
        self.log(f"CAD vs firmware-IK joint location error: max {self.cad_error_mm:.2f} mm")

    # -- XML -------------------------------------------------------------------
    def emit_xml(self, geoms, legs, inertials, parts) -> str:
        tau_max = C.ODRIVE["current_lim"] * C.MOTOR_KT * C.gr * C.CAPSTAN_EFFICIENCY
        armature = C.MOTOR_ROTOR_INERTIA * C.gr ** 2
        X = []
        a = X.append
        a('<mujoco model="CARA 2.0">')
        a(f'  <compiler angle="radian" meshdir="assets" inertiafromgeom="false" autolimits="true"/>')
        a(f'  <option timestep="{C.PHYSICS_DT}" integrator="implicitfast" cone="elliptic" impratio="10"/>')
        a('  <size memory="64M"/>')
        a('  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096"/>'
          '<headlight ambient="0.35 0.35 0.35" diffuse="0.6 0.6 0.6"/></visual>')
        a('  <default>')
        a('    <geom contype="0" conaffinity="0"/>')
        a('    <default class="visual"><geom type="mesh" group="1" mass="0" contype="0" conaffinity="0"/></default>')
        a('    <default class="collision"><geom group="3" contype="2" conaffinity="1" rgba="0.9 0.3 0.3 0.25" '
          'friction="0.8 0.02 0.001"/></default>')
        a(f'    <default class="foot"><geom type="sphere" size="{C.FOOT_RADIUS}" group="3" contype="2" conaffinity="1" '
          f'friction="{C.FOOT_FRICTION} 0.02 0.001" solref="{fmt(C.FOOT_SOLREF)}" priority="1" rgba="0.95 0.85 0.2 1"/></default>')
        a(f'    <default class="actuated"><joint armature="{armature:.6g}" damping="{C.JOINT_DAMPING}" '
          f'frictionloss="{C.JOINT_FRICTIONLOSS}"/></default>')
        a('    <default class="passive"><joint damping="0.002" armature="0.00001"/></default>')
        a('  </default>')
        a('  <asset>')
        a('    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.32 0.34 0.38" rgb2="0.26 0.28 0.31"/>')
        a('    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance="0.1"/>')
        a('    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.55 0.65 0.8" rgb2="0.1 0.1 0.15" width="512" height="512"/>')
        for name, file in sorted(self.vis_assets.items()):
            a(f'    <mesh name="{name}" file="{file}" scale="0.001 0.001 0.001"/>')
        a('  </asset>')
        a('  <worldbody>')
        a('    <light pos="0 0 3" dir="0 0 -1" directional="true" diffuse="0.6 0.6 0.6" castshadow="true"/>')
        a('    <geom name="floor" type="plane" size="0 0 0.05" material="grid" contype="1" conaffinity="2" '
          'friction="1 0.02 0.001"/>')

        def inertial(k, ind):
            t = inertials[k]
            I = t.inertia
            return (f'{ind}<inertial pos="{fmt(t.com)}" mass="{t.mass:.6g}" '
                    f'fullinertia="{fmt([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]])}"/>')

        z0 = (C.zDefault + C.FOOT_RADIUS * 1000) / 1000
        a(f'    <body name="base" pos="0 0 {z0 + 0.01:.4f}">')
        a('      <freejoint name="root"/>')
        a(inertial("base", "      "))
        a('      <site name="imu" pos="0 0 0.09" size="0.008" rgba="0.2 0.8 0.2 1"/>')
        a('      <camera name="track" mode="trackcom" pos="0.9 -1.2 0.6" xyaxes="0.8 0.6 0 -0.25 0.33 0.91"/>')
        # base collision: core + e-box
        base_pts = []
        for p in parts:
            if p["seg"] == "base" and p["leg"] is None:
                vv = (p["T"][:3, :3] @ (self.stl[p["stl"]].bounds.T / 1000)).T + p["T"][:3, 3]
                base_pts.append(vv)
        bp = np.vstack(base_pts)
        lo, hi = bp.min(0), bp.max(0)
        a(f'      <geom name="base_col" class="collision" type="box" pos="{fmt((lo + hi) / 2)}" size="{fmt((hi - lo) / 2)}"/>')
        for g in geoms["base"]:
            a("      " + g)

        for i, g in enumerate(legs):
            L = C.LEG_NAMES[i]
            sx, sy = g["sx"], g["sy"]
            ab, pv, kA, kB, an, ft = g["abad"], g["pivot"], g["kneeA"], g["kneeB"], g["ankle"], g["foot"]
            ranges = {}
            for j, jn in enumerate(C.JOINT_NAMES):
                lo_d, hi_d = C.JOINT_RANGE_DEG[jn]
                ref = g["theta_ref"][j] * 2 * math.pi
                ranges[jn] = (math.radians(lo_d) - ref, math.radians(hi_d) - ref)
            # carriage collision box from carriage parts
            cpts = [((p["T"][:3, :3] @ (self.stl[p["stl"]].bounds.T / 1000)).T + p["T"][:3, 3])
                    for p in parts if p["leg"] == i and p["seg"] == "carriage"]
            cp = np.vstack(cpts)
            clo, chi = cp.min(0) - ab, cp.max(0) - ab
            a(f'      <body name="{L}_carriage" pos="{fmt(ab)}">')
            a(f'        <joint name="{L}_ABAD" class="actuated" axis="{sy} 0 0" range="{fmt(ranges["ABAD"])}"/>')
            a(inertial((i, "carriage"), "        "))
            a(f'        <geom name="{L}_carriage_col" class="collision" type="box" pos="{fmt((clo + chi) / 2)}" size="{fmt((chi - clo) / 2)}"/>')
            for gg in geoms[(i, "carriage")]:
                a("        " + gg)
            # upper A branch
            a(f'        <body name="{L}_upperA" pos="{fmt(pv - ab)}">')
            a(f'          <joint name="{L}_A" class="actuated" axis="0 {-sx} 0" range="{fmt(ranges["A"])}"/>')
            a(inertial((i, "upperA"), "          "))
            a(f'          <geom class="collision" type="capsule" size="0.008" fromto="0 0 0 {fmt(kA - pv)}"/>')
            for gg in geoms[(i, "upperA")]:
                a("          " + gg)
            a(f'          <body name="{L}_lowerA" pos="{fmt(kA - pv)}">')
            a(f'            <joint name="{L}_kneeA" class="passive" axis="0 1 0"/>')
            a(inertial((i, "lowerA"), "            "))
            a(f'            <geom class="collision" type="capsule" size="0.007" fromto="0 0 0 {fmt(an - kA)}"/>')
            a(f'            <site name="{L}_ankleA" pos="{fmt(an - kA)}" size="0.004" rgba="1 0 0 1"/>')
            for gg in geoms[(i, "lowerA")]:
                a("            " + gg)
            a('          </body>')
            a('        </body>')
            # upper B branch
            a(f'        <body name="{L}_upperB" pos="{fmt(pv - ab)}">')
            a(f'          <joint name="{L}_B" class="actuated" axis="0 {sx} 0" range="{fmt(ranges["B"])}"/>')
            a(inertial((i, "upperB"), "          "))
            a(f'          <geom class="collision" type="capsule" size="0.008" fromto="0 0 0 {fmt(kB - pv)}"/>')
            for gg in geoms[(i, "upperB")]:
                a("          " + gg)
            a(f'          <body name="{L}_lowerB" pos="{fmt(kB - pv)}">')
            a(f'            <joint name="{L}_kneeB" class="passive" axis="0 1 0"/>')
            a(inertial((i, "lowerB"), "            "))
            a(f'            <geom class="collision" type="capsule" size="0.008" fromto="0 0 0 {fmt(an - kB)}"/>')
            a(f'            <geom class="collision" type="capsule" size="0.008" fromto="{fmt(an - kB)} {fmt(ft - kB)}"/>')
            a(f'            <geom name="{L}_foot" class="foot" pos="{fmt(ft - kB)}"/>')
            a(f'            <geom class="visual" type="sphere" size="{C.FOOT_RADIUS}" pos="{fmt(ft - kB)}" rgba="0.08 0.08 0.08 1"/>')
            a(f'            <site name="{L}_foot" pos="{fmt(ft - kB)}" size="0.005" rgba="0 0 1 1"/>')
            a(f'            <site name="{L}_ankleB" pos="{fmt(an - kB)}" size="0.004" rgba="0 1 0 1"/>')
            for gg in geoms[(i, "lowerB")]:
                a("            " + gg)
            a('          </body>')
            a('        </body>')
            a('      </body>')
        a('    </body>')
        a('  </worldbody>')

        a('  <contact>')
        a('  </contact>')
        a('  <equality>')
        for i, g in enumerate(legs):
            L = C.LEG_NAMES[i]
            a(f'    <connect name="{L}_5bar" body1="{L}_lowerA" body2="{L}_lowerB" '
              f'anchor="{fmt(g["ankle"] - g["kneeA"])}" solref="0.002 1" solimp="0.95 0.99 0.001"/>')
        a('  </equality>')
        a('  <actuator>')
        for i in range(4):
            L = C.LEG_NAMES[i]
            for jn in C.JOINT_NAMES:
                a(f'    <motor name="{L}_{jn}_motor" joint="{L}_{jn}" ctrlrange="{-tau_max:.4g} {tau_max:.4g}"/>')
        a('  </actuator>')
        a('  <sensor>')
        a('    <framequat name="imu_quat" objtype="site" objname="imu"/>')
        a('    <gyro name="imu_gyro" site="imu"/>')
        a('    <accelerometer name="imu_acc" site="imu"/>')
        a('  </sensor>')
        a('  <keyframe>')
        a(f'    <key name="stand" qpos="0 0 {z0:.4f} 1 0 0 0 {" ".join(["0"] * 20)}"/>')
        a('  </keyframe>')
        a('</mujoco>')
        return "\n".join(X)

    # -- report ---------------------------------------------------------------
    def report(self) -> str:
        lines = ["| Body | Mass (kg) |", "|---|---|"]
        for k, t in sorted(self.inertials.items(), key=lambda kv: str(kv[0])):
            name = k if k == "base" else f"{C.LEG_NAMES[k[0]]}_{k[1]}"
            lines.append(f"| {name} | {t.mass:.3f} |")
        lines.append(f"| **Total** | **{self.total_mass:.3f}** |")
        lines.append("")
        lines.append(f"Printed PLA: raw shell+infill model {self.printed_raw:.3f} kg, "
                     f"calibrated to {self.printed_total:.3f} kg")
        return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    b = ModelBuilder(verbose=not args.quiet)
    xml = b.build()
    C.MJCF_FILE.write_text(xml)
    (C.MODEL_DIR / "mass_report.md").write_text(b.report() + f"\n\nCAD vs IK joint error: {b.cad_error_mm:.2f} mm\n")
    print(b.report())
    print(f"wrote {C.MJCF_FILE}")


if __name__ == "__main__":
    main()
