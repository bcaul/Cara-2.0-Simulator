"""Helpers linking firmware joint angles (revs) to MuJoCo qpos for the CARA model."""
from __future__ import annotations

import math

import numpy as np

from .. import config as C
from ..firmware.kinematics import getIK, linkage_points

THETA_REF = getIK(0, C.e, C.zDefault)   # firmware angles (revs) at q = 0 (CAD pose)


def _plane_vec(XZ_from, XZ_to, sx):
    """Leg-plane vector -> sim-frame (x, z) components (y ignored)."""
    X = sx * (XZ_to[0] - XZ_from[0])
    Z = -(XZ_to[1] - XZ_from[1])
    return np.array([X, Z])


def _rot_about_y(u0, u):
    """Angle alpha so that rotating u0 about +y by alpha gives u (2D x,z vectors)."""
    # rotation about +y: x' = x cos + z sin ; z' = -x sin + z cos
    cross = u0[1] * u[0] - u0[0] * u[1]
    return math.atan2(cross, float(np.dot(u0, u)))


def leg_q_from_theta(leg: int, thetaA, thetaB, thetaABAD):
    """Firmware angles (revs) -> dict of MuJoCo joint positions for one leg."""
    sx = C.LEG_SX[leg]
    qA = (thetaA - THETA_REF[0]) * 2 * math.pi
    qB = (thetaB - THETA_REF[1]) * 2 * math.pi
    qAB = (thetaABAD - THETA_REF[2]) * 2 * math.pi
    ref = linkage_points(THETA_REF[0] * 2 * math.pi, THETA_REF[1] * 2 * math.pi)
    cur = linkage_points(thetaA * 2 * math.pi, thetaB * 2 * math.pi)
    uA0 = _plane_vec(ref["kneeA"], ref["ankle"], sx)
    uA = _plane_vec(cur["kneeA"], cur["ankle"], sx)
    uB0 = _plane_vec(ref["kneeB"], ref["ankle"], sx)
    uB = _plane_vec(cur["kneeB"], cur["ankle"], sx)
    # upperA hinge axis (0,-sx,0): rotation about +y = -sx*qA ; upperB axis (0,sx,0): +sx*qB
    q_kneeA = _rot_about_y(uA0, uA) - (-sx * qA)
    q_kneeB = _rot_about_y(uB0, uB) - (sx * qB)
    L = C.LEG_NAMES[leg]
    return {f"{L}_A": qA, f"{L}_B": qB, f"{L}_ABAD": qAB, f"{L}_kneeA": q_kneeA, f"{L}_kneeB": q_kneeB}


def set_leg_pose(model, data, leg, thetaA, thetaB, thetaABAD):
    for name, q in leg_q_from_theta(leg, thetaA, thetaB, thetaABAD).items():
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = q


def joint_theta(model, data, leg):
    """MuJoCo joint positions -> firmware angles (revs) (thetaA, thetaB, thetaABAD)."""
    L = C.LEG_NAMES[leg]
    out = []
    for j, jn in enumerate(C.JOINT_NAMES):
        q = data.qpos[model.jnt_qposadr[model.joint(f"{L}_{jn}").id]]
        out.append(THETA_REF[j] + q / (2 * math.pi))
    return tuple(out)


def foot_in_leg_frame(model, data, leg):
    """Foot site position expressed in the firmware leg frame (mm), body-relative."""
    sx, sy = C.LEG_SX[leg], C.LEG_SY[leg]
    L = C.LEG_NAMES[leg]
    base = data.body("base")
    Rb = base.xmat.reshape(3, 3)
    p_world = data.site(f"{L}_foot").xpos
    p_body = Rb.T @ (p_world - base.xpos)
    abad = np.array([sx * C.v / 2, sy * C.w / 2, 0.0]) / 1000
    p = (p_body - abad) * 1000
    return np.array([sx * p[0], sy * p[1], -p[2]])


def chain_error_mm(model, data):
    return max(np.linalg.norm(data.site(f"{L}_ankleA").xpos - data.site(f"{L}_ankleB").xpos) * 1000
               for L in C.LEG_NAMES)
