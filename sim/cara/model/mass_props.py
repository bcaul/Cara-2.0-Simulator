"""Mass / inertia estimation for 3D-printed and purchased parts."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import trimesh

from .. import config as C


@dataclass
class MassElement:
    """A rigid mass element expressed in some frame."""
    mass: float                 # kg
    com: np.ndarray             # (3,) m
    inertia: np.ndarray         # (3,3) kg m^2 about com, frame axes
    label: str = ""

    def transformed(self, R: np.ndarray, p: np.ndarray) -> "MassElement":
        return MassElement(self.mass, R @ self.com + p, R @ self.inertia @ R.T, self.label)


def printed_part_mass(mesh: trimesh.Trimesh) -> float:
    """Shell + infill model of an FDM part (mesh in mm). Returns kg."""
    vol = abs(mesh.volume) if mesh.is_watertight else abs(mesh.convex_hull.volume) * 0.6
    area = mesh.area
    vol_m3, area_m2 = vol * 1e-9, area * 1e-6
    shell = min(area_m2 * C.PRINT_SHELL_THICKNESS, vol_m3)
    core = max(vol_m3 - shell, 0.0)
    return C.PLA_DENSITY * (shell + C.PRINT_INFILL * core)


def printed_part_element(mesh: trimesh.Trimesh, label="") -> MassElement:
    """Mass element in the mesh's own frame (metres), uniform-density inertia shape."""
    mass = printed_part_mass(mesh)
    src = mesh if mesh.is_watertight else mesh.convex_hull
    vol = abs(src.volume)
    com = np.asarray(src.center_mass) * 1e-3
    # trimesh inertia is for density=1 in mm^5 about the centre of mass
    I_unit = np.asarray(src.moment_inertia) / vol  # mm^2 per unit mass
    inertia = I_unit * 1e-6 * mass
    return MassElement(mass, com, inertia, label)


def point_cylinder(mass, center, axis, radius, length, label="") -> MassElement:
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    Ia = 0.5 * mass * radius ** 2
    It = mass * (3 * radius ** 2 + length ** 2) / 12
    I = It * np.eye(3) + (Ia - It) * np.outer(axis, axis)
    return MassElement(mass, np.asarray(center, float), I, label)


def point_box(mass, center, size, label="") -> MassElement:
    sx, sy, sz = size
    I = np.diag([mass * (sy ** 2 + sz ** 2) / 12, mass * (sx ** 2 + sz ** 2) / 12,
                 mass * (sx ** 2 + sy ** 2) / 12])
    return MassElement(mass, np.asarray(center, float), I, label)


def combine(elements: list[MassElement]) -> MassElement:
    M = sum(e.mass for e in elements)
    if M <= 0:
        return MassElement(0.0, np.zeros(3), np.zeros((3, 3)))
    com = sum(e.mass * e.com for e in elements) / M
    I = np.zeros((3, 3))
    for e in elements:
        r = e.com - com
        I += e.inertia + e.mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return MassElement(M, com, I)
