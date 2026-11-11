"""Single-leg kinematics ported 1:1 from CARA_2.0_Code/Functions.ino.

getIK(x, y, z)            <- void getIK(int leg, float x, float y, float z)
getFK(thetaA, thetaB, thetaABAD) <- body of the for-loop in void getFK()

Units match the firmware: coordinates in mm (leg frame: x fore/aft, y outward,
z downward from the ABAD axis), joint angles in REVOLUTIONS.
The firmware writes into global arrays (thetaIK, coordFK); here the functions
return tuples and CaraFirmware stores them in the same-named arrays.
"""
from math import acos, asin, atan, cos, pi as PI, sin, sqrt

from ..config import OFF, c, d, e, h, m, phi


def sq(x):
    return x * x


def getIK(x, y, z):
    """Return (thetaA, thetaB, thetaABAD) in revs, or (OFF, OFF, OFF)."""
    if x == OFF and y == OFF and z == OFF:
        return OFF, OFF, OFF
    # Calculate the intermediate variables
    n = sqrt(sq(d) + sq(m) - 2 * d * m * cos(phi * PI / 180))  # mm
    l = sqrt(sq(y) + sq(z) - sq(e)) - h                        # mm
    f = sqrt(sq(x) + sq(l))                                    # mm
    theta2 = acos((sq(c) + sq(f) - sq(n)) / (2 * c * f))       # rads

    # Calculate joint ABAD and B's target positions
    thetaABAD = acos(e / sqrt(sq(y) + sq(z))) + atan(y / z) - PI / 2
    thetaB = theta2 - atan(x / l)

    # Calculate more intermediate variables
    theta8 = PI - thetaB + acos((sq(d) + sq(n) - sq(m)) / (2 * d * n)) - acos((sq(c) + sq(n) - sq(f)) / (2 * c * n))
    v = sqrt(sq(d * sin(theta8) - c * sin(thetaB)) + sq(d * cos(theta8) + c * cos(thetaB)))

    # Calculate joint A's target position
    thetaA = 2 * acos((sq(v) + sq(c) - sq(d)) / (2 * v * c)) - thetaB

    # Convert the joint positions from rads to revs
    return thetaA / (2 * PI), thetaB / (2 * PI), thetaABAD / (2 * PI)


def getFK(thetaA, thetaB, thetaABAD):
    """Joint positions in revs -> (x, y, z) in mm."""
    # Convert joint positions from revs to rads
    thetaA = thetaA * 2 * PI
    thetaB = thetaB * 2 * PI
    thetaABAD = thetaABAD * 2 * PI

    # Calculate the intermediate variables
    n = sqrt(sq(d) + sq(m) - 2 * d * m * cos(phi * PI / 180))
    theta11 = PI - asin(c * sin((thetaA + thetaB) / 2) / d) - (thetaA + thetaB) / 2 + acos((sq(d) + sq(n) - sq(m)) / (2 * d * n))
    f = sqrt(sq(c) + sq(n) - 2 * c * n * cos(theta11))
    theta1 = acos((sq(c) + sq(f) - sq(n)) / (2 * c * f)) - thetaB
    theta4 = thetaABAD + PI / 2 - atan((f * cos(theta1) + h) / e)
    g = sqrt(sq(f * cos(theta1) + h) + sq(e))

    # Calculate effector coordinates
    return f * sin(theta1), g * sin(theta4), g * cos(theta4)


# ---------------------------------------------------------------------------
# Geometric helpers for building the MuJoCo closed chain (not in firmware)
# ---------------------------------------------------------------------------
def linkage_points(thetaA_rad, thetaB_rad):
    """Planar 5-bar joint locations in the leg plane, origin at the A/B pivot,
    X = firmware x (fore/aft), Z = downward. Returns dict of (X, Z) in mm:
    kneeA, kneeB, ankle (lower-link junction P), foot."""
    kA = (c * sin(thetaA_rad), c * cos(thetaA_rad))      # A deflects toward +x
    kB = (-c * sin(thetaB_rad), c * cos(thetaB_rad))     # B deflects toward -x
    # ankle P: intersection of circles radius d about kA and kB, lower branch
    dx, dz = kB[0] - kA[0], kB[1] - kA[1]
    L = sqrt(dx * dx + dz * dz)
    a = L / 2
    hh = sqrt(max(d * d - a * a, 0.0))
    mx, mz = kA[0] + dx / 2, kA[1] + dz / 2
    # unit normal pointing "down" (+Z)
    nx, nz = -dz / L, dx / L
    if nz < 0:
        nx, nz = -nx, -nz
    P = (mx + hh * nx, mz + hh * nz)
    # foot: rigid extension of lower link B (kB -> P) by m, bent by (180 - phi)
    ux, uz = (P[0] - kB[0]) / d, (P[1] - kB[1]) / d
    out = []
    for s in (1, -1):
        ang = s * (PI - phi * PI / 180)
        fx = P[0] + m * (ux * cos(ang) - uz * sin(ang))
        fz = P[1] + m * (ux * sin(ang) + uz * cos(ang))
        out.append((fx, fz))
    return {"kneeA": kA, "kneeB": kB, "ankle": P, "foot_options": out}
