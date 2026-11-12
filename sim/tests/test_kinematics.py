"""Phase 2: kinematics port checks (pure firmware math, no physics)."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

from cara import config as C
from cara.firmware.cara_firmware import CaraFirmware
from cara.firmware.kinematics import getFK, getIK


def workspace_grid():
    for x in np.linspace(-80, 80, 9):
        for dy in np.linspace(-30, 30, 7):
            for z in np.linspace(160, 320, 9):
                yield float(x), float(C.e + dy), float(z)


def test_ik_fk_round_trip_grid():
    """getFK(getIK(p)) == p over the gait/stance workspace."""
    worst, n = 0.0, 0
    for p in workspace_grid():
        try:
            th = getIK(*p)
        except ValueError:
            continue
        worst = max(worst, float(np.linalg.norm(np.array(getFK(*th)) - p)))
        n += 1
    print(f"{n} points, worst round-trip error {worst:.2e} mm")
    assert n > 450
    assert worst < 1e-3


def test_ik_matches_finite_difference_jacobian_is_well_conditioned_at_stand():
    """Sanity: the leg is not singular at the stand pose (small foot moves -> small joint moves)."""
    p0 = np.array([0.0, C.e, C.zDefault])
    J = np.zeros((3, 3))
    for k in range(3):
        dp = np.zeros(3); dp[k] = 0.01
        J[:, k] = (np.array(getIK(*(p0 + dp))) - np.array(getIK(*(p0 - dp)))) / 0.02
    cond = np.linalg.cond(J)
    print("dtheta/dp condition number at stand:", cond)
    assert np.isfinite(cond) and cond < 50


def test_off_passthrough():
    assert getIK(C.OFF, C.OFF, C.OFF) == (C.OFF, C.OFF, C.OFF)


class _NullHAL:
    odrive = [None] * 12
    imu = None
    def millis(self): return 0
    def micros(self): return 0


def test_getIKR_zero_is_nominal_and_pitch_tilts_front_back():
    fw = CaraFirmware(_NullHAL())
    fw.getIKR(0, 0, 0)
    for leg in range(4):
        assert fw.coordsIKR[leg * 3:leg * 3 + 3] == pytest.approx([fw.xNom, fw.yNom, fw.zNom], abs=1e-9)
    fw.getIKR(0, 10, 0)
    zF, zB = fw.coordsIKR[2], fw.coordsIKR[8]
    # 10 deg pitch over the 359 mm wheelbase: front/back legs differ by v*sin(10deg)
    assert abs((zB - zF) - C.v * math.sin(math.radians(10))) < 2.0


def test_firmware_getIK_keeps_nan_like_c():
    fw = CaraFirmware(_NullHAL())
    fw.getIK(0, 0, C.e, 1000)       # unreachable -> acos domain error -> NaN, not an exception
    assert all(math.isnan(t) for t in fw.thetaIK[:3])
