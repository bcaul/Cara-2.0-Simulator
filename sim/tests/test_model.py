"""Phase 1 tests: model compiles, closed chain is exact, geometry matches the firmware
kinematics, and the robot stands at zNom = 280 mm."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import mujoco
import numpy as np
import pytest

from cara import config as C
from cara.firmware.kinematics import getIK
from cara.sim_backend.model_utils import chain_error_mm, foot_in_leg_frame, set_leg_pose


@pytest.fixture(scope="module")
def model():
    from cara.model import load_model
    return load_model()


def test_compiles(model):
    assert model.nu == 12
    assert model.neq == 4                    # one 5-bar loop per leg
    assert model.njnt == 1 + 4 * 5           # free + (ABAD, A, B, kneeA, kneeB) x 4
    total = float(sum(model.body_mass))
    print(f"total mass {total:.3f} kg")
    assert 5.0 < total < 10.0


def test_closed_chain_and_geometry_match_firmware_ik(model):
    """Set joints from getIK for random targets; the loop must close and the
    simulated foot must land on the IK target (all four legs)."""
    d = mujoco.MjData(model)
    rng = np.random.default_rng(1)
    checked = 0
    for _ in range(300):
        x, y, z = rng.uniform(-70, 70), C.e + rng.uniform(-30, 30), rng.uniform(160, 300)
        try:
            th = getIK(x, y, z)
        except ValueError:          # outside workspace (acos domain)
            continue
        mujoco.mj_resetDataKeyframe(model, d, 0)
        for leg in range(4):
            set_leg_pose(model, d, leg, *th)
        mujoco.mj_forward(model, d)
        assert chain_error_mm(model, d) < 0.01
        for leg in range(4):
            assert np.linalg.norm(foot_in_leg_frame(model, d, leg) - [x, y, z]) < 0.01
        checked += 1
    assert checked > 200


def test_stands_at_nominal_height():
    from stand import run
    s = run(seconds=10.0, viewer=False, verbose=True)
    expected = C.zDefault + C.FOOT_RADIUS * 1000      # ABAD axis height above floor
    assert abs(s["height_mean_mm"] - expected) < 10
    assert s["height_max_mm"] - s["height_min_mm"] < 5
    assert s["chain_err_max_mm"] < 0.5
    assert s["upright"] > 0.99
