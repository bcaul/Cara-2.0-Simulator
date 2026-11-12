"""Phase 2: ported firmware + simulated ODrives: homing, standing, kinematicsDemo moves."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

from cara import config as C
from cara.firmware.kinematics import getFK
from cara.firmware.ramp import CUBIC_OUT, LINEAR
from cara.sim_backend.model_utils import foot_in_leg_frame, joint_theta
from cara.sim_backend.sim_hal import Simulation


@pytest.fixture(scope="module")
def homed_and_standing():
    sim = Simulation(held_height=0.5, seed=3)
    marks = {}

    def program(fw):
        fw.setup()
        t0 = fw.millis()
        fw.home()
        marks["home_s"] = (fw.millis() - t0) / 1000
        fw.state = 1
        fw.delay(200)
        est = sim.bank.last_pos * np.array(C.incDir) / C.gr - np.array(fw.thetaOffset)
        true = np.array([joint_theta(sim.m, sim.d, L) for L in range(4)]).ravel()
        marks["offset_err_deg"] = (est - true) * 360
        marks["feet_start"] = [foot_in_leg_frame(sim.m, sim.d, L) for L in range(4)]
        sim.hand.lower_to_floor()
        while sim.hand.active:
            fw.delay(10)
        fw.delay(300)
        n = (fw.xNom, fw.yNom, fw.zNom)
        fw.moveFeet(*(n * 4), LINEAR, 500)
        fw.state = 3
        fw.delay(1500)
        marks["stand_mm"] = sim.base_height_mm()
        fw.moveRobot(0, 0, -100, CUBIC_OUT, 1500)
        fw.delay(500)
        marks["low_mm"] = sim.base_height_mm()
        fw.moveRobot(0, 0, 0, CUBIC_OUT, 1000)
        fw.tiltRobot(0, 10, 0, CUBIC_OUT, 500)
        fw.delay(500)
        marks["pitch_deg"] = sim.roll_pitch_deg()[1]
        fw.tiltRobot(0, 0, 0, CUBIC_OUT, 500)
        fw.delay(500)

    sim.start(program)
    while sim.hal.running and sim.time < 30:
        sim.step()
    assert sim.hal.error is None
    marks["sim"] = sim
    return marks


def test_homing_finds_joint_zero(homed_and_standing):
    m = homed_and_standing
    print("homing took", m["home_s"], "s; offset errors (deg):", np.round(m["offset_err_deg"], 2))
    assert m["home_s"] < 6
    assert np.abs(m["offset_err_deg"]).max() < 1.0


def test_start_pose_after_homing(homed_and_standing):
    for f in homed_and_standing["feet_start"]:
        assert np.linalg.norm(f - [C.xStart, C.yStart, C.zStart]) < 5      # mm


def test_stands_and_squats_through_firmware(homed_and_standing):
    m = homed_and_standing
    print("stand", m["stand_mm"], "low", m["low_mm"])
    assert abs(m["stand_mm"] - (C.zDefault + 20)) < 10
    assert abs(m["low_mm"] - (C.zDefault - 100 + 20)) < 15


def test_tiltRobot_pitches_body(homed_and_standing):
    p = homed_and_standing["pitch_deg"]
    print("commanded pitch 10, IMU-convention pitch", p)
    assert 7 < abs(p) < 13


def test_no_odrive_faults(homed_and_standing):
    sim = homed_and_standing["sim"]
    assert sim.bank.overspeed_trips == 0
    assert sim.bank.nan_commands == 0
