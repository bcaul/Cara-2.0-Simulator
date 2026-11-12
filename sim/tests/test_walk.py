"""Phase 3: full RC session through the firmware's getReceiver() state machine.

Hold button -> home (robot held in the air) -> set down -> hold button -> stand ->
walk forward 5 s -> spin CW -> low stance -> back up -> hold button -> crouch.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

from cara.sim_backend.sim_hal import Simulation
from cara.teleop.rc_transmitter import ScriptedTransmitter

TIMELINE = [
    (0.5, dict(button=True)), (1.7, dict(button=False)),      # home
    (6.0, dict(button=True)), (7.2, dict(button=False)),      # stand
    (9.0, dict(right_y=1.0)), (14.0, dict(right_y=0.0)),      # walk forward 5 s
    (15.0, dict(left_x=1.0)), (18.0, dict(left_x=0.0)),       # spin CW 3 s
    (19.0, dict(sw5=-1)), (21.0, dict(sw5=0)),                # low stance, then default
    (23.0, dict(button=True)), (24.2, dict(button=False)),    # crouch (state 1)
]


@pytest.fixture(scope="module")
def session():
    sim = Simulation(held_height=0.5, seed=5)
    tx = ScriptedTransmitter(TIMELINE, clock=lambda: sim.time)
    sim.rc_source = tx.widths
    sim.start()
    log, set_down = [], False
    while sim.time < 26 and sim.hal.running:
        if not set_down and sim.fw.state == 1:
            sim.hand.lower_to_floor()
            set_down = True
        sim.step()
        if sim.step_count % 100 == 0:
            R = sim.d.body("base").xmat
            r, p = sim.roll_pitch_deg()
            log.append((sim.time, sim.d.qpos[0], sim.d.qpos[1], sim.d.qpos[2],
                        math.degrees(math.atan2(R[3], R[0])), r, p, sim.fw.state, sim.fw.zNom))
    assert sim.hal.error is None
    return sim, np.array(log)


def at(log, t):
    return log[np.searchsorted(log[:, 0], t)]


def test_states(session):
    _, log = session
    assert at(log, 5.5)[7] == 1          # homed, crouched
    assert at(log, 8.5)[7] == 3          # standing
    assert at(log, 25.9)[7] == 1         # crouched again


def test_walks_forward(session):
    _, log = session
    a, b = at(log, 9.0), at(log, 14.5)
    seg = log[(log[:, 0] > 9) & (log[:, 0] < 14.5)]
    dx = (b[1] - a[1]) * math.cos(math.radians(a[4])) + (b[2] - a[2]) * math.sin(math.radians(a[4]))
    print(f"forward {dx:.2f} m in 5 s, |roll| {np.abs(seg[:, 5]).max():.1f}, |pitch| {np.abs(seg[:, 6]).max():.1f}")
    assert dx > 0.3
    assert np.abs(seg[:, 5]).max() < 20 and np.abs(seg[:, 6]).max() < 20


def test_spins_clockwise(session):
    _, log = session
    dyaw = (at(log, 18.5)[4] - at(log, 15.0)[4] + 180) % 360 - 180
    print(f"yaw change {dyaw:.1f} deg")
    assert dyaw < -30                    # clockwise seen from above


def test_low_stance_switch(session):
    _, log = session
    assert at(log, 20.5)[8] == 200       # zNom = zLow
    assert abs(at(log, 20.5)[3] * 1000 - 220) < 15
    assert at(log, 22.5)[8] == 280


def test_no_faults(session):
    sim, _ = session
    assert sim.bank.overspeed_trips == 0
    assert sim.bank.nan_commands == 0
