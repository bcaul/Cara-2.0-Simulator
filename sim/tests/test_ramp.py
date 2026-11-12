"""Phase 2: Ramp library port reproduces the C++ behaviour (grain, timing, easing)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest

from cara.firmware.ramp import CUBIC_OUT, LINEAR, NONEE, QUADRATIC_OUT, ramp_calc, rampFloat


class Clock:
    def __init__(self):
        self.t = 0

    def __call__(self):
        return self.t


def test_linear_ramp_with_5ms_loop_updates_every_10ms_grain():
    clk = Clock()
    r = rampFloat(clk)
    r.go(0)
    r.go(100, 50, LINEAR)
    vals = []
    while r.isRunning():
        clk.t += 5                     # the firmware's delay(5)
        vals.append(r.update())
    assert vals == [0, 20, 20, 40, 40, 60, 60, 80, 80, 100]
    assert clk.t == 50


def test_late_update_jumps_by_elapsed_time():
    clk = Clock()
    r = rampFloat(clk)
    r.go(0)
    r.go(1, 100, LINEAR)
    clk.t = 23
    assert r.update() == pytest.approx(0.23)


def test_go_without_duration_is_instant_and_finished():
    clk = Clock()
    r = rampFloat(clk)
    assert r.go(1000) == 1000          # moveFeet's rampCoord[i].go(OFF)
    assert not r.isRunning()
    assert r.mode == NONEE
    assert r.update() == 1000


def test_descending_ramp_and_easings_match_cpp_formulas():
    assert ramp_calc(0.5, CUBIC_OUT) == pytest.approx(1 - 0.5 ** 3)
    assert ramp_calc(0.5, QUADRATIC_OUT) == pytest.approx(0.75)
    clk = Clock()
    r = rampFloat(clk, 280.0)
    r.go(160, 100, CUBIC_OUT)
    clk.t = 50
    # C++: val = A - (A-B)*ramp_calc(k)  when B < A
    assert r.update() == pytest.approx(280 - 120 * (1 - 0.5 ** 3))


def test_restarting_rampT_like_step_does():
    clk = Clock()
    r = rampFloat(clk)
    r.go(0.5)
    r.go(1, 50, LINEAR)
    clk.t += 10
    assert r.update() == pytest.approx(0.6)
    r.update()                         # second call in the same loop iteration: no change
    assert r.getValue() == pytest.approx(0.6)
