"""Phase 3: virtual transmitter -> pulse widths -> ported isrHandler -> chnVal / criteria."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cara.firmware.cara_firmware import CaraFirmware
from cara.teleop.rc_transmitter import RCState


class _NullHAL:
    odrive = [None] * 12
    imu = None
    def millis(self): return 0
    def micros(self): return 0


def decode(**kw):
    fw = CaraFirmware(_NullHAL())
    fw.isrHandler(RCState(**kw).widths())
    return fw


def test_neutral_sticks_do_nothing():
    fw = decode()
    assert fw.chnVal[4:7] == [0, 0, 0]
    assert all(abs(v) < 6 for v in fw.chnVal[:4])
    assert not any(fw.criteria(n) for n in range(10))
    assert fw.width[2] > 1000                   # transmitter on


def test_walk_directions_match_firmware_criteria():
    cases = {0: dict(right_y=1), 1: dict(right_y=-1), 2: dict(right_x=1), 3: dict(right_x=-1),
             4: dict(right_x=1, right_y=1), 5: dict(right_x=1, right_y=-1),
             6: dict(right_x=-1, right_y=1), 7: dict(right_x=-1, right_y=-1),
             8: dict(left_x=1), 9: dict(left_x=-1)}
    for num, kw in cases.items():
        fw = decode(**kw)
        active = [n for n in range(10) if fw.criteria(n)]
        assert active == [num], (kw, fw.chnVal, active)


def test_switches_button_and_tilt_sticks():
    assert decode(sw5=1).chnVal[4] == 1
    assert decode(sw5=-1).chnVal[4] == -1
    assert decode(sw7=1).chnVal[6] == 1
    assert decode(sw7=-1).chnVal[6] == -1
    assert decode(button=True).chnVal[5] == 1
    assert decode(left_y=1).chnVal[2] == 10
    assert decode(left_y=-1).chnVal[2] == -10
    assert decode(knob=100).chnVal[7] == 100


def test_transmitter_off_reads_disconnected():
    fw = decode(on=False)
    assert fw.width[2] < 1000
