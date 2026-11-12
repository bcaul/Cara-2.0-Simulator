"""Virtual RC transmitter: keyboard / gamepad -> the 8 PWM pulse widths CARA's receiver outputs.

The widths go through the ported isrHandler() (map + constrain), exactly like the real
receiver signals. Stick directions are chosen so the firmware sees what the real
transmitter produces (e.g. right stick forward -> chnVal[1] = +10 -> walk forward).

Channels (index = firmware chnVal index):
  0 ch1  right stick X    right = walk right
  1 ch2  right stick Y    forward = walk forward (channel reversed on the real TX)
  2 ch3  left stick Y     full forward = roll demo, full back = pitch demo
  3 ch4  left stick X     right = spin CW, left = spin CCW
  4 ch5  3-pos switch     forward = step in place / re-arm, middle, back = low stance / idle
  5 ch6  push button      hold 1 s = home -> stand -> sit
  6 ch7  3-pos switch     forward + button = kinematicsDemo, back + button = jump
  7 ch8  knob             unused by the firmware
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field

FAILSAFE_CH3 = 900      # receiver output on ch3 when the transmitter is off (< 1000 -> "DISCONNECTED")


@dataclass
class RCState:
    right_x: float = 0.0    # -1 .. 1
    right_y: float = 0.0
    left_x: float = 0.0
    left_y: float = 0.0
    sw5: int = 0            # +1 forward, 0 middle, -1 back
    button: bool = False
    sw7: int = 0
    knob: float = 0.0       # 0 .. 100
    on: bool = True

    def widths(self):
        def stick(v):
            return 1500 + 500 * max(-1.0, min(1.0, v))
        if not self.on:
            return [0, 0, FAILSAFE_CH3, 0, 0, 0, 0, 0]
        return [stick(self.right_x),
                stick(-self.right_y),
                stick(self.left_y),
                stick(self.left_x),
                1500 - 500 * self.sw5,
                2000 if self.button else 1000,
                1500 - 500 * self.sw7,
                1000 + 10 * max(0.0, min(100.0, self.knob))]


class Transmitter:
    """Thread-safe holder of the current RCState; the simulation polls widths() every frame."""

    def __init__(self):
        self._lock = threading.Lock()
        self.state = RCState()

    def set(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self.state, k, v)

    def widths(self):
        with self._lock:
            return self.state.widths()


@dataclass
class ScriptedTransmitter:
    """Transmitter driven by a time-line [(t_seconds, {field: value}), ...] (for tests)."""
    timeline: list
    clock: object = None            # callable returning sim time (s)
    state: RCState = field(default_factory=RCState)
    _i: int = 0

    def widths(self):
        t = self.clock()
        while self._i < len(self.timeline) and self.timeline[self._i][0] <= t:
            for k, v in self.timeline[self._i][1].items():
                setattr(self.state, k, v)
            self._i += 1
        return self.state.widths()
