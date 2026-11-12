"""Port of the RAMP Arduino library (Sylvain Garnavault, RAMP-master.zip in the project).

Only the clock source changes: millis() comes from the HAL, so ramps run on the
simulator's virtual clock exactly as they do on the Teensy (10 ms grain included).
"""
from __future__ import annotations

from math import asin, cos, fabs, pi as M_PI, sin, sqrt

# ramp_mode
NONEE, LINEAR = 0x00, 0x01
QUADRATIC_IN, QUADRATIC_OUT, QUADRATIC_INOUT = 0x02, 0x03, 0x04
CUBIC_IN, CUBIC_OUT, CUBIC_INOUT = 0x05, 0x06, 0x07
QUARTIC_IN, QUARTIC_OUT, QUARTIC_INOUT = 0x08, 0x09, 0x0A
QUINTIC_IN, QUINTIC_OUT, QUINTIC_INOUT = 0x0B, 0x0C, 0x0D
SINUSOIDAL_IN, SINUSOIDAL_OUT, SINUSOIDAL_INOUT = 0x0E, 0x0F, 0x10
EXPONENTIAL_IN, EXPONENTIAL_OUT, EXPONENTIAL_INOUT = 0x11, 0x12, 0x13
CIRCULAR_IN, CIRCULAR_OUT, CIRCULAR_INOUT = 0x14, 0x15, 0x16
ELASTIC_IN, ELASTIC_OUT, ELASTIC_INOUT = 0x17, 0x18, 0x19
BACK_IN, BACK_OUT, BACK_INOUT = 0x1A, 0x1B, 0x1C
BOUNCE_IN, BOUNCE_OUT, BOUNCE_INOUT = 0x1D, 0x1E, 0x1F
# speed_mode
FORWARD, BACKWARD = 0x00, 0x01
# loop_mode
ONCEFORWARD, LOOPFORWARD, FORTHANDBACK, ONCEBACKWARD, LOOPBACKWARD, BACKANDFORTH = range(6)


def powin(k, p):
    return k ** p


def powout(k, p):
    return 1 - (1 - k) ** p


def powinout(k, p):
    k *= 2
    if k < 1:
        return 0.5 * k ** p
    return 1 - 0.5 * fabs((2 - k) ** p)


def ramp_calc(k, m):
    if k == 0 or k == 1:
        return k
    if m == QUADRATIC_IN: return powin(k, 2)
    if m == QUADRATIC_OUT: return powout(k, 2)
    if m == QUADRATIC_INOUT: return powinout(k, 2)
    if m == CUBIC_IN: return powin(k, 3)
    if m == CUBIC_OUT: return powout(k, 3)
    if m == CUBIC_INOUT: return powinout(k, 3)
    if m == QUARTIC_IN: return powin(k, 4)
    if m == QUARTIC_OUT: return powout(k, 4)
    if m == QUARTIC_INOUT: return powinout(k, 4)
    if m == QUINTIC_IN: return powin(k, 5)
    if m == QUINTIC_OUT: return powout(k, 5)
    if m == QUINTIC_INOUT: return powinout(k, 5)
    if m == SINUSOIDAL_IN: return 1 - cos(k * (M_PI / 2))
    if m == SINUSOIDAL_OUT: return sin(k * (M_PI / 2))
    if m == SINUSOIDAL_INOUT: return -0.5 * (cos(M_PI * k) - 1)
    if m == EXPONENTIAL_IN: return 2 ** (10 * (k - 1))
    if m == EXPONENTIAL_OUT: return 1 - 2 ** (-10 * k)
    if m == EXPONENTIAL_INOUT:
        k *= 2.0
        if k < 1:
            return 0.5 * 2 ** (10 * (k - 1))
        k -= 1
        return 0.5 * (2 - 2 ** (-10 * k))
    if m == CIRCULAR_IN: return -(sqrt(1 - k * k) - 1)
    if m == CIRCULAR_OUT:
        k -= 1
        return sqrt(1 - k * k)
    if m == CIRCULAR_INOUT:
        k *= 2
        if k < 1:
            return -0.5 * (sqrt(1 - k * k) - 1)
        k -= 2
        return 0.5 * (sqrt(1 - k * k) + 1)
    if m == ELASTIC_IN:
        k -= 1
        a, p = 1, 0.3 * 1.5
        s = p * asin(1 / a) / (2 * M_PI)
        return -a * 2 ** (10 * k) * sin((k - s) * (2 * M_PI) / p)
    if m == ELASTIC_OUT:
        a, p = 1, 0.3
        s = p * asin(1 / a) / (2 * M_PI)
        return a * 2 ** (-10 * k) * sin((k - s) * (2 * M_PI) / p) + 1
    if m == ELASTIC_INOUT:
        k = k * 2 - 1
        a, p = 1, 0.3 * 1.5
        s = p * asin(1 / a) / (2 * M_PI)
        if (k + 1) < 1:
            return -0.5 * a * 2 ** (10 * k) * sin((k - s) * (2 * M_PI) / p)
        return a * 2 ** (-10 * k) * sin((k - s) * (2 * M_PI) / p) * 0.5 + 1
    if m == BACK_IN:
        s = 1.70158
        return k * k * ((s + 1) * k - s)
    if m == BACK_OUT:
        k -= 1
        s = 1.70158
        return k * k * ((s + 1) * k + s) + 1
    if m == BACK_INOUT:
        k *= 2
        s = 1.70158 * 1.525
        if k < 1:
            return 0.5 * k * k * ((s + 1) * k - s)
        k -= 2
        return 0.5 * k * k * ((s + 1) * k + s) + 1
    if m == BOUNCE_IN:
        return 1 - ramp_calc(1 - k, BOUNCE_OUT)
    if m == BOUNCE_OUT:
        if k < (1 / 2.75):
            return 7.5625 * k * k
        if k < (2 / 2.75):
            k -= 1.5 / 2.75
            return 7.5625 * k * k + 0.75
        if k < (2.5 / 2.75):
            k -= 2.25 / 2.75
            return 7.5625 * k * k + 0.9375
        k -= 2.625 / 2.75
        return 7.5625 * k * k + 0.984375
    if m == BOUNCE_INOUT:
        if k < 0.5:
            return ramp_calc(k * 2, BOUNCE_IN) * 0.5
        return ramp_calc(k * 2 - 1, BOUNCE_OUT) * 0.5 + 0.5
    return k   # LINEAR / default


class rampFloat:
    """_ramp<float>. `millis` is a zero-argument callable returning the board time in ms."""

    def __init__(self, millis, _val=0.0):
        self._millis = millis
        self.A = self.B = self.val = float(_val)
        self.mode = NONEE
        self.t = millis()
        self.dur = 0
        self.pos = 0
        self.grain = 10
        self.loop = ONCEFORWARD
        self.speed = FORWARD
        self.paused = False
        self.automated = True

    def update(self):
        doUpdate = True
        newTime = 0
        delta = self.grain
        if self.automated:
            newTime = self._millis()
            delta = newTime - self.t
            doUpdate = delta >= self.grain

        if self.mode != NONEE and doUpdate:
            self.t = newTime
            if self.isFinished():
                if self.loop == LOOPFORWARD:
                    self.pos = 0
                elif self.loop == LOOPBACKWARD:
                    self.pos = self.dur
                elif self.loop in (FORTHANDBACK, BACKANDFORTH):
                    self.speed = BACKWARD if self.speed == FORWARD else FORWARD
            if not self.paused:
                if self.speed == FORWARD:
                    self.pos = self.pos + delta if self.pos + delta < self.dur else self.dur
                else:
                    self.pos = self.pos - delta if self.pos - delta > 0 else 0
                if self.mode != NONEE and self.dur > 0 and self.A != self.B:
                    k = self.pos / self.dur
                    r = ramp_calc(k, self.mode)
                    self.val = (self.A + (self.B - self.A) * r) if self.B >= self.A else (self.A - (self.A - self.B) * r)
                else:
                    self.val = self.B
        return self.val

    def go(self, _val, _dur=0, _mode=None, _loop=ONCEFORWARD):
        if _mode is None:
            _mode = NONEE if _dur == 0 else LINEAR   # go(v) -> NONEE ; go(v, dur) -> LINEAR
        _dur = int(_dur)                              # unsigned long parameter
        self.A = self.val
        self.B = float(_val)
        self.mode = _mode
        self.dur = _dur
        self.t = self._millis()
        if _dur == 0:
            self.val = self.B
        if _loop < ONCEBACKWARD:
            self.pos = 0
            self.speed = FORWARD
        else:
            self.pos = self.dur
            self.speed = BACKWARD
        self.loop = _loop
        self.paused = False
        return self.val

    def isFinished(self):
        if self.speed == FORWARD:
            return self.pos == self.dur
        return self.pos == 0

    def isRunning(self):
        return not self.isFinished() and not self.paused

    def getValue(self):
        return self.val

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def setGrain(self, g):
        self.grain = int(g)
