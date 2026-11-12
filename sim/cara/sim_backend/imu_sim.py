"""Simulated BNO086 rotation-vector output (SparkFun BNO08x API subset).

Orientation of the MuJoCo `imu` site relative to GRAVITY (like the real sensor's
gravity-referenced rotation vector), rotated by the mounting (config.IMU_MOUNT_YAW_DEG),
with white noise + a random-walk bias, reported every IMU_REPORT_MS.
getRoll/getPitch use the same quaternion -> Euler formulas as the SparkFun library.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from .. import config as C
from ..firmware.hal import IMUBase, SENSOR_REPORTID_ROTATION_VECTOR


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _qexp(rv):
    a = float(np.linalg.norm(rv))
    if a < 1e-12:
        return np.array([1.0, 0, 0, 0])
    return np.concatenate([[math.cos(a / 2)], math.sin(a / 2) * rv / a])


class SimIMU(IMUBase):
    def __init__(self, model, data, rng=None):
        self.m, self.d = model, data
        self.rng = rng or np.random.default_rng(1)
        self.site = model.site("imu").id
        yaw = math.radians(C.IMU_MOUNT_YAW_DEG)
        self.q_mount = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
        self.sigma = math.radians(C.IMU_ANGLE_NOISE_DEG)
        self.walk = math.radians(C.IMU_BIAS_WALK_DEG)
        self.bias = np.zeros(3)
        self.period_us = int(C.IMU_REPORT_MS * 1000)
        self.enabled = False
        self.q = np.array([1.0, 0, 0, 0])
        self._new = False

    # physics side ---------------------------------------------------------------
    def _q_true(self):
        """Mounted-sensor orientation in a frame whose -z is the gravity direction."""
        q_site = np.zeros(4)
        mujoco.mju_mat2Quat(q_site, self.d.site_xmat[self.site])
        g = -np.asarray(self.m.opt.gravity, float)
        g /= np.linalg.norm(g)                       # "up" as the sensor feels it
        axis = np.cross(g, [0.0, 0.0, 1.0])
        s, c = np.linalg.norm(axis), float(np.dot(g, [0.0, 0.0, 1.0]))
        q_g = np.array([1.0, 0, 0, 0]) if s < 1e-12 else _qexp(axis / s * np.arctan2(s, c))
        return _qmul(_qmul(q_g, q_site), self.q_mount)

    def update(self, t_us):
        if not self.enabled or t_us % self.period_us:
            return
        q = self._q_true()
        self.bias += self.walk * math.sqrt(self.period_us * 1e-6) * self.rng.normal(size=3)
        q = _qmul(q, _qexp(self.bias + self.sigma * self.rng.normal(size=3)))
        self.q = q / np.linalg.norm(q)
        self._new = True

    def true_roll_pitch(self):
        """Noise-free (roll, pitch) in the same convention, for displays/tests."""
        return self._euler(self._q_true())

    @staticmethod
    def _euler(q):
        w, x, y, z = q
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        return roll, math.asin(t2)

    # firmware side (SparkFun API) -------------------------------------------------
    def enableRotationVector(self, timeBetweenReports_ms=10):
        self.enabled = True
        return True

    def wasReset(self):
        return False

    def getSensorEvent(self):
        if self._new:
            self._new = False
            return True
        return False

    def getSensorEventID(self):
        return SENSOR_REPORTID_ROTATION_VECTOR

    def getRoll(self):
        return self._euler(self.q)[0]

    def getPitch(self):
        return self._euler(self.q)[1]
