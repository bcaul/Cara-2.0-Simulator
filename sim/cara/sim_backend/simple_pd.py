"""PHASE-1 TEMPORARY joint controller.

A joint-space PID whose gains are the ODrive cascade (Build Guide config) reflected
through the 9.7:1 capstan. Phase 2 replaces this with the motor-side ODrive model
(odrive_sim.py) driven by the ported firmware.
"""
from __future__ import annotations

import math

import numpy as np

from .. import config as C

_O = C.ODRIVE
# torque actually produced per unit of controller "torque" (controller assumes 0.04 Nm/A)
_KT_RATIO = C.MOTOR_KT / _O["controller_torque_constant"]
_G = C.gr * C.CAPSTAN_EFFICIENCY * _KT_RATIO * C.gr / (2 * math.pi)

KP = _G * _O["vel_gain"] * _O["pos_gain"]                    # Nm/rad
KD = _G * _O["vel_gain"]                                     # Nm s/rad
KI = _G * _O["vel_integrator_gain"] * _O["pos_gain"]         # Nm/(rad s)
TAU_MAX = _O["current_lim"] * C.MOTOR_KT * C.gr * C.CAPSTAN_EFFICIENCY


class JointPID:
    def __init__(self, model):
        self.model = model
        self.names = [f"{L}_{j}" for L in C.LEG_NAMES for j in C.JOINT_NAMES]
        self.qadr = np.array([model.jnt_qposadr[model.joint(n).id] for n in self.names])
        self.vadr = np.array([model.jnt_dofadr[model.joint(n).id] for n in self.names])
        self.act = np.array([model.actuator(f"{n}_motor").id for n in self.names])
        self.target = np.zeros(12)
        self.integ = np.zeros(12)

    def __call__(self, data, dt):
        err = self.target - data.qpos[self.qadr]
        self.integ = np.clip(self.integ + KI * err * dt, -TAU_MAX, TAU_MAX)
        tau = KP * err - KD * data.qvel[self.vadr] + self.integ
        data.ctrl[self.act] = np.clip(tau, -TAU_MAX, TAU_MAX)
