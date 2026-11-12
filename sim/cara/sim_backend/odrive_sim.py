"""Simulated MKS XDrive Mini / ODrive 0.5.x axes (12, vectorised) + their CAN facade.

Per physics step, for every axis in CLOSED_LOOP_CONTROL (ODrive 0.5.x Controller::update):

    INPUT_MODE_POS_FILTER : 2nd-order tracking filter, kp = bw^2, ki = 2 bw  (bw is in rad/s)
    INPUT_MODE_PASSTHROUGH: pos_setpoint = input_pos, vel_setpoint = input_vel
    POSITION control      : vel_des = vel_setpoint + pos_gain * (pos_setpoint - pos_estimate)
    VELOCITY control      : vel_des = vel_setpoint
    vel_des clamped to +-vel_limit
    torque = vel_gain * (vel_des - vel_estimate) + vel_integrator     (controller units, Nm)
    torque clamped to current_lim * controller_torque_constant  (anti-windup: integrator *= 0.99)
    Iq = torque / controller_torque_constant                    -> real motor torque = MOTOR_KT * Iq
    joint torque = incDir * gr * CAPSTAN_EFFICIENCY * motor torque

Positions are motor turns. The encoder is single-turn absolute, so at power-up
Pos_Estimate is somewhere in [0, 1) turn; homing finds the real joint zero.
"""
from __future__ import annotations

import math

import numpy as np

from .. import config as C
from ..firmware.hal import (AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, CONTROL_MODE_POSITION_CONTROL,
                            INPUT_MODE_PASSTHROUGH, INPUT_MODE_POS_FILTER, ODriveCANBase)
from .model_utils import THETA_REF

ERROR_OVERSPEED = 0x1


class ODriveBank:
    def __init__(self, model, data, rng=None):
        rng = rng or np.random.default_rng(0)
        self.rng = rng
        O, S = C.ODRIVE, C.ODRIVE_SIM
        names = [f"{L}_{j}" for L in C.LEG_NAMES for j in C.JOINT_NAMES]
        self.qadr = np.array([model.jnt_qposadr[model.joint(n).id] for n in names])
        self.vadr = np.array([model.jnt_dofadr[model.joint(n).id] for n in names])
        self.act = np.array([model.actuator(f"{n}_motor").id for n in names])
        self.inc = np.array(C.incDir, float)
        self.ref = np.array([THETA_REF[i % 3] for i in range(12)])
        self.k_motor = self.inc * C.gr / (2 * math.pi)            # joint rad/s -> motor turn/s
        self.pos_gain, self.vel_gain, self.vel_int_gain = O["pos_gain"], O["vel_gain"], O["vel_integrator_gain"]
        self.vel_limit = O["vel_limit"]
        self.kt_ctrl = O["controller_torque_constant"]
        self.torque_lim = O["current_lim"] * self.kt_ctrl
        bw = O["input_filter_bandwidth"]
        self.filt_ki = 2.0 * bw
        self.filt_kp = 0.25 * self.filt_ki ** 2
        self.enc_bw, self.cur_bw = S["encoder_bandwidth"], S["current_bandwidth"]
        self.iq_noise = S["iq_noise"]
        self.overspeed = S["enable_overspeed_error"]
        self.vel_tol = S["vel_limit_tolerance"]
        self.joint_gain = self.inc * C.gr * C.CAPSTAN_EFFICIENCY * C.MOTOR_KT   # A -> joint Nm
        n = 12
        self.axis_state = np.full(n, AXIS_STATE_IDLE)
        self.control_mode = np.full(n, CONTROL_MODE_POSITION_CONTROL)   # 0.5.x defaults
        self.input_mode = np.full(n, INPUT_MODE_PASSTHROUGH)
        self.error = np.zeros(n, int)
        self.input_pos = np.zeros(n)
        self.input_vel = np.zeros(n)
        self.pos_setpoint = np.zeros(n)
        self.vel_setpoint = np.zeros(n)
        self.vel_integrator = np.zeros(n)
        self.iq_setpoint = np.zeros(n)
        self.iq = np.zeros(n)
        self.vel_estimate = np.zeros(n)
        self.nan_commands = 0
        self.overspeed_trips = 0
        self.max_abs_vel = 0.0
        # absolute single-turn encoder: Pos_Estimate starts in [0, 1)
        self.enc_zero = self._motor_turns(data) - rng.uniform(0, 1, n)
        self.last_pos = self.pos_estimate(data)

    # --- measurements ---------------------------------------------------------
    def _motor_turns(self, d):
        theta = self.ref + d.qpos[self.qadr] / (2 * math.pi)
        return self.inc * C.gr * theta

    def pos_estimate(self, d):
        return self._motor_turns(d) - self.enc_zero

    # --- CAN commands (called from the firmware thread while physics is paused) ---
    def request_state(self, i, s):
        if s == AXIS_STATE_CLOSED_LOOP_CONTROL:
            if self.axis_state[i] != AXIS_STATE_CLOSED_LOOP_CONTROL and self.error[i] == 0:
                self.axis_state[i] = AXIS_STATE_CLOSED_LOOP_CONTROL
                p = self.last_pos[i]
                self.input_pos[i] = self.pos_setpoint[i] = p          # hold where it is
                self.input_vel[i] = self.vel_setpoint[i] = 0.0
                self.vel_integrator[i] = 0.0
        else:
            self.axis_state[i] = AXIS_STATE_IDLE

    def set_controller_mode(self, i, cm, im):
        self.control_mode[i], self.input_mode[i] = cm, im

    def set_input_pos(self, i, p, vff=0.0):
        if not (math.isfinite(p) and math.isfinite(vff)):
            self.nan_commands += 1                                 # IK failed (C would send NaN)
            return
        self.input_pos[i], self.input_vel[i] = p, vff

    def set_input_vel(self, i, vel):
        self.input_vel[i] = vel

    def clear_errors(self, i):
        self.error[i] = 0

    def iq_measured(self, i):
        return float(self.iq[i] + self.rng.normal(0, self.iq_noise))

    # --- physics-rate update ----------------------------------------------------
    def step(self, d, dt):
        pos = self.pos_estimate(d)
        self.last_pos = pos
        vel_true = self.k_motor * d.qvel[self.vadr]
        self.vel_estimate += (1 - math.exp(-self.enc_bw * dt)) * (vel_true - self.vel_estimate)
        vel = self.vel_estimate

        cl = self.axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL
        posmode = self.control_mode == CONTROL_MODE_POSITION_CONTROL
        pf = cl & posmode & (self.input_mode == INPUT_MODE_POS_FILTER)
        pt = cl & ~pf
        accel = self.filt_kp * (self.input_pos - self.pos_setpoint) + self.filt_ki * (self.input_vel - self.vel_setpoint)
        self.vel_setpoint = np.where(pf, self.vel_setpoint + dt * accel, self.vel_setpoint)
        self.pos_setpoint = np.where(pf, self.pos_setpoint + dt * self.vel_setpoint, self.pos_setpoint)
        self.pos_setpoint = np.where(pt & posmode, self.input_pos, self.pos_setpoint)
        self.vel_setpoint = np.where(pt, self.input_vel, self.vel_setpoint)
        vel_des = self.vel_setpoint + np.where(posmode, self.pos_gain * (self.pos_setpoint - pos), 0.0)
        vel_des = np.clip(vel_des, -self.vel_limit, self.vel_limit)
        v_err = vel_des - vel
        torque = self.vel_gain * v_err + self.vel_integrator
        limited = np.abs(torque) > self.torque_lim
        torque = np.clip(torque, -self.torque_lim, self.torque_lim)
        self.vel_integrator = np.where(limited, self.vel_integrator * 0.99,
                                       self.vel_integrator + self.vel_int_gain * dt * v_err)
        self.vel_integrator[~cl] = 0.0
        self.iq_setpoint = np.where(cl, torque / self.kt_ctrl, 0.0)
        self.iq += (1 - math.exp(-self.cur_bw * dt)) * (self.iq_setpoint - self.iq)
        if cl.any():
            self.max_abs_vel = max(self.max_abs_vel, float(np.abs(vel[cl]).max()))
        if self.overspeed:            # disarms the axis like the real ODrive
            trip = cl & (np.abs(vel) > self.vel_tol * self.vel_limit)
            if trip.any():
                self.error[trip] |= ERROR_OVERSPEED
                self.axis_state[trip] = AXIS_STATE_IDLE
                self.overspeed_trips += int(trip.sum())
        d.ctrl[self.act] = self.joint_gain * self.iq


class SimODriveCAN(ODriveCANBase):
    """One ODriveCAN object (CAN node id = index) backed by the ODriveBank."""

    def __init__(self, hal, index):
        self.hal, self.i = hal, index
        self._feedback_cb = self._status_cb = None

    @property
    def bank(self):
        return self.hal.sim.bank

    def clearErrors(self):
        self.bank.clear_errors(self.i)

    def setState(self, requested_state):
        self.bank.request_state(self.i, requested_state)

    def setControllerMode(self, control_mode, input_mode):
        self.bank.set_controller_mode(self.i, control_mode, input_mode)

    def setPosition(self, position, velocity_feedforward=0.0, torque_feedforward=0.0):
        self.bank.set_input_pos(self.i, position, velocity_feedforward)

    def setVelocity(self, velocity, torque_feedforward=0.0):
        self.bank.set_input_vel(self.i, velocity)

    def getCurrents(self, msg):
        self.hal._advance(C.ODRIVE_SIM["can_request_us"])          # blocking request/response
        msg.Iq_Setpoint = float(self.bank.iq_setpoint[self.i])
        msg.Iq_Measured = self.bank.iq_measured(self.i)
        return True
