"""SimHAL + Simulation: run the ported firmware against MuJoCo in lockstep.

The firmware runs on its own thread with a VIRTUAL clock (fw_time_us). Whenever it
needs time to pass (delay, pumpEvents busy-waits, CAN request round-trips) and its
clock gets ahead of the physics clock, it blocks and the physics thread steps until
it catches up. So `while (ramp.isRunning()) { ...; delay(5); }`, the 10 ms Ramp grain
and millis()-based logic all behave as on the Teensy, deterministically, however
fast or slow the PC is.

Physics step (2 kHz) order:  RC frame ISR / encoder CAN frames / IMU report
                             -> firmware runs if its wake time has come
                             -> ODrive controllers -> hand (optional) -> mj_step
"""
from __future__ import annotations

import math
import threading
import traceback

import mujoco
import numpy as np

from .. import config as C
from ..firmware.cara_firmware import CaraFirmware
from ..firmware.hal import HAL, FirmwareStopped, Get_Encoder_Estimates_msg_t
from ..firmware.kinematics import getIK
from ..model import load_model
from .imu_sim import SimIMU
from .model_utils import set_leg_pose
from .odrive_sim import ODriveBank, SimODriveCAN


class SimHAL(HAL):
    def __init__(self, sim):
        self.sim = sim
        self.fw_time_us = 0
        self.odrive = [SimODriveCAN(self, i) for i in range(12)]
        self.imu = sim.imu
        self._cv = threading.Condition()
        self._fw_turn = False
        self._wake_us = None
        self._stopped = False
        self._finished = False
        self.error = None
        self._isr = None
        self._feedback = {}          # axis -> latest Get_Encoder_Estimates_msg_t (CAN RX)
        self._serial_in = []
        self.log = []                # serial output

    # --- Arduino services (firmware thread) ------------------------------------
    def millis(self):
        return self.fw_time_us // 1000

    def micros(self):
        return self.fw_time_us

    def delay(self, ms):
        self._advance(ms * 1000)

    def pumpEvents(self):
        fb, self._feedback = self._feedback, {}
        for i, msg in fb.items():
            cb = self.odrive[i]._feedback_cb
            if cb:
                cb[0](msg, cb[1])
        self._advance(C.PUMP_EVENTS_US)

    def serial_println(self, text):
        self.log.append((self.fw_time_us, text))
        print(f"[fw {self.fw_time_us / 1e6:7.3f}s] {text}")

    def serial_read(self):
        return self._serial_in.pop(0) if self._serial_in else None

    def attach_rc_interrupt(self, isr):
        self._isr = isr

    # --- lockstep ----------------------------------------------------------------
    def _advance(self, us):
        self.fw_time_us += int(round(us))
        if self.fw_time_us > self.sim.time_us:
            self._yield()

    def _yield(self):
        with self._cv:
            self._wake_us = self.fw_time_us
            self._fw_turn = False
            self._cv.notify_all()
            self._cv.wait_for(lambda: self._fw_turn or self._stopped)
        if self._stopped:
            raise FirmwareStopped()

    def start(self, program, fw):
        def runner():
            with self._cv:
                self._cv.wait_for(lambda: self._fw_turn or self._stopped)
            try:
                if not self._stopped:
                    program(fw)
            except FirmwareStopped:
                pass
            except Exception as ex:          # noqa: BLE001 - reported on the physics side
                self.error = ex
                traceback.print_exc()
            finally:
                with self._cv:
                    self._finished = True
                    self._fw_turn = False
                    self._cv.notify_all()

        self._wake_us = 0
        self.thread = threading.Thread(target=runner, name="cara-firmware", daemon=True)
        self.thread.start()

    def service(self, t_us):
        """Physics side: if the firmware is due, let it run until it blocks again."""
        if self._finished or self._wake_us is None or t_us < self._wake_us:
            return
        with self._cv:
            self._wake_us = None
            self._fw_turn = True
            self._cv.notify_all()
            self._cv.wait_for(lambda: not self._fw_turn)

    def stop(self):
        with self._cv:
            self._stopped = True
            self._cv.notify_all()

    @property
    def running(self):
        return not self._finished


class Hand:
    """Kinematically holds the robot by its handle (like a person holding it during homing)."""

    def __init__(self, sim):
        self.sim = sim
        self.pos = None
        self.quat = np.array([1.0, 0, 0, 0])
        self.lowering = False
        self.speed = 0.15
        self._lift_to, self._lift_speed = None, 0.4
        self.foot_geoms = {sim.m.geom(f"{L}_foot").id for L in C.LEG_NAMES}

    @property
    def active(self):
        return self.pos is not None

    def hold(self, height):
        d = self.sim.d
        self.pos = np.array([d.qpos[0], d.qpos[1], height])
        self.quat = np.array([1.0, 0, 0, 0])
        self.lowering = False

    def lower_to_floor(self, speed=0.15):
        self.speed, self.lowering = speed, True

    def pick_up(self, height=0.45, speed=0.4):
        """Grab the handle where the robot is, level it (keeping its heading) and lift it."""
        d = self.sim.d
        R = d.body("base").xmat.reshape(3, 3)
        yaw = math.atan2(R[1, 0], R[0, 0])
        self.pos = d.qpos[0:3].copy()
        self.quat = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
        self.lowering = False
        self._lift_to, self._lift_speed = height, speed

    def release(self):
        self.pos, self.lowering = None, False

    def _feet_touching(self):
        d = self.sim.d
        return any(d.contact[k].geom1 in self.foot_geoms or d.contact[k].geom2 in self.foot_geoms
                   for k in range(d.ncon))

    def apply(self, dt):
        if self.pos is None:
            return
        d = self.sim.d
        if self.lowering:
            self.pos[2] -= self.speed * dt
            if self._feet_touching() or self.pos[2] < 0.05:
                self.release()
                return
        elif self._lift_to is not None:
            self.pos[2] = min(self._lift_to, self.pos[2] + self._lift_speed * dt)
        d.qpos[0:3] = self.pos
        d.qpos[3:7] = self.quat
        d.qvel[0:6] = 0.0


class Simulation:
    """MuJoCo world + 12 ODrives + IMU + RC receiver + firmware thread.

    held_height: start held in the air by the handle at this base height (m), or None to
                 start resting on the floor.
    rc_source:   callable returning 8 pulse widths (us), polled every RC frame; None = no receiver.
    """

    def __init__(self, held_height=0.5, stabilize=False, seed=0, rc_source=None, model=None):
        self.m = model or load_model()
        self.d = mujoco.MjData(self.m)
        self.dt_us = int(round(self.m.opt.timestep * 1e6))
        self.step_count = 0
        rng = np.random.default_rng(seed)
        self._power_on_pose(held_height)
        self.bank = ODriveBank(self.m, self.d, rng)
        self.imu = SimIMU(self.m, self.d, np.random.default_rng(seed + 1))
        self.hal = SimHAL(self)
        self.fw = CaraFirmware(self.hal, stabilize=stabilize)
        self.hand = Hand(self)
        if held_height is not None:
            self.hand.hold(held_height)
        self.rc_source = rc_source
        self._enc_us = int(C.ODRIVE_SIM["encoder_msg_rate_ms"] * 1000)
        self._rc_us = int(C.RC_FRAME_MS * 1000)

    def _power_on_pose(self, held_height):
        """Robot as it is stored: legs folded at the zStart pose, motors unpowered."""
        m, d = self.m, self.d
        mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
        th = getIK(C.xStart, C.yStart, C.zStart)
        for leg in range(4):
            set_leg_pose(m, d, leg, *th)
        d.qpos[2] = held_height if held_height is not None else (C.zStart / 1000 + C.FOOT_RADIUS + 0.005)
        mujoco.mj_forward(m, d)

    @property
    def time_us(self):
        return self.step_count * self.dt_us

    @property
    def time(self):
        return self.step_count * self.dt_us * 1e-6

    def start(self, program=None):
        """Start the firmware thread. Default program = setup() then loop() forever."""
        if program is None:
            def program(fw):
                fw.setup()
                while True:
                    fw.loop()
        self.hal.start(program, self.fw)

    def step(self):
        t = self.time_us
        if t % self._enc_us == 0:                     # cyclic encoder CAN frames
            pos = self.bank.last_pos
            for i in range(12):
                self.hal._feedback[i] = Get_Encoder_Estimates_msg_t(float(pos[i]), float(self.bank.vel_estimate[i]))
        if self.rc_source is not None and self.hal._isr is not None and t % self._rc_us == 0:
            widths = self.rc_source()
            if widths is not None:
                self.hal._isr(widths)                  # firmware is blocked: safe "interrupt"
        self.imu.update(t)
        self.hal.service(t)
        self.bank.step(self.d, self.m.opt.timestep)
        self.hand.apply(self.m.opt.timestep)
        mujoco.mj_step(self.m, self.d)
        self.step_count += 1

    def run_for(self, seconds, every=None, callback=None):
        n = int(round(seconds * 1e6 / self.dt_us))
        for k in range(n):
            self.step()
            if callback and every and k % every == 0:
                callback(self)
            if self.hal.error:
                raise RuntimeError("firmware thread crashed") from self.hal.error

    def stop(self):
        self.hal.stop()

    # convenience readouts ---------------------------------------------------------
    def base_height_mm(self):
        return float(self.d.qpos[2] * 1000)

    def roll_pitch_deg(self):
        r, p = self.imu.true_roll_pitch()
        return -math.degrees(r), -math.degrees(p)     # firmware sign convention (rawRoll/rawPitch)
