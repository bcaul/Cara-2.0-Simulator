"""Python port of the CARA 2.0 Teensy firmware (CARA_2.0_Code.ino + Functions.ino).

Function names, global variable names, control flow, blocking loops and delays are
kept identical so the two can be read side by side. Differences are marked PORT NOTE.
All hardware access goes through a HAL (cara/firmware/hal.py), so this file runs
unchanged against the simulator (SimHAL) or, later, the real robot (RealHAL).

Run it on its own thread:   fw = CaraFirmware(hal); fw.setup(); while True: fw.loop()
"""
from __future__ import annotations

from math import acos, asin, atan, cos, pi as PI, sin, sqrt, tan

from .. import config as _C
from . import kinematics as _K
from .hal import (AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, CONTROL_MODE_POSITION_CONTROL,
                  CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH, INPUT_MODE_POS_FILTER,
                  SENSOR_REPORTID_ROTATION_VECTOR, Get_Encoder_Estimates_msg_t, Get_Iq_msg_t,
                  Heartbeat_msg_t)
from .lowpass import LowPass
from .ramp import (CUBIC_OUT, EXPONENTIAL_OUT, LINEAR, QUADRATIC_IN, QUADRATIC_OUT, rampFloat)

# Constants (CARA_2.0_Code.ino)
OFF = _C.OFF
gr, homingIq = _C.gr, _C.homingIq
incDir, homeDir, homeAbsPos = _C.incDir, _C.homeDir, _C.homeAbsPos
c, d, e, h, m, phi, w, v = _C.c, _C.d, _C.e, _C.h, _C.m, _C.phi, _C.w, _C.v
xStart, yStart, zStart = _C.xStart, _C.yStart, _C.zStart
chnPin, mapRange, constrainRange, sensitivity = _C.chnPin, _C.mapRange, _C.constrainRange, _C.sensitivity
NAN = float("nan")


def sq(x):
    return x * x


def constrain(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _map(x, in_min, in_max, out_min, out_max):
    """Teensyduino map() for a floating-point x (width[] is float): no integer rounding."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def _to_int(x):
    """C float -> int conversion (truncation toward zero)."""
    return int(x)


class ODriveUserData:
    def __init__(self):
        self.last_heartbeat = Heartbeat_msg_t()
        self.received_heartbeat = False
        self.last_feedback = Get_Encoder_Estimates_msg_t()
        self.received_feedback = False


class CaraFirmware:
    def __init__(self, hal, stabilize=False):
        self.hal = hal
        self.odrive = hal.odrive
        self.myIMU = hal.imu
        self.stabilize = stabilize      # True = enable the getShifts() block commented out in loop()
        ms = hal.millis
        # Objects
        self.rampCoord = [rampFloat(ms) for _ in range(12)]
        self.rampTilt = [rampFloat(ms) for _ in range(3)]
        self.rampT = rampFloat(ms)
        self.userData = [ODriveUserData() for _ in range(12)]
        self.lpRoll = LowPass(1, 0.1, 1e3, True, hal.micros)
        self.lpPitch = LowPass(1, 0.1, 1e3, True, hal.micros)
        # ODrive variables
        self.theta = [0.0] * 12
        self.thetaOffset = [0.0] * 12
        self.iq = [0.0] * 12
        # Kinematics variables
        self.coordFK = [0.0] * 12
        self.thetaIK = [0.0] * 12
        self.coordsIKR = [0.0] * 12
        # RC variables
        self.chnVal = [0] * 8
        self.chn5Prev = self.chn6Prev = 0
        self.start = [0.0] * 8
        self.width = [0.0] * 8
        self.btnTime = 0
        self.loopTimeI = 0
        # Other variables
        self.state = 0                  # 0 = non calibrated, 1 = starting, 2 = idle, 3 = standing
        self.zDefault, self.zLow = float(_C.zDefault), float(_C.zLow)
        self.xNom, self.yNom, self.zNom = 0.0, e, self.zDefault
        self.xHome = [self.xNom] * 4
        self.yHome = [self.yNom] * 4
        self.zHome = [self.zNom] * 4
        self.rawRoll = self.rawPitch = self.filtRoll = self.filtPitch = 0.0
        self._disconnected_printed = False

    # Arduino helpers -----------------------------------------------------------
    def millis(self):
        return self.hal.millis()

    def micros(self):
        return self.hal.micros()

    def delay(self, ms):
        self.hal.delay(ms)

    def pumpEvents(self):
        self.hal.pumpEvents()

    # setup / loop (CARA_2.0_Code.ino) -----------------------------------------
    def setup(self):
        self.odriveSetup()
        self.myIMU.begin(0x4B)
        self.myIMU.enableRotationVector()
        self.hal.attach_rc_interrupt(self.isrHandler)

    def loop(self):
        # Stabilize the robot when it's not walking (commented out in the real firmware)
        if self.stabilize and self.state == 3:
            self.getIMU()
            self.getShifts()
            self.moveFeetInst(self.xHome[0], self.yHome[0], self.zHome[0], self.xHome[1], self.yHome[1], self.zHome[1],
                              self.xHome[2], self.yHome[2], self.zHome[2], self.xHome[3], self.yHome[3], self.zHome[3])
        self.getReceiver()
        self.delay(5)

    # --------------------***CAN Functions***--------------------
    @staticmethod
    def onHeartbeat(msg, user_data):
        user_data.last_heartbeat = msg
        user_data.received_heartbeat = True

    @staticmethod
    def onFeedback(msg, user_data):
        user_data.last_feedback = msg
        user_data.received_feedback = True

    def odriveSetup(self):
        for i in range(12):
            self.odrive[i].onFeedback(self.onFeedback, self.userData[i])
            self.odrive[i].onStatus(self.onHeartbeat, self.userData[i])

    # --------------------***ODrive Functions***--------------------
    def setState(self, num, state):
        timeI = self.millis()
        while self.millis() - timeI < 5:
            self.odrive[num].clearErrors()
            self.odrive[num].setState(AXIS_STATE_CLOSED_LOOP_CONTROL if state else AXIS_STATE_IDLE)
            self.pumpEvents()
            self.delay(1)

    def getJointPos(self):
        timeI = self.millis()
        while self.millis() - timeI < 5:
            self.pumpEvents()
            for i in range(12):
                if self.userData[i].received_feedback:
                    feedback = self.userData[i].last_feedback
                    self.userData[i].received_feedback = False
                    self.theta[i] = feedback.Pos_Estimate * incDir[i] / gr - self.thetaOffset[i]

    def getJointIq(self):
        for i in range(12):
            msg = Get_Iq_msg_t()
            self.odrive[i].getCurrents(msg)
            self.iq[i] = msg.Iq_Measured

    # --------------------***Kinematics Functions***--------------------
    def getIK(self, leg, x, y, z):
        try:
            tA, tB, tAB = _K.getIK(x, y, z)
        except (ValueError, ZeroDivisionError):
            # PORT NOTE: C's acos() returns NaN outside [-1, 1]; Python raises. Keep NaN.
            tA = tB = tAB = NAN
        self.thetaIK[leg * 3], self.thetaIK[leg * 3 + 1], self.thetaIK[leg * 3 + 2] = tA, tB, tAB

    def getFK(self):
        self.getJointPos()
        for i in range(4):
            try:
                xyz = _K.getFK(self.theta[i * 3], self.theta[i * 3 + 1], self.theta[i * 3 + 2])
            except (ValueError, ZeroDivisionError):
                xyz = (NAN, NAN, NAN)
            self.coordFK[i * 3:i * 3 + 3] = xyz

    def getIKR(self, roll, pitch, yaw):
        xNom, yNom, zNom = self.xNom, self.yNom, self.zNom
        roll *= PI / 180
        pitch *= PI / 180
        yaw *= PI / 180
        # Compute roll axis IK
        b = zNom - w / 2 * sin(roll)
        c_ = yNom + w / 2 * (1 - cos(roll))
        zR = sqrt(b * b + c_ * c_) * sin(PI / 2 - roll - atan(c_ / b))
        yR = sqrt(b * b + c_ * c_) * cos(PI / 2 - roll - atan(c_ / b))
        b = zNom + w / 2 * sin(roll)
        zL = sqrt(b * b + c_ * c_) * sin(PI / 2 + roll - atan(c_ / b))
        yL = sqrt(b * b + c_ * c_) * cos(PI / 2 + roll - atan(c_ / b))
        # Compute pitch axis IK using the Z values (ZL and ZR) from the roll axis
        z = [zR, zL]
        zOutF, zOutB, xF, xB = [0.0] * 2, [0.0] * 2, [0.0] * 2, [0.0] * 2
        for i in range(2):
            b = z[i] - v / 2 * sin(pitch)
            c_ = xNom + v / 2 * (1 - cos(pitch))
            zOutF[i] = sqrt(b * b + c_ * c_) * sin(PI / 2 - pitch - atan(c_ / b))
            xF[i] = sqrt(b * b + c_ * c_) * cos(PI / 2 - pitch - atan(c_ / b))
            b = z[i] + v / 2 * sin(pitch)
            zOutB[i] = sqrt(b * b + c_ * c_) * sin(PI / 2 + pitch - atan(c_ / b))
            xB[i] = sqrt(b * b + c_ * c_) * cos(PI / 2 + pitch - atan(c_ / b))
        # Compute yaw axis IK
        y = [yR, yL]
        x0 = [xF[0], xB[1]]
        x1 = [xB[0], xF[1]]
        xOut0, yOut0, xOut1, yOut1 = [0.0] * 2, [0.0] * 2, [0.0] * 2, [0.0] * 2
        for i in range(2):
            e_ = w / 2 + y[i] - 0.5 * sqrt(sq(v) + sq(w)) * cos(atan(v / w) + yaw)
            f = v / 2 + x0[i] - 0.5 * sqrt(sq(v) + sq(w)) * sin(atan(v / w) + yaw)
            g = f * tan(yaw)
            yOut0[i] = (e_ + g) * cos(yaw)
            xOut0[i] = f / cos(yaw) - (e_ + g) * sin(yaw)
            e_ = w / 2 + y[i] - 0.5 * sqrt(sq(v) + sq(w)) * cos(atan(v / w) - yaw)
            f = v / 2 + x1[i] - 0.5 * sqrt(sq(v) + sq(w)) * sin(atan(v / w) - yaw)
            g = -f * tan(yaw)
            yOut1[i] = (e_ + g) * cos(yaw)
            xOut1[i] = f / cos(yaw) + (e_ + g) * sin(yaw)
        coords = [xOut1[1], yOut1[1], zOutF[1], xOut0[0], yOut0[0], zOutF[0],
                  xOut0[1], yOut0[1], zOutB[1], xOut1[0], yOut1[0], zOutB[0]]
        for i in range(12):
            self.coordsIKR[i] = coords[i]

    # --------------------***Movement Functions***--------------------
    def setJointPos(self):
        self.getIMU()
        for i in range(12):
            if self.thetaIK[i] != OFF:
                self.odrive[i].setPosition((self.thetaIK[i] + self.thetaOffset[i]) * incDir[i] * gr)

    def home(self):
        check = [0] * 12
        sum_ = 0
        for i in range(12):
            self.setState(i, 1)
            self.thetaOffset[i] = 0
            self.odrive[i].setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH)
            self.odrive[i].setVelocity(1.5 * homeDir[i])
        while sum_ != 12:
            self.getJointIq()
            for i in range(12):
                if not check[i] and abs(self.iq[i]) > homingIq:
                    check[i] = 1
                    self.getJointPos()
                    self.thetaOffset[i] = self.theta[i] - homeAbsPos[i % 3] / 360
                    self.odrive[i].setVelocity(0)
                    self.setState(i, 0)
                    self.setState(i, 1)
                    self.odrive[i].setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_POS_FILTER)
                    sum_ += 1
        self.moveFeet(xStart, yStart, zStart, xStart, yStart, zStart, xStart, yStart, zStart, xStart, yStart, zStart,
                      LINEAR, 500)

    def moveFeet(self, x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, mode, time):
        coord = [x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3]
        self.getFK()
        for i in range(12):
            if coord[i] == OFF:
                self.rampCoord[i].go(OFF)
            else:
                self.rampCoord[i].go(self.coordFK[i])
                self.rampCoord[i].go(coord[i], time, mode)
        self.rampT.go(0)
        self.rampT.go(1, time, LINEAR)
        while self.rampT.isRunning():
            for i in range(4):
                self.getIK(i, self.rampCoord[i * 3].update(), self.rampCoord[i * 3 + 1].update(),
                           self.rampCoord[i * 3 + 2].update())
            self.rampT.update()
            self.setJointPos()
            self.delay(5)

    def moveFeetInst(self, x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3):
        coord = [x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3]
        for i in range(4):
            self.getIK(i, coord[i * 3], coord[i * 3 + 1], coord[i * 3 + 2])
        self.setJointPos()

    def moveRobot(self, x, y, z, mode, time):
        xNom, yNom, zNom = self.xNom, self.yNom, self.zNom
        self.moveFeet(xNom - x, yNom + y, zNom + z, xNom - x, yNom - y, zNom + z,
                      xNom + x, yNom + y, zNom + z, xNom + x, yNom - y, zNom + z, mode, time)

    def tiltRobot(self, roll, pitch, yaw, mode, time):
        self.rampTilt[0].go(roll, time, mode)
        self.rampTilt[1].go(pitch, time, mode)
        self.rampTilt[2].go(yaw, time, mode)
        self.rampT.go(0)
        self.rampT.go(1, time, LINEAR)
        while self.rampT.isRunning():
            self.getIKR(self.rampTilt[0].update(), self.rampTilt[1].update(), self.rampTilt[2].update())
            for i in range(4):
                self.getIK(i, self.coordsIKR[i * 3], self.coordsIKR[i * 3 + 1], self.coordsIKR[i * 3 + 2])
            self.rampT.update()
            self.setJointPos()
            self.delay(5)

    def stepPhase(self, leg, phase, h, l, alpha, t):
        xHome, yHome, zHome = self.xHome, self.yHome, self.zHome
        x = y = z = 0.0     # PORT NOTE: uninitialised in C; every caller passes phase 0..2
        if phase == 0:
            aT = l * (t - sin(2 * PI * t) / 2 / PI)
            x = xHome[leg] - l / 2 * cos(alpha[leg]) + aT * cos(alpha[leg])
            y = yHome[leg] - l / 2 * sin(alpha[leg]) + aT * sin(alpha[leg])
            z = zHome[leg] - h / 2 * (1 - cos(2 * PI * t))
        if phase == 1:
            x = xHome[leg] + l * cos(alpha[leg]) * (0.5 - 2 * t)
            y = yHome[leg] + l * sin(alpha[leg]) * (0.5 - 2 * t)
            z = zHome[leg]
        if phase == 2:
            x = xHome[leg] + l * cos(alpha[leg]) * (0.5 - t)
            y = yHome[leg] + l * sin(alpha[leg]) * (0.5 - t)
            z = zHome[leg]
        self.getIK(leg, x, y, z)

    def _phase_loop(self, start, end, time, legs_phases, h, l, alpha):
        """rampT.go(start); rampT.go(end, time, LINEAR); while (rampT.isRunning()) {...}"""
        rampT = self.rampT
        rampT.go(start)
        rampT.go(end, time, LINEAR)
        while rampT.isRunning():
            for leg, phase in legs_phases:
                self.stepPhase(leg, phase, h, l, alpha, rampT.update())
            self.setJointPos()
            self.delay(5)

    def step(self, h, l, alpha0, alpha1, alpha2, alpha3, time, num):
        if self.criteria(num):
            xHome, yHome, zHome = self.xHome, self.yHome, self.zHome
            alpha = [alpha0 * PI / 180, alpha1 * PI / 180, alpha2 * PI / 180, alpha3 * PI / 180]
            # **Pre loop sequences**
            # FL and BR = up, FR and BL = back
            self.moveFeet(xHome[0], yHome[0], zHome[0] - h,
                          xHome[1] - h / 2 * cos(alpha[1]), yHome[1] - h / 2 * sin(alpha[1]), zHome[1],
                          xHome[2] - h / 2 * cos(alpha[2]), yHome[2] - h / 2 * sin(alpha[2]), zHome[2],
                          xHome[3], yHome[3], zHome[3] - h, LINEAR, time)
            # FL and BR = swing 2, FR and BL = nothing
            self._phase_loop(0.5, 1, time, [(0, 0), (3, 0)], h, l, alpha)
            # Take steps
            while True:
                if self.stabilize:
                    self.getShifts()    # commented out in the firmware
                # FL and BR = stance, FR and BL = swing 1
                self._phase_loop(0, 0.5, time, [(0, 1), (1, 0), (2, 0), (3, 1)], h, l, alpha)
                # FL and BR = nothing, FR and BL = swing 2
                self._phase_loop(0.5, 1, time, [(1, 0), (2, 0)], h, l, alpha)
                # **Continuous loop sequences**
                if self.criteria(num):
                    # FL and BR = swing 1, FR and BL = stance
                    self._phase_loop(0, 0.5, time, [(0, 0), (1, 1), (2, 1), (3, 0)], h, l, alpha)
                    # FL and BR = swing 2, FR and BL = nothing
                    self._phase_loop(0.5, 1, time, [(0, 0), (3, 0)], h, l, alpha)
                # **Post loop sequences**
                else:
                    # FL and BR = swing 1, FR and BL = stance'
                    self._phase_loop(0, 0.5, time, [(0, 0), (1, 2), (2, 2), (3, 0)], h, l, alpha)
                    # FL and BR = move down, FR and BL = nothing
                    self.moveFeet(xHome[0], yHome[0], zHome[0], OFF, OFF, OFF, OFF, OFF, OFF,
                                  xHome[3], yHome[3], zHome[3], LINEAR, time)
                    break

    def stepInPlace(self, h, time):
        if self.stabilize:
            self.getShifts()            # commented out in the firmware
        X, Y, Z = self.xHome, self.yHome, self.zHome
        # FL and BR = move up
        self.moveFeet(X[0], Y[0], Z[0] - h, X[1], Y[1], Z[1], X[2], Y[2], Z[2], X[3], Y[3], Z[3] - h, QUADRATIC_OUT, time)
        # FL and BR = move down
        self.moveFeet(X[0], Y[0], Z[0], X[1], Y[1], Z[1], X[2], Y[2], Z[2], X[3], Y[3], Z[3], QUADRATIC_OUT, time)
        # FR and BL = move up
        self.moveFeet(X[0], Y[0], Z[0], X[1], Y[1], Z[1] - h, X[2], Y[2], Z[2] - h, X[3], Y[3], Z[3], QUADRATIC_OUT, time)
        # FL and BR = move down
        self.moveFeet(X[0], Y[0], Z[0], X[1], Y[1], Z[1], X[2], Y[2], Z[2], X[3], Y[3], Z[3], QUADRATIC_OUT, time)

    def kinematicsDemo(self):
        mode = CUBIC_OUT
        time = 500
        mR, tR = self.moveRobot, self.tiltRobot
        # **Translation kinematics**
        mR(0, 0, 40, mode, time); mR(0, 0, -100, mode, time * 3); mR(0, 0, 0, mode, time * 2)
        mR(50, 0, 0, mode, time); mR(-50, 0, 0, mode, time * 2); mR(0, 0, 0, mode, time)
        mR(0, 25, 0, mode, time); mR(0, -25, 0, mode, time * 2); mR(0, 0, 0, mode, time)
        self.delay(100)
        # **Rotation Kinematics**
        time = 500
        tR(15, 0, 0, mode, time); tR(-15, 0, 0, mode, time * 2); tR(0, 0, 0, mode, time)
        tR(0, 15, 0, mode, time); tR(0, -15, 0, mode, time * 2); tR(0, 0, 0, mode, time)
        tR(0, 0, 15, mode, time); tR(0, 0, -15, mode, time * 2); tR(0, 0, 0, mode, time)
        self.delay(100)
        time = 500
        # Tilt in pitch and roll
        tR(0, 10, 0, mode, time); tR(-10, 10, 0, mode, time); tR(10, 10, 0, mode, time * 2)
        tR(0, 0, 0, mode, time * 2); tR(0, -10, 0, mode, time); tR(-10, -10, 0, mode, time)
        tR(10, -10, 0, mode, time * 2)
        # Tilt in yaw, pitch, and roll
        tR(0, 0, 15, mode, time * 2); tR(0, 10, 15, mode, time); tR(0, -10, 15, mode, time * 2)
        tR(0, 0, 15, mode, time); tR(10, 0, 15, mode, time); tR(-10, 0, 15, mode, time * 2)
        tR(0, 0, 0, mode, time * 1.5); tR(0, 0, -15, mode, time * 2); tR(0, 10, -15, mode, time)
        tR(0, -10, -15, mode, time * 2); tR(0, 0, -15, mode, time); tR(10, 0, -15, mode, time)
        tR(-10, 0, -15, mode, time * 2); tR(0, 0, 0, mode, time * 1.5)
        self.delay(100)
        # Circulate in roll and pitch
        time = 500
        tR(0, 15, 0, mode, time); tR(-15, 0, 0, QUADRATIC_IN, time); tR(0, -15, 0, QUADRATIC_OUT, time)
        tR(15, 0, 0, QUADRATIC_IN, time); tR(0, 15, 0, QUADRATIC_OUT, time); tR(0, 0, 0, mode, time)
        tR(0, -15, 0, mode, time); tR(-15, 0, 0, QUADRATIC_IN, time); tR(0, 15, 0, QUADRATIC_OUT, time)
        tR(15, 0, 0, QUADRATIC_IN, time); tR(0, -15, 0, QUADRATIC_OUT, time); tR(0, 0, 0, mode, time)
        self.delay(100)
        self.jump(3)

    def jump(self, num):
        for i in range(num):
            self.moveRobot(0, 0, -120, LINEAR, 300)
            self.moveRobot(0, 0, 40, EXPONENTIAL_OUT, 100)
            self.moveRobot(0, 0, 0, EXPONENTIAL_OUT, 100 / 4)
            self.delay(500)

    # --------------------***Control Functions***--------------------
    def getIMU(self):
        if self.myIMU.wasReset():
            self.myIMU.enableRotationVector()
        if self.myIMU.getSensorEvent() and self.myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR:
            self.rawRoll = constrain(-self.myIMU.getRoll(), -20 * PI / 180, 20 * PI / 180)
            self.rawPitch = constrain(-self.myIMU.getPitch(), -20 * PI / 180, 20 * PI / 180)
            self.filtRoll = self.lpRoll.filt(self.rawRoll)
            self.filtPitch = self.lpPitch.filt(self.rawPitch)

    def getShifts(self):
        xShift = self.zNom * tan(self.filtPitch)
        yShift = self.zNom * tan(self.filtRoll)
        xNom, yNom = self.xNom, self.yNom
        self.xHome[0] = xNom + xShift
        self.xHome[1] = xNom + xShift
        self.xHome[2] = xNom - xShift
        self.xHome[3] = xNom - xShift
        self.yHome[0] = yNom - yShift
        self.yHome[1] = yNom + yShift
        self.yHome[2] = yNom - yShift
        self.yHome[3] = yNom + yShift

    def criteria(self, num):
        chnVal = self.chnVal
        if num == 0: return abs(chnVal[0]) < 6 and chnVal[1] > 7      # forward
        if num == 1: return abs(chnVal[0]) < 6 and chnVal[1] < -7     # backward
        if num == 2: return chnVal[0] > 7 and abs(chnVal[1]) < 6      # right
        if num == 3: return chnVal[0] < -7 and abs(chnVal[1]) < 6     # left
        if num == 4: return chnVal[0] > 7 and chnVal[1] > 7           # forward right
        if num == 5: return chnVal[0] > 7 and chnVal[1] < -7          # back right
        if num == 6: return chnVal[0] < -7 and chnVal[1] > 7          # forward left
        if num == 7: return chnVal[0] < -7 and chnVal[1] < -7         # back left
        if num == 8: return chnVal[3] > 7                             # turn CW
        if num == 9: return chnVal[3] < -7                            # turn CCW
        return False

    # --------------------***User Input Functions***--------------------
    def getReceiver(self):
        chnVal = self.chnVal
        if self.width[2] > 1000:
            self._disconnected_printed = False
            # Add to the total button time if channel 6 is pressed, else reset it
            self.btnTime += (self.millis() - self.loopTimeI) if chnVal[5] else -self.btnTime
            self.loopTimeI = self.millis()

            if self.btnTime >= 1000:
                self.btnTime = 0
                if self.state == 0:
                    self.home()
                    self.state = 1
                elif self.state == 1:
                    xNom, yNom, zNom = self.xNom, self.yNom, self.zNom
                    self.moveFeet(xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, LINEAR, 500)
                    self.state = 3
                elif self.state == 3:
                    self.moveFeet(xStart, yStart, zStart, xStart, yStart, zStart, xStart, yStart, zStart,
                                  xStart, yStart, zStart, LINEAR, 500)
                    self.state = 1

            if self.state == 1 and chnVal[4] == -1:
                for i in range(12):
                    self.setState(i, 0)
                self.state = 2
            elif self.state == 2 and chnVal[4] == 1:
                for i in range(12):
                    self.setState(i, 1)
                self.moveFeet(xStart, yStart, zStart, xStart, yStart, zStart, xStart, yStart, zStart,
                              xStart, yStart, zStart, LINEAR, 1000)
                self.state = 1

            if self.state == 3:
                if chnVal[4] == 0 and self.zNom == self.zLow:
                    self.zNom = self.zDefault
                    self.zHome[:] = [self.zNom] * 4
                    xNom, yNom, zNom = self.xNom, self.yNom, self.zNom
                    self.moveFeet(xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, LINEAR, 500)
                if self.zNom == self.zDefault:
                    if chnVal[4] == 1:
                        self.stepInPlace(100, 50)
                    if chnVal[4] == -1:
                        self.zNom = self.zLow
                        self.zHome[:] = [self.zNom] * 4
                        xNom, yNom, zNom = self.xNom, self.yNom, self.zNom
                        self.moveFeet(xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, LINEAR, 500)
                    if chnVal[6] == 1 and chnVal[5] == 1:
                        self.kinematicsDemo()
                    if chnVal[6] == -1 and chnVal[5] == 1:
                        self.jump(1)
                    if chnVal[2] == 10:
                        self.tiltRobot(15, 0, 0, CUBIC_OUT, 500)
                        self.tiltRobot(-15, 0, 0, CUBIC_OUT, 1000)
                        self.tiltRobot(0, 0, 0, CUBIC_OUT, 500)
                    if chnVal[2] == -10:
                        self.tiltRobot(0, 15, 0, CUBIC_OUT, 500)
                        self.tiltRobot(0, -15, 0, CUBIC_OUT, 1000)
                        self.tiltRobot(0, 0, 0, CUBIC_OUT, 500)
                st = self.step
                if self.zNom == self.zDefault:
                    st(70, 90, 0, 0, 180, 180, 50, 0)
                    st(70, 90, 180, 180, 0, 0, 50, 1)
                    st(50, 70, -90, 90, -90, 90, 50, 2)
                    st(50, 70, 90, -90, 90, -90, 50, 3)
                    st(50, 70, -45, 45, -135, 135, 50, 4)
                    st(50, 70, -135, 135, -45, 45, 50, 5)
                    st(50, 70, 45, -45, 135, -135, 50, 6)
                    st(50, 70, 135, -135, 45, -45, 50, 7)
                    st(70, 90, -45, 135, 135, -45, 75, 8)
                    st(70, 90, 135, -45, -45, 135, 75, 9)
                elif self.zNom == self.zLow:
                    st(50, 70, 0, 0, 180, 180, 50, 0)
                    st(50, 70, 180, 180, 0, 0, 50, 1)
                    st(50, 70, -90, 90, -90, 90, 50, 2)
                    st(50, 70, 90, -90, 90, -90, 50, 3)
                    st(50, 70, -45, 45, -135, 135, 50, 4)
                    st(50, 70, -135, 135, -45, 45, 50, 5)
                    st(50, 70, 45, -45, 135, -135, 50, 6)
                    st(50, 70, 135, -135, 45, -45, 50, 7)
                    st(50, 70, -45, 135, 135, -45, 75, 8)
                    st(50, 70, 135, -45, -45, 135, 75, 9)
        else:
            if not self._disconnected_printed:      # PORT NOTE: firmware prints every loop
                self.hal.serial_println("DISCONNECTED")
                self._disconnected_printed = True

    def isrHandler(self, widths):
        """PORT NOTE: the pulse-width measurement half of isrHandler (digitalRead/micros on
        pin-change interrupts) is done by the HAL, which passes the 8 widths in microseconds.
        The map/constrain half is identical to the firmware."""
        for i in range(8):
            self.width[i] = float(widths[i])
        for i in range(8):
            self.chnVal[i] = _to_int(constrain(_map(self.width[i], 1000 / sensitivity, 2000 * sensitivity,
                                                    mapRange[i][0], mapRange[i][1]),
                                               constrainRange[i][0], constrainRange[i][1]))

    def getTestingInput(self):
        ch = self.hal.serial_read()
        if not ch:
            return
        xNom, yNom, zNom = self.xNom, self.yNom, self.zNom
        if ch == 'h':
            self.home()
            self.delay(500)
            self.moveFeet(xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, LINEAR, 500)
            self.state = 3
        elif ch == 'c':
            for i in range(12):
                self.setState(i, 1)
            self.state = 3
        elif ch == 'n':
            self.moveFeet(xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, LINEAR, 1000)
        elif ch == 'i':
            for i in range(12):
                self.setState(i, 0)
            self.state = 2
        elif ch == 'w':
            self.tiltRobot(-10, 10, 15, LINEAR, 1000)
        elif ch == 'e':
            self.tiltRobot(-10, -10, 15, LINEAR, 1000)
            # (unreachable jump(1) after break in the firmware)
        # 'q' (rotate while speaking) omitted: a 5x repeat of tilt sequences, see Functions.ino
