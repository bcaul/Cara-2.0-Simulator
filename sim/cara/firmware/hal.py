"""Hardware abstraction layer (HAL) between the ported firmware and a robot.

The firmware port (cara_firmware.py) only talks to the robot through this interface,
using the same calls the Teensy code uses (millis, delay, odrive[i]->setPosition, ...).
Two implementations are planned:

  * SimHAL  (cara/sim_backend/sim_hal.py)  - MuJoCo, virtual clock in lockstep with physics
  * RealHAL (cara/hooks/real_hal.py, Phase 4 stub) - serial/CAN to the real robot

Nothing in this file imports MuJoCo.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

# ODrive enums (ODrive 0.5.x / ODriveArduino naming)
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_POS_FILTER = 3

# BNO08x report id (SparkFun library)
SENSOR_REPORTID_ROTATION_VECTOR = 0x05


@dataclass
class Get_Encoder_Estimates_msg_t:
    Pos_Estimate: float = 0.0     # motor turns
    Vel_Estimate: float = 0.0     # motor turns/s


@dataclass
class Get_Iq_msg_t:
    Iq_Setpoint: float = 0.0
    Iq_Measured: float = 0.0


@dataclass
class Heartbeat_msg_t:
    Axis_Error: int = 0
    Axis_State: int = AXIS_STATE_IDLE


class ODriveCANBase(ABC):
    """Subset of the ODriveCAN Arduino API used by the CARA firmware (one object per axis)."""

    @abstractmethod
    def clearErrors(self): ...

    @abstractmethod
    def setState(self, requested_state: int): ...

    @abstractmethod
    def setControllerMode(self, control_mode: int, input_mode: int): ...

    @abstractmethod
    def setPosition(self, position: float, velocity_feedforward: float = 0.0, torque_feedforward: float = 0.0): ...

    @abstractmethod
    def setVelocity(self, velocity: float, torque_feedforward: float = 0.0): ...

    @abstractmethod
    def getCurrents(self, msg: Get_Iq_msg_t) -> bool:
        """Blocking request/response (fills msg). Consumes bus time."""

    # Callback registration (ODriveCAN::onFeedback / onStatus)
    def onFeedback(self, callback, user_data):
        self._feedback_cb = (callback, user_data)

    def onStatus(self, callback, user_data):
        self._status_cb = (callback, user_data)


class IMUBase(ABC):
    """Subset of the SparkFun BNO08x API used by the firmware."""

    def begin(self, *args) -> bool:
        return True

    @abstractmethod
    def enableRotationVector(self, timeBetweenReports_ms: int = 10) -> bool: ...

    @abstractmethod
    def wasReset(self) -> bool: ...

    @abstractmethod
    def getSensorEvent(self) -> bool: ...

    @abstractmethod
    def getSensorEventID(self) -> int: ...

    @abstractmethod
    def getRoll(self) -> float: ...      # rad

    @abstractmethod
    def getPitch(self) -> float: ...     # rad


class FirmwareStopped(Exception):
    """Raised inside the firmware thread when the HAL is shut down."""


class HAL(ABC):
    """Board-level services: time, CAN pumping, serial, RC interrupts."""

    odrive: list          # 12 ODriveCANBase objects, CAN node ids 0..11
    imu: IMUBase

    @abstractmethod
    def millis(self) -> int: ...

    @abstractmethod
    def micros(self) -> int: ...

    @abstractmethod
    def delay(self, ms: float): ...

    @abstractmethod
    def pumpEvents(self):
        """Process incoming CAN frames (fires onFeedback/onStatus callbacks)."""

    def serial_println(self, text: str):
        print(text)

    def serial_read(self):
        """Serial.available() ? Serial.read() : None (used by getTestingInput)."""
        return None

    @abstractmethod
    def attach_rc_interrupt(self, isr):
        """Register the RC receiver ISR. The HAL measures the 8 pulse widths (us) and calls
        isr(widths) whenever a new frame arrives (the Teensy measures them with pin-change
        interrupts in isrHandler; that measuring part is hardware-specific)."""
