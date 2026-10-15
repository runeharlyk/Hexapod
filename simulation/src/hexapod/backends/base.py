"""What every backend offers the Robot facade, and the values that cross it.

Velocities are REP-103: x forward, y to the left, yaw counter-clockwise seen from above. The robot's own body frame
is X to the right, Y forward, Z up (firmware/include/kinematics.h), so a backend converts at its boundary:
REP x = body Y, REP y = -body X, REP yaw = yaw about body Z (both frames are right-handed with the same up axis).
"""
from dataclasses import dataclass
from enum import IntEnum
from typing import Protocol

from ..servos import ServoCalibration


class Mode(IntEnum):
    """firmware MOTION_STATE, by wire value (platform_shared/message.proto ModesEnum)."""

    DEACTIVATED = 0
    IDLE = 1
    POSE = 2
    STAND = 3
    WALK = 4
    WALK_NN = 5
    ANIMATE = 6


class Gait(IntEnum):
    """firmware GaitType, by wire value (platform_shared/message.proto GaitEnum)."""

    TRI_GATE = 0
    BI_GATE = 1
    WAVE = 2
    RIPPLE = 3
    TUNED = 4
    AUTO = 5


@dataclass(frozen=True)
class Velocity:
    vx: float  # m/s forward
    vy: float  # m/s to the left
    yaw_rate: float  # rad/s counter-clockwise


ZERO = Velocity(0.0, 0.0, 0.0)


@dataclass(frozen=True)
class RobotState:
    t: float  # seconds on the backend's clock
    mode: Mode | None  # None until the robot has reported one: the firmware pushes its mode only on a change
    gait: Gait | None
    yaw: float | None  # degrees counter-clockwise, None without an IMU
    link_drops: int  # times the robot's dead-man stop may have fired since connecting
    roll: float | None = None  # degrees, left side up (REP-103), None without an IMU


class RobotError(Exception):
    pass


class RobotTimeout(RobotError):
    pass


class RobotDisconnected(RobotError):
    pass


class Backend(Protocol):
    def connect(self) -> None: ...
    def close(self) -> None: ...
    def set_mode(self, mode: Mode) -> None: ...  # returns once the robot reports the mode
    def set_gait(self, gait: Gait) -> None: ...
    def set_height(self, height: float) -> None: ...  # the firmware's height input: -1 to 1, positive is LOWER
    def set_velocity(self, velocity: Velocity) -> Velocity: ...  # returns what was commanded, after clamping
    def max_velocity(self) -> Velocity: ...
    def state(self) -> RobotState: ...
    def now(self) -> float: ...
    def sleep(self, seconds: float) -> None: ...
    def servo_calibration(self) -> ServoCalibration: ...
    def set_servo_pwm(self, servo: int, pwm: int) -> None: ...  # ServoPWMData: one servo's raw PCA9685 counts
