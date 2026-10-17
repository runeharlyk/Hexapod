"""What every backend offers the Robot facade, and the values that cross it.

Velocities are REP-103: x forward, y to the left, yaw counter-clockwise seen from above. The robot's own body frame
is X to the right, Y forward, Z up (firmware/include/kinematics.h), so a backend converts at its boundary:
REP x = body Y, REP y = -body X, REP yaw = yaw about body Z (both frames are right-handed with the same up axis).
Goals and the odometry cross it the same way, with their angles in degrees on this side and radians on the robot's.
"""
import math
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


class GoalKind(IntEnum):
    """firmware GoalKind, by wire value (platform_shared/message.proto)."""

    FORWARD = 0
    SIDEWAYS = 1
    TURN = 2
    ARC = 3


class GoalState(IntEnum):
    """firmware GoalState, by wire value (platform_shared/message.proto)."""

    IDLE = 0
    SETTLING = 1
    RUNNING = 2
    REACHED = 3
    CANCELLED = 4
    TIMED_OUT = 5
    REFUSED = 6


ENDED = (GoalState.REACHED, GoalState.CANCELLED, GoalState.TIMED_OUT, GoalState.REFUSED)


@dataclass(frozen=True)
class Goal:
    """A move the robot walks on its own odometry, in REP-103: FORWARD `distance` m forward, SIDEWAYS `distance` m to
    the left, TURN `angle` degrees counter-clockwise, ARC forward along a circle of `radius` m through `angle` degrees,
    counter-clockwise (curving left) when positive."""

    kind: GoalKind
    distance: float = 0.0
    angle: float = 0.0
    radius: float = 0.0

    def in_body_frame(self) -> tuple[GoalKind, float, float, float]:
        """kind, distance (m, SIDEWAYS positive right), angle (rad) and radius (m), as GoalMove carries them."""
        distance = -self.distance if self.kind is GoalKind.SIDEWAYS else self.distance
        return self.kind, distance, math.radians(self.angle), self.radius


@dataclass(frozen=True)
class Style:
    """How a goal walks, in the firmware's slider terms: the gait, `speed` a share in (0, 1] of the gait's top speed at
    the posture (AUTO's is BI_GATE's), and the stride, step height (s1), height (h, positive is LOWER) and feet
    distance (fd) sliders, each in [-1, 1]."""

    gait: Gait = Gait.TRI_GATE
    speed: float = 0.5
    stride: float = 0.0
    s1: float = 0.0
    h: float = 0.0
    fd: float = 0.0


@dataclass(frozen=True)
class GoalStatus:
    id: int  # the goal's number since the robot booted
    state: GoalState
    kind: GoalKind
    target: float  # as the Goal's: m forward or to the left, or degrees counter-clockwise
    progress: float  # share of the target covered, 0 to 1
    x: float  # the odometry pose, as Odometry's
    y: float
    yaw: float
    passes: int  # passes started, the first included

    @classmethod
    def from_body(cls, id, state, kind, target, progress, x, y, yaw, passes) -> "GoalStatus":
        kind = GoalKind(kind)
        if kind is GoalKind.SIDEWAYS:
            target = -target
        elif kind in (GoalKind.TURN, GoalKind.ARC):
            target = math.degrees(target)
        return cls(int(id), GoalState(state), kind, target, progress, y, -x, math.degrees(yaw), int(passes))

    @property
    def ended(self) -> bool:
        return self.state in ENDED


@dataclass(frozen=True)
class Odometry:
    """The robot's own estimate of its pose in the frame it faced at the last reset."""

    x: float  # m forward
    y: float  # m to the left
    yaw: float  # degrees counter-clockwise, the gyro fused in
    kinematic_yaw: float  # degrees, as yaw, from the feet alone
    distance: float  # m travelled along the path

    @classmethod
    def from_body(cls, x, y, yaw, kinematic_yaw, distance) -> "Odometry":
        return cls(y, -x, math.degrees(yaw), math.degrees(kinematic_yaw), distance)


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
    odometry: Odometry | None = None  # None from a robot that takes no goals


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
    def supports_goals(self) -> bool: ...
    # Centres the sticks, which the goal takes over, and keeps the style's posture in every input after it; raises
    # RobotError when the robot refuses the goal.
    def goal_move(self, goal: Goal, style: Style) -> GoalStatus: ...
    def goal_cancel(self) -> None: ...
    def goal_status(self) -> GoalStatus: ...  # the newest goal's
    def odometry(self) -> Odometry: ...
    def reset_odometry(self) -> None: ...  # takes effect on the robot's next control tick
