"""Distance and turn motions, written once against the Robot: goals the robot walks on its own odometry when it takes
them, otherwise timed on its velocity, state and clock."""
import math

from src.robot.firmware_gait import FEET_EASE_S, GaitCommandRamp, GaitState, gait_is_commanded

from . import speed_model
from .backends.base import Gait, Mode, RobotError, RobotTimeout, Velocity

CONTROL_TICK_S = 0.005  # the firmware's control period (firmware/src/main.cpp)


def _stop_seconds(stride_mm: float) -> float:
    """How long the firmware's stop (GaitCommandRamp, gait.h) takes to bring `stride_mm` under the stride deadband."""
    ramp, live, stop = GaitCommandRamp(), GaitState(step_y=stride_mm, phase_rate=1.0), GaitState()
    ticks = 0
    while gait_is_commanded(live):
        ramp.step(live, stop, CONTROL_TICK_S)
        ticks += 1
    return ticks * CONTROL_TICK_S


# After a stop the firmware's command ramp brings a full 100 mm stride under its deadband, and the feet then ease back
# to stance.
SETTLE_S = _stop_seconds(100.0) + FEET_EASE_S
POLL_S = 0.05
SLOW_DOWN_DEG = 30.0
MIN_RATE_FRACTION = 0.2
TOLERANCE_DEG = 3.0
AIM_DEG = 0.5  # about one poll step at the slowest turn rate
MAX_TURN_PASSES = 3


def _open_loop(robot, vx: float, vy: float, yaw_rate: float, amount: float) -> None:
    """Holds the velocity for `amount` (metres or radians) over the commanded speed, then stops.

    That covers the amount only while the robot walks and hears every command throughout: a link that stalled past
    its dead-man timeout, or someone switching the robot out of WALK, may have stopped it partway, and an open-loop
    motion cannot tell how far it got, so it says so.
    """
    drops = robot.state().link_drops
    commanded = robot.set_velocity(vx, vy, yaw_rate)
    speed = abs(commanded.yaw_rate) if yaw_rate else math.hypot(commanded.vx, commanded.vy)
    # No lead for the command ramp: it predicts a net overshoot of 0.25 tau (the start loses tau of travel, the stop
    # coasts 1.25 tau), 4 mm at half speed, but the simulation shows none. The gait phase the stop lands on scatters a
    # move by about 1 cm, and a lead only made the mean shorter.
    robot.sleep(amount / speed)
    robot.stop()
    robot.sleep(SETTLE_S)
    state = robot.state()
    if state.mode is not Mode.WALK:
        raise RobotError(f"the robot was switched to {state.mode.name} during the motion: it moved less than asked")
    if state.link_drops > drops:
        raise RobotError("the link stalled past the robot's dead-man timeout: it may have moved less than asked")


def _goal_top(gait: Gait) -> Velocity:
    """The speed model's tops for a goal's gait; AUTO's goal speed is a share of BI_GATE's top, as on the robot."""
    return speed_model.max_velocity(Gait.BI_GATE if gait is Gait.AUTO else gait)


def _share(speed: float, top: float) -> float:
    if speed <= 0:
        raise ValueError(f"a motion needs a positive speed, not {speed}")
    return min(1.0, speed / top)


def move(robot, forward: float, sideways: float, speed: float | None) -> None:
    """Walks `forward` and `sideways` metres (one of them zero) at `speed` m/s. By default a goal walks the style's
    share of the gait's top speed (half of it unless set) and a timed move half the gait's top."""
    distance = math.hypot(forward, sideways)
    if distance == 0:
        return
    if robot.takes_goals():
        share = None
        if speed is not None:
            top = _goal_top(robot.style().gait)
            share = _share(speed, top.vx if sideways == 0 else top.vy)
        robot.walk(forward if sideways == 0 else sideways, sideways=sideways != 0, speed=share)
        return
    top = robot.max_velocity()
    v = speed if speed is not None else (top.vx if sideways == 0 else top.vy) / 2
    if v <= 0:
        raise ValueError(f"a move needs a positive speed, not {v}")
    _open_loop(robot, v * forward / distance, v * sideways / distance, 0.0, distance)


def rotate(robot, degrees: float, rate: float | None) -> None:
    """Turns `degrees` counter-clockwise at `rate` rad/s, by default the style's share of the gait's top for a goal and
    half the gait's top otherwise.

    A robot that takes goals closes the turn itself, on its gyro-fused odometry, from the heading the goals so far
    meant (Robot.turn). Otherwise, with an IMU the turn closes
    on the measured yaw: each pass slows over the last SLOW_DOWN_DEG and runs on to within AIM_DEG, then the yaw is
    read again once the gait has settled and another pass turns the rest until it is within TOLERANCE_DEG, up to
    MAX_TURN_PASSES. A pass that stopped at the edge of the tolerance would leave the settle drift free to push the yaw
    out of it. The gait holds its stroke at any rate, and after the stop its feet return to stance, which turns the
    body by an amount that depends on the gait phase, well beyond the mean coast of 1.25 tau at the commanded rate. A
    lead on the stop would correct only that mean, so the turn is closed on the settled yaw instead. Without an IMU the
    turn is timed like a move.
    """
    if degrees == 0:
        return
    if robot.takes_goals():
        share = None if rate is None else _share(rate, _goal_top(robot.style().gait).yaw_rate)
        robot.turn(degrees, speed=share)
        return
    top = rate if rate is not None else robot.max_velocity().yaw_rate / 2
    if top <= 0:
        raise ValueError(f"a turn needs a positive rate, not {top}")
    if robot.state().yaw is None:
        _open_loop(robot, 0.0, 0.0, math.copysign(top, degrees), math.radians(abs(degrees)))
        return
    last = robot.state().yaw
    turned = 0.0
    for _ in range(MAX_TURN_PASSES):
        remaining = degrees - turned
        if abs(remaining) <= TOLERANCE_DEG:
            return
        expected = math.radians(abs(remaining)) / top + math.radians(SLOW_DOWN_DEG) / (MIN_RATE_FRACTION * top)
        start = robot.time()
        while abs(remaining) > AIM_DEG:
            if robot.time() - start > 2 * expected:
                robot.stop()
                raise RobotTimeout(f"turned {turned:.0f} of {degrees:.0f} degrees in {2 * expected:.1f} s")
            fraction = min(1.0, max(MIN_RATE_FRACTION, abs(remaining) / SLOW_DOWN_DEG))
            robot.set_velocity(0.0, 0.0, math.copysign(top * fraction, remaining))
            robot.sleep(POLL_S)
            yaw = robot.state().yaw
            turned += (yaw - last + 180) % 360 - 180
            last = yaw
            remaining = degrees - turned
        robot.stop()
        robot.sleep(SETTLE_S)
        yaw = robot.state().yaw
        turned += (yaw - last + 180) % 360 - 180
        last = yaw
