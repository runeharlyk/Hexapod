"""Distance and turn motions, written once against the Robot's velocity, state and clock."""
import math

from .backends.base import Mode, RobotError, RobotTimeout

# After a stop the firmware's command ramp (GAIT_COMMAND_TAU_S = 0.15 s) takes ~0.5 s to bring the stride under its
# 2 mm deadband, and the feet then ease back to stance.
SETTLE_S = 0.6
POLL_S = 0.05
SLOW_DOWN_DEG = 30.0
MIN_RATE_FRACTION = 0.2
TOLERANCE_DEG = 3.0


def _open_loop(robot, vx: float, vy: float, yaw_rate: float, amount: float) -> None:
    """Holds the velocity for `amount` (metres or radians) over the commanded speed, then stops.

    The command ramp's lag while starting roughly equals its overrun while stopping, so the amount covered is about
    speed times time (the simulation falls a near-constant 1-2 cm short, test_sim_backend.py). That holds only while the robot walks and hears every command throughout: a link that stalled past its
    dead-man timeout, or someone switching the robot out of WALK, may have stopped it partway, and an open-loop
    motion cannot tell how far it got, so it says so.
    """
    drops = robot.state().link_drops
    commanded = robot.set_velocity(vx, vy, yaw_rate)
    speed = abs(commanded.yaw_rate) if yaw_rate else math.hypot(commanded.vx, commanded.vy)
    robot.sleep(amount / speed)
    robot.stop()
    robot.sleep(SETTLE_S)
    state = robot.state()
    if state.mode is not Mode.WALK:
        raise RobotError(f"the robot was switched to {state.mode.name} during the motion: it moved less than asked")
    if state.link_drops > drops:
        raise RobotError("the link stalled past the robot's dead-man timeout: it may have moved less than asked")


def move(robot, forward: float, sideways: float, speed: float | None) -> None:
    """Walks `forward` and `sideways` metres (one of them zero) at `speed`, by default half the gait's top speed."""
    distance = math.hypot(forward, sideways)
    if distance == 0:
        return
    top = robot.max_velocity()
    v = speed if speed is not None else (top.vx if sideways == 0 else top.vy) / 2
    if v <= 0:
        raise ValueError(f"a move needs a positive speed, not {v}")
    _open_loop(robot, v * forward / distance, v * sideways / distance, 0.0, distance)


def rotate(robot, degrees: float, rate: float | None) -> None:
    """Turns `degrees` counter-clockwise at `rate` rad/s (by default half the gait's top).

    With an IMU the turn closes on the measured yaw, slowing over the last SLOW_DOWN_DEG and stopping within
    TOLERANCE_DEG; without one it is timed like a move.
    """
    if degrees == 0:
        return
    top = rate if rate is not None else robot.max_velocity().yaw_rate / 2
    if top <= 0:
        raise ValueError(f"a turn needs a positive rate, not {top}")
    if robot.state().yaw is None:
        _open_loop(robot, 0.0, 0.0, math.copysign(top, degrees), math.radians(abs(degrees)))
        return
    expected = math.radians(abs(degrees)) / top + math.radians(SLOW_DOWN_DEG) / (MIN_RATE_FRACTION * top)
    start = robot.time()
    last = robot.state().yaw
    turned = 0.0
    while True:
        yaw = robot.state().yaw
        turned += (yaw - last + 180) % 360 - 180
        last = yaw
        remaining = degrees - turned
        if abs(remaining) <= TOLERANCE_DEG:
            break
        if robot.time() - start > 2 * expected:
            robot.stop()
            raise RobotTimeout(f"turned {turned:.0f} of {degrees:.0f} degrees in {2 * expected:.1f} s")
        fraction = min(1.0, max(MIN_RATE_FRACTION, abs(remaining) / SLOW_DOWN_DEG))
        robot.set_velocity(0.0, 0.0, math.copysign(top * fraction, remaining))
        robot.sleep(POLL_S)
    robot.stop()
    robot.sleep(SETTLE_S)
