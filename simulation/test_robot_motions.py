"""The Robot facade and its motions on a scripted backend: a virtual clock that integrates the commanded velocity."""
import math
from collections import deque

import pytest

from src.hexapod import Gait, Mode, Robot, RobotError, RobotTimeout, Velocity
from src.hexapod import motions
from src.hexapod.backends.base import ZERO, RobotState

DT = 0.01


class ScriptedBackend:
    """Integrates the commanded REP-103 velocity into a pose while walking, optionally with a yaw that arrives late,
    a gait that turns slower than commanded, a yaw that drifts by `stop_drifts` (degrees, in the turn's direction, one
    per stop of a turn) while the feet settle, a dead-man stop at `drop_at`, a mode switched from elsewhere and no IMU;
    records every command."""

    def __init__(self, top=Velocity(0.3, 0.29, 1.5), yaw=0.0, yaw_lag_s=0.0, turn_scale=1.0, drop_at=None,
                 imu=True, mode=Mode.DEACTIVATED, refuse=(), switch_at=None, stop_drifts=()):
        self.top, self.turn_scale, self.drop_at, self.imu, self.refuse = top, turn_scale, drop_at, imu, refuse
        self.stop_drifts = list(stop_drifts)
        self.switch_at = switch_at  # (time, mode): someone else changes the mode, as the app's stop button does
        self.link_drops = 0
        self.t, self.x, self.y, self.yaw = 0.0, 0.0, 0.0, yaw
        self.mode = mode
        self.gait = Gait.TRI_GATE
        self.velocity = ZERO
        self._turning = 0.0  # the yaw rate of the turn a stop would end, until a stop takes its drift
        self.log = []  # (t, what, value) for every command, in order
        self.closed = False
        self._seen = deque([yaw] * (round(yaw_lag_s / DT) + 1))

    def connect(self):
        pass

    def close(self):
        self.closed = True

    def set_mode(self, mode):
        self.log.append((self.t, "mode", mode))
        if mode in self.refuse:
            raise RobotTimeout(f"the robot did not report {mode.name}")
        self.mode = mode

    def set_gait(self, gait):
        self.log.append((self.t, "gait", gait))
        self.gait = gait

    def set_height(self, height):
        self.log.append((self.t, "height", height))

    def max_velocity(self):
        return self.top

    def set_velocity(self, v):
        clamp = lambda value, limit: max(-limit, min(limit, value))
        self.velocity = Velocity(
            clamp(v.vx, self.top.vx), clamp(v.vy, self.top.vy), clamp(v.yaw_rate, self.top.yaw_rate)
        )
        self.log.append((self.t, "velocity", self.velocity))
        if self.velocity.yaw_rate:
            self._turning = self.velocity.yaw_rate
        elif self.velocity == ZERO and self._turning and self.stop_drifts and self.mode is Mode.WALK:
            self.yaw += math.copysign(self.stop_drifts.pop(0), self._turning)
            self._turning = 0.0
        return self.velocity

    def state(self):
        seen = (self._seen[0] + 180) % 360 - 180 if self.imu else None
        return RobotState(self.t, self.mode, self.gait, seen, self.link_drops)

    def now(self):
        return self.t

    # Whole 10 ms steps, then the remainder, so a move's distance comes out exact.
    def sleep(self, seconds):
        steps, rest = divmod(seconds, DT)
        for dt in [DT] * int(steps) + ([rest] if rest > 1e-9 else []):
            self._advance(dt)

    def _advance(self, dt):
        v = self.velocity if self.mode is Mode.WALK else ZERO
        heading = math.radians(self.yaw)
        self.x += (v.vx * math.cos(heading) - v.vy * math.sin(heading)) * dt
        self.y += (v.vx * math.sin(heading) + v.vy * math.cos(heading)) * dt
        self.yaw += math.degrees(v.yaw_rate * self.turn_scale) * dt
        if self.drop_at is not None and self.t < self.drop_at <= self.t + dt:
            self.link_drops += 1
        if self.switch_at is not None and self.t < self.switch_at[0] <= self.t + dt:
            self.mode = self.switch_at[1]
        self.t += dt
        self._seen.append(self.yaw)
        self._seen.popleft()

    def logged(self, what):
        return [value for _, what_, value in self.log if what_ == what]


def _robot(**options):
    backend = ScriptedBackend(**options)
    return Robot(backend=backend), backend


def test_connecting_puts_the_robot_in_the_scripts_gait():
    backend = ScriptedBackend()
    with Robot(gait=Gait.TUNED, backend=backend):
        assert backend.logged("gait") == [Gait.TUNED]


def test_a_velocity_starts_the_walk_and_a_stop_keeps_walking_mode():
    robot, backend = _robot()
    robot.stand()
    robot.set_velocity(0.05, 0, 0)
    assert backend.logged("mode") == [Mode.STAND, Mode.WALK]
    robot.stop()
    assert backend.velocity == ZERO and backend.mode is Mode.WALK


# The app's stop button deactivates the robot and IDLE/POSE/ANIMATE are the user's; an unknown mode may be either.
@pytest.mark.parametrize("mode", [None, Mode.DEACTIVATED, Mode.IDLE, Mode.ANIMATE])
def test_a_robot_the_script_did_not_stand_is_not_walked(mode):
    robot, backend = _robot(mode=mode)
    with pytest.raises(RobotError):
        robot.set_velocity(0.05, 0, 0)
    assert backend.logged("mode") == [] and backend.velocity == ZERO


def test_stand_takes_a_height_in_millimetres_up():
    robot, backend = _robot()
    robot.stand(height_mm=25)
    assert backend.logged("height") == [-0.5]  # the firmware's height input lowers the body when positive
    with pytest.raises(ValueError):
        robot.stand(height_mm=60)


# An open-loop move cannot tell how far the robot walked while its dead-man stop held it: it must say so.
def test_a_move_cut_by_the_dead_man_raises_after_stopping():
    robot, backend = _robot(mode=Mode.STAND, drop_at=1.0)
    with pytest.raises(RobotError):
        robot.move_forward(0.5)
    assert backend.velocity == ZERO


def test_a_move_the_app_stopped_midway_raises():
    robot, _ = _robot(mode=Mode.STAND, switch_at=(1.0, Mode.DEACTIVATED))
    with pytest.raises(RobotError, match="DEACTIVATED"):
        robot.move_forward(0.5)


def test_a_move_runs_half_the_top_speed_for_its_distance():
    robot, backend = _robot(mode=Mode.STAND)
    robot.move_forward(0.6)
    (start, _, v), (end, _, stop) = [entry for entry in backend.log if entry[1] == "velocity"][-2:]
    assert v == Velocity(0.15, 0, 0) and stop == ZERO
    assert end - start == pytest.approx(0.6 / 0.15)
    assert (backend.x, backend.y) == pytest.approx((0.6, 0.0), abs=1e-3)


def test_moves_go_back_and_to_the_left():
    robot, backend = _robot(mode=Mode.STAND)
    robot.move_forward(-0.2)
    robot.move_sideways(0.1)
    assert (backend.x, backend.y) == pytest.approx((-0.2, 0.1), abs=1e-3)
    assert backend.logged("velocity")[-2].vy == pytest.approx(0.29 / 2)  # sideways defaults to the lateral top


def test_a_move_beyond_the_top_speed_still_covers_its_distance():
    robot, backend = _robot(mode=Mode.STAND)
    robot.move_forward(0.3, speed=5.0)
    assert backend.x == pytest.approx(0.3, abs=1e-3)


# The robot's yaw arrives late and its gait turns slower than commanded: the turn must still land on target.
def test_a_lagging_turn_slows_down_and_lands_on_target():
    robot, backend = _robot(mode=Mode.STAND, yaw_lag_s=0.2, turn_scale=0.9)
    robot.rotate(90)
    assert backend.yaw == pytest.approx(90, abs=3)
    last_turn = [v.yaw_rate for v in backend.logged("velocity") if v.yaw_rate][-1]
    assert abs(last_turn) < 0.35 * backend.top.yaw_rate
    start = backend.yaw
    robot.rotate(-90)
    assert backend.yaw - start == pytest.approx(-90, abs=3)


# A pass that stopped at the edge of the tolerance would leave the settle drift free to push the yaw out of it.
def test_a_turn_pass_runs_on_to_the_target_not_to_the_edge_of_the_tolerance():
    robot, backend = _robot(mode=Mode.STAND)
    robot.rotate(90)
    assert backend.yaw == pytest.approx(90, abs=1.0)


def test_a_turn_whose_stop_lands_off_target_is_corrected_once_settled():
    robot, backend = _robot(mode=Mode.WALK, stop_drifts=[8.0, 0.0, 0.0])
    robot.rotate(90)
    assert backend.yaw == pytest.approx(90, abs=motions.TOLERANCE_DEG)
    assert backend.stop_drifts == [0.0]  # two stops: the second pass corrected the first's overshoot


def test_a_turn_that_keeps_landing_off_target_gives_up_after_its_passes():
    robot, backend = _robot(mode=Mode.WALK, stop_drifts=[8.0] * 10)
    robot.rotate(90)
    assert len(backend.stop_drifts) == 10 - motions.MAX_TURN_PASSES


def test_a_stop_drift_is_taken_by_the_stop_of_a_turn_only():
    robot, backend = _robot(mode=Mode.WALK, stop_drifts=[5.0, 7.0])
    backend.set_velocity(Velocity(0.0, 0.0, 1.0))
    backend.set_velocity(ZERO)
    backend.set_velocity(ZERO)
    backend.set_velocity(Velocity(0.1, 0.0, 0.0))
    backend.set_velocity(ZERO)
    assert backend.stop_drifts == [7.0]


def test_a_turn_across_the_wrap_and_a_full_circle():
    robot, backend = _robot(mode=Mode.STAND, yaw=170.0)
    robot.rotate(40)
    assert (backend.yaw + 180) % 360 - 180 == pytest.approx(-150, abs=3)
    start = backend.yaw
    robot.rotate(360)
    assert backend.yaw - start == pytest.approx(360, abs=3)


def test_without_an_imu_a_turn_is_timed():
    robot, backend = _robot(mode=Mode.STAND, imu=False)
    robot.rotate(-120)
    assert backend.yaw == pytest.approx(-120, abs=0.01)
    assert {v.yaw_rate for v in backend.logged("velocity")} == {-0.75, 0.0}


def test_a_move_or_turn_without_speed_is_refused():
    robot, _ = _robot(mode=Mode.STAND)
    with pytest.raises(ValueError):
        robot.move_forward(0.1, speed=0)
    with pytest.raises(ValueError):
        robot.rotate(10, rate=0)


def test_a_turn_that_never_comes_times_out_and_stops():
    robot, backend = _robot(mode=Mode.STAND, turn_scale=0.0)
    with pytest.raises(RobotTimeout):
        robot.rotate(45)
    assert backend.velocity == ZERO


# The firmware swaps the leg schedule only once the gait stopped stepping, so a gait change stops and waits.
def test_a_gait_change_stops_first_and_waits_for_the_swap():
    robot, backend = _robot(mode=Mode.STAND)
    robot.set_velocity(0.1, 0, 0)
    robot.set_gait(Gait.BI_GATE)
    assert [what for _, what, _ in backend.log][-2:] == ["velocity", "gait"]
    assert backend.velocity == ZERO and backend.t == pytest.approx(motions.SETTLE_S)


def test_an_exception_stops_sits_and_deactivates():
    robot, backend = _robot()
    with pytest.raises(KeyboardInterrupt):
        with robot:
            robot.stand()
            robot.set_velocity(0.05, 0, 0.2)
            raise KeyboardInterrupt
    tail = [(what, value) for _, what, value in backend.log][-4:]
    assert tail == [("velocity", ZERO), ("mode", Mode.STAND), ("height", 1.0), ("mode", Mode.DEACTIVATED)]
    assert backend.closed


def test_leaving_a_deactivated_robot_does_not_wake_it():
    robot, backend = _robot()
    with robot:
        pass
    assert backend.logged("mode") == [Mode.DEACTIVATED] and backend.closed


# A robot that will not sit (the mode request timed out) must still be switched off on the way out.
def test_a_failed_sit_still_deactivates():
    robot, backend = _robot(mode=Mode.WALK, refuse=(Mode.STAND,))
    with robot:
        pass
    assert backend.logged("mode") == [Mode.STAND, Mode.DEACTIVATED] and backend.mode is Mode.DEACTIVATED
    assert backend.closed
