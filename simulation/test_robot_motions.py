"""The Robot facade and its motions on a scripted backend: a virtual clock that integrates the commanded velocity."""
import math
from collections import deque

import pytest

from src.hexapod import Gait, GoalTimedOut, Mode, Robot, RobotError, RobotTimeout, Velocity, speed_model
from src.hexapod import motions
from src.hexapod.backends.base import ZERO, Goal, GoalKind, GoalState, GoalStatus, Odometry, RobotState, Style

DT = 0.01
GOAL_SETTLE_S = 0.1  # the scripted goal's SETTLING before it runs


class ScriptedBackend:
    """Integrates the commanded REP-103 velocity into a pose while walking, optionally with a yaw that arrives late,
    a gait that turns slower than commanded, a yaw that drifts by `stop_drifts` (degrees, in the turn's direction, one
    per stop of a turn) while the feet settle, a dead-man stop at `drop_at`, a mode switched from elsewhere and no IMU;
    records every command. With `goals` it takes goal moves as the firmware does from STAND or WALK: each settles for
    GOAL_SETTLE_S, runs for `goal_s`, then ends in `goal_ends`, unless another client's goal replaces it at
    `replaced_at` (seconds into the goal). A goal that is REACHED has moved the pose by exactly its target, an arc
    only its heading. The link loses the reply to a goal with `move_reply_lost`, and to a status read from
    `status_lost_at` seconds into the goal, as a reply timeout does."""

    def __init__(self, top=Velocity(0.3, 0.29, 1.5), yaw=0.0, yaw_lag_s=0.0, turn_scale=1.0, drop_at=None,
                 imu=True, mode=Mode.DEACTIVATED, refuse=(), switch_at=None, stop_drifts=(), goals=False, goal_s=1.0,
                 goal_ends=GoalState.REACHED, replaced_at=None, move_reply_lost=False, status_lost_at=None):
        self.top, self.turn_scale, self.drop_at, self.imu, self.refuse = top, turn_scale, drop_at, imu, refuse
        self.goals, self.goal_s, self.goal_ends, self.replaced_at = goals, goal_s, goal_ends, replaced_at
        self.move_reply_lost, self.status_lost_at = move_reply_lost, status_lost_at
        self._goal = None  # (id, goal, start time)
        self._goal_moved = False
        self._goal_count = 0
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

    def supports_goals(self):
        return self.goals

    def goal_move(self, goal, style):
        self.log.append((self.t, "goal", (goal, style)))
        if self.mode not in (Mode.STAND, Mode.WALK):
            raise RobotError("the robot refused the request with status 409")
        self.mode = Mode.WALK  # a goal from STAND enters WALK itself
        self._goal_count += 1
        self._goal = (self._goal_count, goal, self.t)
        self._goal_moved = False
        if self.move_reply_lost:
            raise RobotTimeout("no answer from the scripted link")
        return self.goal_status()

    def goal_cancel(self):
        self.log.append((self.t, "cancel", None))

    def goal_status(self):
        number, goal, start = self._goal
        elapsed = self.t - start
        if self.status_lost_at is not None and elapsed >= self.status_lost_at:
            raise RobotTimeout("no answer from the scripted link")
        if self.replaced_at is not None and elapsed >= self.replaced_at:
            number, state, progress = number + 1, GoalState.SETTLING, 0.0
        elif elapsed < GOAL_SETTLE_S:
            state, progress = GoalState.SETTLING, 0.0
        elif elapsed < GOAL_SETTLE_S + self.goal_s:
            state, progress = GoalState.RUNNING, (elapsed - GOAL_SETTLE_S) / self.goal_s
        else:
            state, progress = self.goal_ends, 1.0 if self.goal_ends is GoalState.REACHED else 0.5
            if state is GoalState.REACHED and not self._goal_moved:
                self._goal_moved = True
                heading = math.radians(self.yaw)
                forward, left = (goal.distance, 0.0) if goal.kind is GoalKind.FORWARD else (0.0, goal.distance)
                if goal.kind in (GoalKind.FORWARD, GoalKind.SIDEWAYS):
                    self.x += forward * math.cos(heading) - left * math.sin(heading)
                    self.y += forward * math.sin(heading) + left * math.cos(heading)
                else:
                    self.yaw += goal.angle
        target = goal.distance if goal.kind in (GoalKind.FORWARD, GoalKind.SIDEWAYS) else goal.angle
        return GoalStatus(number, state, goal.kind, target, progress, self.x, self.y, self.yaw, 1)

    def odometry(self):
        return Odometry(self.x, self.y, self.yaw, self.yaw, 0.0)

    def reset_odometry(self):
        self.log.append((self.t, "reset odometry", None))


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


# --- goal moves ----------------------------------------------------------------------------------------------------

def _goals(**options):
    return _robot(mode=Mode.STAND, goals=True, **options)


def test_walk_turn_and_arc_send_rep103_goals_in_the_style():
    robot, backend = _goals()
    style = robot.style(gait=Gait.BI_GATE, speed=0.8, stride=0.5, step_height=30, body_height=10, feet_distance=-0.2)
    assert style == Style(gait=Gait.BI_GATE, speed=0.8, stride=0.5, s1=0.5, h=-0.2, fd=-0.2)
    robot.walk(1.0)
    robot.walk(-0.3, sideways=True)
    robot.turn(90)
    robot.arc(0.5, -180)
    assert backend.logged("goal") == [
        (Goal(GoalKind.FORWARD, distance=1.0), style), (Goal(GoalKind.SIDEWAYS, distance=-0.3), style),
        (Goal(GoalKind.TURN, angle=90), style), (Goal(GoalKind.ARC, angle=-180, radius=0.5), style)]


def test_a_style_keeps_the_fields_it_is_not_given():
    robot, _ = _goals()
    robot.style(speed=0.3, feet_distance=0.4)
    assert robot.style(step_height=10) == Style(speed=0.3, s1=-0.5, fd=0.4)


@pytest.mark.parametrize("field, value", [
    ("speed", 0.0), ("speed", 1.5), ("stride", 1.2), ("step_height", 41), ("step_height", -1), ("body_height", 60),
    ("feet_distance", -1.1)])
def test_a_style_outside_the_sliders_is_refused(field, value):
    robot, _ = _goals()
    with pytest.raises(ValueError):
        robot.style(**{field: value})
    assert robot.style() == Style()


def test_stand_sets_the_styles_body_height():
    robot, backend = _goals()
    robot.stand(height_mm=25)
    robot.walk(0.2)
    (_, style), = backend.logged("goal")
    assert style.h == -0.5


def test_a_goal_blocks_until_reached_and_reports_each_change():
    robot, backend = _goals(goal_s=1.0)
    seen = []
    status = robot.walk(0.5, on_progress=seen.append)
    assert status.state is GoalState.REACHED and seen[-1] == status
    assert backend.t >= GOAL_SETTLE_S + 1.0
    states = [s.state for s in seen]
    assert states[0] is GoalState.SETTLING and GoalState.RUNNING in states
    assert all(a != b for a, b in zip(seen, seen[1:]))  # only changes are reported
    progress = [s.progress for s in seen]
    assert progress == sorted(progress) and len(set(progress)) > 5


@pytest.mark.parametrize("ending, error", [
    (GoalState.CANCELLED, RobotError), (GoalState.TIMED_OUT, GoalTimedOut), (GoalState.REFUSED, RobotError)])
def test_a_goal_that_does_not_reach_raises(ending, error):
    robot, backend = _goals(goal_ends=ending)
    with pytest.raises(error, match=ending.name) as raised:
        robot.turn(45)
    assert (type(raised.value) is GoalTimedOut) == (ending is GoalState.TIMED_OUT)
    assert backend.logged("cancel") == []  # it ended


# The newer goal is another client's: a cancel would end it.
def test_a_goal_replaced_by_another_clients_raises():
    robot, backend = _goals(replaced_at=0.5)
    with pytest.raises(RobotError, match="another goal"):
        robot.walk(1.0)
    assert backend.logged("cancel") == []


# USB serial drops frames: a lost status reply is no TIMED_OUT, and the goal may still be walking, kept alive by the
# keep-alive. It is cancelled, and the RobotTimeout raised is not a GoalTimedOut, so no script retries a goal it
# believes ended.
def test_a_lost_status_reply_cancels_the_goal_and_is_no_goal_timeout():
    robot, backend = _goals(status_lost_at=0.5)
    with pytest.raises(RobotTimeout) as raised:
        robot.walk(1.0)
    assert not isinstance(raised.value, GoalTimedOut)
    assert len(backend.logged("cancel")) == 1


# The robot may have taken a goal whose reply was lost.
def test_a_lost_goal_reply_cancels_the_goal():
    robot, backend = _goals(move_reply_lost=True)
    with pytest.raises(RobotTimeout):
        robot.walk(1.0)
    assert len(backend.logged("cancel")) == 1


def test_a_robot_error_from_the_progress_callback_cancels_the_goal():
    robot, backend = _goals()

    def fail(status):
        if status.state is GoalState.RUNNING:
            raise RobotError("the script's own check failed")

    with pytest.raises(RobotError, match="own check"):
        robot.walk(1.0, on_progress=fail)
    assert len(backend.logged("cancel")) == 1


# Ctrl-C while a goal runs must not leave the robot walking it.
def test_an_interrupted_goal_is_cancelled():
    robot, backend = _goals()

    def interrupt(status):
        if status.state is GoalState.RUNNING:
            raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        robot.walk(1.0, on_progress=interrupt)
    assert len(backend.logged("cancel")) == 1


def test_a_goal_the_robot_refuses_raises():
    robot, backend = _robot(mode=Mode.DEACTIVATED, goals=True)
    with pytest.raises(RobotError, match="409"):
        robot.walk(0.3)
    assert backend.logged("cancel") == []


def test_nothing_to_walk_sends_no_goal_and_an_arc_needs_a_radius():
    robot, backend = _goals()
    robot.walk(0)
    robot.turn(0)
    robot.arc(0.5, 0)
    with pytest.raises(ValueError):
        robot.arc(0, 90)
    with pytest.raises(ValueError):
        robot.arc(-0.5, 90)
    assert backend.logged("goal") == []


def test_a_backend_without_goals_refuses_goal_moves():
    robot, backend = _robot(mode=Mode.STAND)
    with pytest.raises(RobotError, match="goal"):
        robot.walk(0.3)
    assert backend.velocity == ZERO


# m/s and rad/s become a share of the style's gait's top speed; with none given, the style's own share.
def test_moves_and_turns_take_goals_at_their_speed_as_a_share():
    robot, backend = _goals()
    top = speed_model.max_velocity(Gait.TRI_GATE)
    robot.move_forward(0.6, speed=0.1)
    robot.move_sideways(0.2)
    robot.move_forward(-0.3, speed=5.0)
    robot.rotate(-45, rate=0.5)
    sent = [(goal, style.speed) for goal, style in backend.logged("goal")]
    assert sent == [(Goal(GoalKind.FORWARD, distance=0.6), pytest.approx(0.1 / top.vx)),
                    (Goal(GoalKind.SIDEWAYS, distance=0.2), 0.5),
                    (Goal(GoalKind.FORWARD, distance=-0.3), 1.0),
                    (Goal(GoalKind.TURN, angle=-45), pytest.approx(0.5 / top.yaw_rate))]
    assert backend.logged("velocity") == []


def test_a_sideways_share_is_of_the_lateral_top_and_auto_walks_bis():
    robot, backend = _goals()
    robot.style(gait=Gait.AUTO)
    robot.move_sideways(0.2, speed=0.1)
    (_, style), = backend.logged("goal")
    assert style.speed == pytest.approx(0.1 / speed_model.max_velocity(Gait.BI_GATE).vy)
    assert style.gait is Gait.AUTO


def test_a_goal_move_without_speed_is_refused():
    robot, backend = _goals()
    with pytest.raises(ValueError):
        robot.move_forward(0.1, speed=0)
    with pytest.raises(ValueError):
        robot.rotate(10, rate=-1)
    assert backend.logged("goal") == []


def test_the_state_carries_the_odometry_and_it_can_be_reset():
    robot, backend = _goals()
    backend.x, backend.y, backend.yaw = 0.4, -0.1, 30.0
    assert robot.state().odometry == Odometry(0.4, -0.1, 30.0, 30.0, 0.0)
    robot.reset_odometry()
    assert len(backend.logged("reset odometry")) == 1
    plain, _ = _robot()
    assert plain.state().odometry is None


def test_set_gait_is_the_styles_gait():
    robot, backend = _goals()
    robot.set_gait(Gait.WAVE)
    robot.walk(0.1)
    (_, style), = backend.logged("goal")
    assert style.gait is Gait.WAVE


# Each turn takes up the heading drift of the walks before it, so a square closes on the robot's own heading.
def test_a_turn_after_a_walk_that_drifted_turns_the_rest_of_the_way():
    robot, backend = _goals()
    robot.walk(1.0)
    backend.yaw += 2.0  # the walk ended 2 deg to the left of its heading
    robot.turn(90)
    backend.yaw -= 1.0  # and the turn 1 deg short of its 88
    robot.walk(1.0)
    robot.turn(90)
    robot.arc(0.5, 90)
    angles = [goal.angle for goal, _ in backend.logged("goal") if goal.kind is not GoalKind.FORWARD]
    assert angles == pytest.approx([88.0, 91.0, 90.0])


def test_a_full_turn_stays_one_and_a_drift_past_the_wrap_is_the_short_way():
    robot, backend = _goals(yaw=179.0)
    robot.turn(10)
    backend.yaw = -168.0  # 192 deg: 3 past the 189 it meant, across the wrap
    robot.turn(360)
    angles = [goal.angle for goal, _ in backend.logged("goal")]
    assert angles == pytest.approx([10.0, 357.0])


# The heading a script meant is its own goals' only: a velocity walk moves it, and a reset of the odometry zeroes it.
def test_the_intended_heading_follows_velocity_walks_and_odometry_resets():
    robot, backend = _goals()
    robot.turn(90)
    robot.set_velocity(0.0, 0.0, 0.5)
    backend.yaw = 135.0
    robot.turn(90)
    robot.reset_odometry()
    backend.yaw = 4.0  # the odometry zeroed, and the robot since nudged 4 deg
    robot.turn(-90)
    angles = [goal.angle for goal, _ in backend.logged("goal")]
    assert angles == pytest.approx([90.0, 90.0, -94.0])


def test_a_turn_that_did_not_end_forgets_the_heading_it_meant():
    robot, backend = _goals(goal_ends=GoalState.CANCELLED)
    with pytest.raises(RobotError):
        robot.turn(90)
    backend.goal_ends, backend.yaw = GoalState.REACHED, 30.0
    robot.turn(90)
    assert backend.logged("goal")[-1][0].angle == pytest.approx(90.0)


# TUNED has no speed envelope, so the robot refuses its goals: its moves and turns stay timed.
def test_a_tuned_script_moves_and_turns_by_time():
    robot, backend = _goals()
    robot.set_gait(Gait.TUNED)
    robot.move_forward(0.1)
    robot.move_sideways(0.1)
    robot.rotate(30)
    assert backend.logged("goal") == []
    assert {round(v.vx, 3) for v in backend.logged("velocity") if v.vx} == {backend.top.vx / 2}


def test_connecting_again_forgets_the_heading_the_goals_meant():
    robot, backend = _goals()
    robot.turn(90)
    robot.close()
    robot.connect()
    backend.yaw = 30.0
    robot.turn(90)
    assert [goal.angle for goal, _ in backend.logged("goal")] == pytest.approx([90.0, 90.0])
