"""The MuJoCo backend, headless and stepped only inside sleep(), alone and behind the Robot facade."""
import math

import numpy as np
import pytest

from src.hexapod import Gait, Mode, Robot, RobotError, Velocity, speed_model
from src.hexapod.backends.base import ZERO, GoalState
from src.hexapod.backends.sim import SimBackend
from src.sim import recording
from src.sim.firmware_loop import FIRMWARE_DT
from src.sim.virtual_robot import firmware_version

STAND_HEIGHT_M = 0.072  # base height of the default stance (resting feet 66 mm below the body origin)


@pytest.fixture
def sim():
    backend = SimBackend()
    backend.connect()
    yield backend
    backend.close()


def _left_forward(backend):
    """World position in the start pose's REP-103 frame: the robot starts facing world +Y, so left is world -X."""
    x, y = backend.sim.data.qpos[:2]
    return np.array([-x, y])


def test_a_standing_robot_is_level_at_its_stance_height(sim):
    sim.set_mode(Mode.STAND)
    sim.sleep(1.0)
    assert sim.sim.base_height() == pytest.approx(STAND_HEIGHT_M, abs=0.003)
    assert sim.sim.body_tilt_deg() < 1.0 and abs(sim.state().yaw) < 1.0
    assert sim.state().t == pytest.approx(1.0)


def test_a_standing_robot_on_miscalibrated_servos_holds_still():
    # Servos at 2.28 PWM counts per degree against the factory 2.0 tilt the stand, which STAND's self-levelling must
    # correct and then hold. Fed the IMU's angles in degrees where the firmware reads radians, it swung 3.5 deg.
    backend = SimBackend(servo_scale=2.28)
    backend.connect()
    backend.set_mode(Mode.STAND)
    backend.sleep(2.0)
    rolls = []
    for _ in range(60):
        backend.sleep(0.05)
        rolls.append(backend.state().roll)
    backend.close()
    assert np.ptp(rolls) < 0.5


# The physical check of the axis mapping: REP-103 forward, left and counter-clockwise, in the world.
def test_forward_left_and_counter_clockwise_follow_rep103(sim):
    sim.set_mode(Mode.STAND)
    sim.sleep(1.5)
    sim.set_mode(Mode.WALK)
    start = _left_forward(sim)
    sim.set_velocity(Velocity(0.1, 0, 0))
    sim.sleep(2.0)
    left, forward = _left_forward(sim) - start
    # The firmware's gait drifts across by 4-5 % of the distance from a stand, levelling or not, and by up to 0.033 m
    # here on Linux, where the firmware's floating point differs slightly (measured).
    assert forward > 0.15 and abs(left) < 0.045
    middle = _left_forward(sim)
    sim.set_velocity(Velocity(0, 0.1, 0))
    sim.sleep(2.0)
    left, forward = _left_forward(sim) - middle
    assert left > 0.15 and abs(forward) < 0.045
    yaw = sim.state().yaw
    sim.set_velocity(Velocity(0, 0, 0.5))
    sim.sleep(2.0)
    assert sim.state().yaw - yaw > 40


def test_a_velocity_moves_the_robot_only_when_it_walks(sim):
    sim.set_mode(Mode.STAND)
    start = _left_forward(sim)
    assert sim.set_velocity(Velocity(0.1, 0, 0)).vx == pytest.approx(0.1)
    sim.sleep(2.0)
    assert np.linalg.norm(_left_forward(sim) - start) < 0.005


def test_a_deactivated_robot_goes_limp_and_stands_back_up(sim):
    sim.sleep(1.0)  # booted DEACTIVATED
    assert sim.sim.base_height() < 0.03
    sim.set_mode(Mode.STAND)
    sim.sleep(1.5)
    assert sim.sim.base_height() == pytest.approx(STAND_HEIGHT_M, abs=0.003)


def test_the_simulation_refuses_modes_it_does_not_run(sim):
    with pytest.raises(RobotError):
        sim.set_mode(Mode.ANIMATE)


# Clamped to the peak in each direction, which max_velocity's both-ways top may undercut. The classical gaits now walk
# the same peak both ways, so TUNED (its rightward peak, 0.088 m/s, is 9 % above its leftward one, measured) carries
# the check.
def test_a_velocity_beyond_the_gait_is_clamped_to_its_peak_that_way(sim):
    sim.set_gait(Gait.TUNED)
    axes = speed_model._axes(Gait.TUNED)
    clamped = sim.set_velocity(Velocity(10, -10, 100))
    assert clamped == Velocity(axes["vx"].values.max(), axes["vy"].values.min(), axes["yaw_rate"].values.max())
    assert -clamped.vy > sim.max_velocity().vy
    assert sim.set_velocity(ZERO) == ZERO


def _pose(backend):
    """The true pose: the position in the world's REP-103 frame (forward is world +Y, where the robot starts facing,
    and left is world -X) and the yaw in degrees, counter-clockwise."""
    x, y = backend.sim.data.qpos[:2]
    return np.array([y, -x]), math.degrees(backend.sim.base_yaw())


def _moved(start, end):
    """Where the pose `end` is in the frame of the pose `start`: forward and left in m, and the yaw turned in degrees
    within +-180."""
    (p0, yaw0), (p1, yaw1) = start, end
    c, s = math.cos(math.radians(yaw0)), math.sin(math.radians(yaw0))
    d = p1 - p0
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]]), (yaw1 - yaw0 + 180) % 360 - 180


@pytest.fixture
def standing():
    backend = SimBackend()
    with Robot(backend=backend) as robot:
        robot.stand()
        yield robot, backend


# The tolerances the design binds on MuJoCo's true pose (docs/superpowers/specs/2026-10-17-goal-moves-design.md):
# the firmware closes each goal on its own odometry, so these measure how far the odometry is from the truth.
def test_a_goal_walks_a_metre_to_within_6_cm(standing):
    robot, backend = standing
    start = _pose(backend)
    status = robot.walk(1.0)
    position, _ = _moved(start, _pose(backend))
    assert status.state is GoalState.REACHED
    # MuJoCo's feet slip: TRI walks 4.2-4.8 % beyond its odometry, a simulation contact effect that the user chose not
    # to calibrate out unless the floor runs need it (measured 2026-10-17), so the simulation's tolerance is 6 cm.
    assert np.linalg.norm(position - (1.0, 0.0)) < 0.06


def test_a_goal_turns_90_degrees_to_within_3(standing):
    robot, backend = standing
    start = _pose(backend)
    robot.turn(90)
    _, yaw = _moved(start, _pose(backend))
    assert yaw == pytest.approx(90, abs=3)


# The firmware's ARC_REACHED_M (firmware/include/goal_controller.h): how far a reached arc ends from its end point.
ARC_REACHED_M = 0.03
# How far MuJoCo's true pose ends from the odometry's, per metre of the odometry's path: the feet slip, by 4.2-5.0 %
# on TRI's arc of 0.5 m through 90 deg and 4.2-5.6 % on BI's (measured 2026-10-30 over 8 clock phases), held to 7 %.
SLIP_PER_M = 0.07


# The arc is held in two parts: the firmware ends it on its end point in its own odometry, and the odometry's slip
# against the true pose grows with the path. A single bound on the true pose mixed them, and sat 4 mm from TRI's worst.
def test_a_goal_arc_ends_on_its_end_point_and_slips_with_its_path(standing):
    robot, backend = standing
    robot.reset_odometry()
    robot.sleep(0.1)
    start = _pose(backend)
    robot.arc(0.5, 90)
    position, yaw = _moved(start, _pose(backend))
    odometry = robot.state().odometry
    estimate = np.array([odometry.x, odometry.y])
    assert np.linalg.norm(estimate - (0.5, 0.5)) <= ARC_REACHED_M
    assert np.linalg.norm(position - estimate) <= SLIP_PER_M * odometry.distance
    assert yaw == pytest.approx(90, abs=5)


# Each turn takes up the heading its walk drifted (Robot.turn). Without, the per-goal tolerances add up, as each walk
# ends up to 1.6 deg off its heading and each turn about 0.9 deg over, to 4.1-6.2 deg (measured 2026-10-17).
def test_a_goal_square_closes_within_10_cm_and_5_degrees(standing):
    robot, backend = standing
    start = _pose(backend)
    for _ in range(4):
        robot.walk(1.0)
        robot.turn(90)
    position, yaw = _moved(start, _pose(backend))
    assert np.linalg.norm(position) < 0.10 and abs(yaw) < 5


# BI is bound by the walk and the arc, which it meets (measured 2026-10-17 over 8 clock phases: walk <= 3.6 cm, arc
# <= 3.9 cm and <= 0.9 deg). Its turn ends 2.6-3.2 deg short, at the edge of 3 deg, and its square does not close
# (0.8-11.6 cm, up to 9.8 deg), as each turn's corrective pass starts with a yaw of about 2 deg that the feet do not
# command and the fused heading misses: neither is bound.
@pytest.fixture
def standing_bi():
    backend = SimBackend()
    with Robot(backend=backend, gait=Gait.BI_GATE) as robot:
        robot.stand()
        yield robot, backend


def test_a_bi_goal_walks_a_metre_to_within_6_cm(standing_bi):
    test_a_goal_walks_a_metre_to_within_6_cm(standing_bi)


def test_a_bi_goal_arc_ends_on_its_end_point_and_slips_with_its_path(standing_bi):
    test_a_goal_arc_ends_on_its_end_point_and_slips_with_its_path(standing_bi)


# The fused heading integrates the DMP's yaw between packets, into which the walk's yaw rocking does not alias: a 1 m
# walk ends within 0.21 deg of the true yaw, where a gyro sample held over the 25 ms read period ended 0.05-1.5 deg
# (TRI) and 0.24-1.7 deg (BI) off (measured 2026-10-17 over 8 clock phases).
@pytest.mark.parametrize("gait", [Gait.TRI_GATE, Gait.BI_GATE])
def test_a_walk_s_fused_heading_follows_the_true_yaw(gait):
    backend = SimBackend()
    with Robot(backend=backend, gait=gait) as robot:
        robot.stand()
        start, heading = _pose(backend), robot.state().odometry.yaw
        robot.walk(1.0)
        _, yaw = _moved(start, _pose(backend))
        assert (robot.state().odometry.yaw - heading + 180) % 360 - 180 == pytest.approx(yaw, abs=0.4)


# The firmware zeroes the odometry on its next tick: a turn read before it would aim from the old heading (a 90 deg turn
# after a 60 deg one turned 32.5).
def test_a_turn_right_after_an_odometry_reset_turns_its_angle(standing):
    robot, backend = standing
    robot.turn(60)
    robot.reset_odometry()
    assert robot.state().odometry.yaw == pytest.approx(0, abs=0.05)
    start = _pose(backend)
    robot.turn(90)
    _, turned = _moved(start, _pose(backend))
    assert turned == pytest.approx(90, abs=3)


# Ruling R19: the pose is a running goal's origin, so a reset during the goal is refused and resets nothing; the
# refusal raises out of the progress callback, which cancels the goal.
def test_an_odometry_reset_during_a_goal_is_refused(standing):
    robot, backend = standing

    def reset_midway(status):
        if status.state is GoalState.RUNNING and status.progress > 0.3:
            robot.reset_odometry()

    with pytest.raises(RobotError, match="409"):
        robot.walk(1.0, on_progress=reset_midway)
    robot.sleep(0.05)
    assert robot.state().odometry.x > 0.25
    assert backend.goal_status().state is GoalState.CANCELLED


# REP-103 at the facade: sideways positive is left, where the firmware's SIDEWAYS is positive right.
def test_a_sideways_goal_walks_left_when_positive(standing):
    robot, backend = standing
    start = _pose(backend)
    robot.walk(-0.2, sideways=True)
    position, _ = _moved(start, _pose(backend))
    assert np.linalg.norm(position - (0.0, -0.2)) < 0.03
    robot.walk(0.2, sideways=True)
    position, _ = _moved(start, _pose(backend))
    assert np.linalg.norm(position) < 0.03


# The body vibrates at the physics rate, so one gyro sample per 5 ms tick aliases it: WAVE's integral read 91.2 deg for
# a true 83.0 (measured). The firmware is fed the rate averaged over the tick, as the MPU6050's low-pass does.
def test_the_gyro_fed_to_the_firmware_integrates_to_the_true_yaw():
    backend = SimBackend()
    with Robot(backend=backend, gait=Gait.WAVE) as robot:
        robot.stand()
        fed = []
        set_imu = backend._loop.robot.set_imu
        backend._loop.robot.set_imu = lambda rpy, up, gyro: (fed.append(gyro[2]), set_imu(rpy, up, gyro))
        start = _pose(backend)
        robot.turn(90)
        _, turned = _moved(start, _pose(backend))
    assert math.degrees(sum(fed) * FIRMWARE_DT) == pytest.approx(turned, rel=0.005)


def _walk_and_turn():
    backend = SimBackend()
    with Robot(backend=backend) as robot:
        robot.stand()
        robot.walk(0.3)
        robot.turn(45)
        return backend.sim.data.qpos.copy(), robot.state().odometry


# Goals refuse TUNED, so its moves are timed from the PC (motions._open_loop). From a settled stand they land as the
# README states: forward 8 % short at 0.3 m and 5.5 % at 0.5 m, sideways within 4 % (measured 2026-10-17 over 6 clock
# phases: 7.7-8.6 % and 5.1-6.0 % short forward, 1.0-2.2 % short sideways, at most 3.2 cm across).
@pytest.mark.parametrize("sideways, metres, covered, within", [
    (False, 0.3, 0.92, 0.015), (False, 0.5, 0.945, 0.015), (True, 0.3, 1.0, 0.04), (True, -0.3, 1.0, 0.04)])
def test_a_timed_tuned_move_from_a_settled_stand_lands_as_documented(sideways, metres, covered, within):
    backend = SimBackend()
    with Robot(backend=backend, gait=Gait.TUNED) as robot:
        robot.stand()
        robot.sleep(1.0)
        start = _pose(backend)
        (robot.move_sideways if sideways else robot.move_forward)(metres)
        (forward, left), _ = _moved(start, _pose(backend))
    along, across = (left, forward) if sideways else (forward, left)
    assert along / metres == pytest.approx(covered, abs=within) and abs(across) < 0.04


# The firmware's clock starts over with each robot, so a run does not depend on the runs before it.
def test_the_same_goals_run_alike_twice_in_one_process():
    (pose, odometry), (again, odometry_again) = _walk_and_turn(), _walk_and_turn()
    assert np.array_equal(pose, again) and odometry == odometry_again


# The odometry the state carries is the firmware's, in REP-103, and follows the true pose up to MuJoCo's slip: a short
# TRI walk goes 4-7 % beyond its odometry (measured 2026-10-17 over 12 clock phases: 1.3-2.1 cm on 0.3 m).
def test_the_state_carries_the_odometry_in_rep103(standing):
    robot, backend = standing
    robot.reset_odometry()
    robot.sleep(0.1)
    start = _pose(backend)
    robot.walk(0.3)
    robot.turn(45)
    position, yaw = _moved(start, _pose(backend))
    odometry = robot.state().odometry
    assert (odometry.x, odometry.y) == pytest.approx(tuple(position), abs=0.03)
    assert odometry.yaw == pytest.approx(yaw, abs=2)


# Leaving sits the robot down before cutting the servos, so it drops a few millimetres instead of from full height.
def test_leaving_sits_the_robot_down_before_switching_it_off():
    backend = SimBackend()
    with Robot(backend=backend) as robot:
        robot.stand()
        robot.sleep(1.0)
    seated = backend.sim.base_height()
    assert backend.state().mode is Mode.DEACTIVATED
    backend.sleep(1.0)
    assert seated < STAND_HEIGHT_M - 0.04
    assert seated - backend.sim.base_height() < 0.005


def test_a_recording_is_the_firmwares_own_telemetry_tick_by_tick(tmp_path):
    backend = SimBackend()
    with Robot(backend=backend) as robot:
        robot.stand()
        robot.sleep(0.5)
        with robot.recording(tmp_path / "r.hexrec"):
            robot.sleep(0.2)
            commanded = robot.set_velocity(0.1, 0.0, 0.0)
            robot.sleep(1.5)
            robot.stop()
            robot.sleep(0.3)
    rec = recording.load(tmp_path / "r.hexrec")
    t = rec.ticks
    assert rec.features["firmwareVersion"] == firmware_version() and rec.features["backend"] == "simulation"
    assert np.all(np.diff(t["seq"]) == 1) and np.all(t["t_us"] == 5000 * t["seq"]) and np.all(t["period_us"] == 5000)
    assert len(t["seq"]) == 400  # 2 s of ticks, in whole batches of five
    assert t["mode"][0] == Mode.STAND and Mode.WALK in t["mode"]
    walking = t["command"][:, 1] != 0.0
    assert t["command"][walking, 1] == pytest.approx(speed_model.sticks_for(Gait.TRI_GATE, commanded).ly)
    assert t["odom_distance"][-1] - t["odom_distance"][0] == pytest.approx(0.15, rel=0.15)
    assert np.count_nonzero(t["imu_fresh"]) == pytest.approx(len(t["seq"]) / 5, abs=2)  # read every 25 ms


def test_a_joint_moves_its_servo_in_idle_and_nowhere_else(sim):
    femur = sim.sim.qpos_adr[1]
    sim.set_mode(Mode.STAND)
    sim.sleep(1.0)
    standing = math.degrees(sim.sim.data.qpos[femur])
    sim.set_joint(1, 60.0)  # STAND drives the joints itself and ignores it
    sim.sleep(0.5)
    assert math.degrees(sim.sim.data.qpos[femur]) == pytest.approx(standing, abs=1.0)
    sim.set_mode(Mode.IDLE)
    sim.sleep(0.2)
    sim.set_joint(1, 60.0)  # lifts leg 0's foot off the floor
    sim.sleep(0.5)
    assert math.degrees(sim.sim.data.qpos[femur]) == pytest.approx(60.0, abs=2.0)
    with pytest.raises(RobotError, match="no servo 18"):
        sim.set_joint(18, 0.0)


class CtrlCOnStop:
    """A simulated backend on which the next stop raises KeyboardInterrupt, as a second Ctrl-C does when it lands
    while the facade stops the robot on its way out of the first."""

    def __init__(self, backend: SimBackend):
        self._backend, self.armed = backend, False

    def __getattr__(self, name):
        return getattr(self._backend, name)

    def set_velocity(self, velocity):
        if self.armed and velocity == ZERO:
            self.armed = False
            raise KeyboardInterrupt
        return self._backend.set_velocity(velocity)


def test_a_second_ctrl_c_while_leaving_still_switches_the_robot_off():
    backend = SimBackend()
    interrupting = CtrlCOnStop(backend)
    with pytest.raises(KeyboardInterrupt):
        with Robot(backend=interrupting) as robot:
            robot.stand()
            robot.set_velocity(0.05, 0.0, 0.0)
            robot.sleep(0.5)
            interrupting.armed = True
            raise KeyboardInterrupt
    assert backend.fresh_mode() is Mode.DEACTIVATED


def test_a_pwm_outside_the_boards_counts_is_refused_before_it_is_sent(sim):
    robot = Robot(backend=sim)
    sim.set_mode(Mode.IDLE)
    sim.sleep(0.1)
    written = sim._servos.pwm.copy()
    for wrong in (-1, 4096, 70000):
        with pytest.raises(ValueError, match="0-4095"):
            robot.set_servo_pwm(1, wrong)
    assert np.array_equal(sim._servos.pwm, written)
    robot.set_servo_pwm(1, 4095)
    assert sim._servos.pwm[1] == 4095
