"""The MuJoCo backend, headless and stepped only inside sleep(), alone and behind the Robot facade."""
import numpy as np
import pytest

from src.hexapod import Gait, Mode, Robot, RobotError, Velocity, speed_model
from src.hexapod.backends.base import ZERO
from src.hexapod.backends.sim import SimBackend

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


# Distances are open-loop on the steady-state speed model, held RAMP_LAG_S longer for the command ramp. The stroke is
# held at any speed, so a forward move ends about 0.035 m short of that for TRI_GATE, which a lead cannot correct as
# it depends on where the stop catches the gait: from a fresh stand TRI_GATE walks 0.066/0.1, 0.264/0.3 and
# 0.461/0.5 m forward and 0.099/0.1 and 0.200/0.2 m sideways, so a forward move shorter than 0.3 m misses by more
# than 15 %. The turn closes on the settled yaw.
def test_the_facade_walks_distances_and_turns_angles():
    backend = SimBackend()
    with Robot(backend=backend) as robot:
        robot.stand()
        start = _left_forward(backend)
        robot.move_forward(0.3)
        left, forward = _left_forward(backend) - start
        assert forward == pytest.approx(0.3, rel=0.15) and abs(left) < 0.045
        start = _left_forward(backend)
        robot.move_sideways(-0.2)
        left, forward = _left_forward(backend) - start
        assert left == pytest.approx(-0.2, rel=0.15) and abs(forward) < 0.045
        yaw = robot.state().yaw
        robot.rotate(90)
        assert (robot.state().yaw - yaw + 180) % 360 - 180 == pytest.approx(90, abs=4)


# On the firmware's code BI_GATE moves 0.2 m sideways to 1.010 of it either way with the ramp term and 1.04 without it
# (measured on Windows); test_robot_motions pins the term itself against the ramp's model.
@pytest.mark.parametrize("metres", [0.2, -0.2])
def test_a_timed_move_covers_its_distance_despite_the_command_ramp(metres):
    backend = SimBackend()
    with Robot(backend=backend) as robot:
        robot.stand()
        robot.set_gait(Gait.BI_GATE)
        start = _left_forward(backend)
        robot.move_sideways(metres)
        left, forward = _left_forward(backend) - start
    assert left == pytest.approx(metres, rel=0.15) and abs(forward) < 0.03


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
