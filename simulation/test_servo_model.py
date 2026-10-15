"""The simulated MG92B: it drives with its stall torque but holds against a load well above it.

A powered hobby servo cannot be back-driven by a load its motor could not lift, because the load has
to turn the gear train backwards; the model nets stall torque while driving and the larger holding
torque while being back-driven or standing still.
"""
import numpy as np
import pytest

from src.envs.hexapod_mj_env import HexapodMjEnv
from src.sim.mj_runtime import SERVO_HOLD, SERVO_STALL, HexapodSim

FEMUR_0 = 1  # JOINT_NAMES index of leg 0's femur
PHYSICS_HZ = 500
LOADS_BETWEEN_STALL_AND_HOLD = [1.05 * SERVO_STALL, 0.5 * (SERVO_STALL + SERVO_HOLD), 0.95 * SERVO_HOLD]
# The weak and strong corners of the env's servo randomization (stall and hold scales, each 0.8-1.2).
DR_CORNERS = [(1.0, 1.0), (0.8, 0.8), (0.8, 1.2), (1.2, 0.8), (1.2, 1.2)]


def _floating_sim():
    """The robot in zero gravity, far above the ground, so a joint sees only the load a test applies."""
    sim = HexapodSim()
    sim.model.opt.gravity[:] = 0.0
    sim.reset_to_stand()
    sim.data.qpos[2] += 1.0
    return sim


def _femur_deg(sim):
    return float(np.rad2deg(sim.data.qpos[sim.qpos_adr[FEMUR_0]]))


def _run_under_load(sim, load_nm, seconds):
    """Steps physics with a constant external torque on the femur; returns its angle trace in degrees."""
    trace = []
    for _ in range(int(seconds * PHYSICS_HZ)):
        sim.data.qfrc_applied[sim.qvel_adr[FEMUR_0]] = load_nm
        sim.step_physics(1)
        trace.append(_femur_deg(sim))
    return np.asarray(trace)


@pytest.mark.parametrize("load", LOADS_BETWEEN_STALL_AND_HOLD)
def test_a_static_joint_holds_a_load_above_stall_but_below_holding_torque(load):
    sim = _floating_sim()
    start = _femur_deg(sim)
    trace = _run_under_load(sim, load, seconds=2.0)
    assert np.abs(trace - start).max() < 0.5
    assert np.ptp(trace[-PHYSICS_HZ:]) < 0.02, "the joint keeps slipping under a load it should hold"


def test_a_load_above_holding_torque_back_drives_the_joint():
    sim = _floating_sim()
    start = _femur_deg(sim)
    trace = _run_under_load(sim, 1.5 * SERVO_HOLD, seconds=0.3)
    assert abs(trace[-1] - start) > 10.0


@pytest.mark.parametrize("load", LOADS_BETWEEN_STALL_AND_HOLD)
@pytest.mark.parametrize("direction", [1.0, -1.0])
def test_the_servo_cannot_drive_against_a_load_above_stall(direction, load):
    """Commanding a move against a load between stall and hold: it neither lifts the load nor gives way."""
    sim = _floating_sim()
    start = _femur_deg(sim)
    target = sim.joint_target.copy()
    target[FEMUR_0] += direction * np.deg2rad(20.0)
    sim.set_joint_targets(target)
    trace = _run_under_load(sim, -direction * load, seconds=2.0)
    progress = direction * (trace - start)
    assert progress.min() > -0.5, "the load back-drove a servo that should hold it"
    assert progress.max() < 0.5, "the servo lifted a load above its stall torque"


def test_the_same_servo_moves_freely_against_a_load_below_stall():
    sim = _floating_sim()
    start = _femur_deg(sim)
    target = sim.joint_target.copy()
    target[FEMUR_0] += np.deg2rad(20.0)
    sim.set_joint_targets(target)
    trace = _run_under_load(sim, -0.5 * SERVO_STALL, seconds=0.5)
    assert trace[-1] - start == pytest.approx(20.0, abs=0.1)


def test_switching_the_servo_off_also_removes_its_holding_torque():
    """SimBackend relies on stall=0 for limp legs; the holding torque must not survive that."""
    sim = _floating_sim()
    sim.set_servo_scale(stall=0.0)
    start = _femur_deg(sim)
    trace = _run_under_load(sim, 0.05, seconds=0.3)
    assert abs(trace[-1] - start) > 10.0


def test_the_hold_scale_randomizes_only_the_holding_torque():
    weak_hold = _floating_sim()
    weak_hold.set_servo_scale(hold=0.5)
    start = _femur_deg(weak_hold)
    trace = _run_under_load(weak_hold, 0.8 * SERVO_HOLD, seconds=0.3)
    assert abs(trace[-1] - start) > 10.0

    driving = _floating_sim()
    driving.set_servo_scale(hold=0.5)
    target = driving.joint_target.copy()
    target[FEMUR_0] += np.deg2rad(20.0)
    driving.set_joint_targets(target)
    trace = _run_under_load(driving, -0.5 * SERVO_STALL, seconds=0.5)
    assert trace[-1] - _femur_deg(_floating_sim()) == pytest.approx(20.0, abs=0.1)


def test_domain_randomization_spreads_the_holding_ratio_over_the_unverified_range():
    """k_hold is a placeholder estimated at 2-3; training must not overfit the single value 2.5."""
    env = HexapodMjEnv("residual_gait", randomize=True, seed=0)
    ratios, motor_caps = [], []
    for seed in range(20):
        env.reset(seed=seed)
        ratios.append(env.sim.k_hold)
        motor_caps.append(env.sim.motor_torque_cap)
    assert min(ratios) >= 2.0 and max(ratios) <= 3.0
    assert np.ptp(ratios) > 0.5
    assert max(motor_caps) <= env.sim.model.actuator_ctrlrange[:, 1].min(), "the XML ctrlrange would clip the motor"


@pytest.mark.parametrize("stall, hold", DR_CORNERS)
@pytest.mark.parametrize("step_deg", [20.0, -20.0])
def test_a_standing_femur_step_arrives_monotonically_without_overshoot(step_deg, stall, hold):
    """The real servo settles on a step with no visible overshoot or ringing, at any randomized strength."""
    sim = HexapodSim()
    sim.set_servo_scale(stall=stall, hold=hold)
    sim.reset_to_stand()
    sim.step_physics(50 * sim.frame_skip)
    target = sim.joint_target.copy()
    target[FEMUR_0] += np.deg2rad(step_deg)
    sim.set_joint_targets(target)
    trace = []
    for _ in range(8 * PHYSICS_HZ):  # one leg lifting the body at 0.8x stall takes ~4.1 s for 20 deg
        sim.step_physics(1)
        trace.append(_femur_deg(sim))
    past_target = np.sign(step_deg) * (np.asarray(trace) - np.rad2deg(target[FEMUR_0]))
    assert past_target.max() < 0.011
    assert np.diff(past_target).min() > -1e-6, "the joint moved away from its target: ringing"
    assert abs(past_target[-1]) < 0.05
    # The soft gear friction lets the joint creep its last hundredths of a degree onto the target.
    assert np.ptp(past_target[-PHYSICS_HZ // 2:]) < 0.005
