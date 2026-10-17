"""The robot's WALK_NN path as the training environment runs it (src/sim/firmware_policy.py): the observation the
network sees, the decoding of what it outputs, and the header the robot gets the same constants from."""
import math
import re
from pathlib import Path

import numpy as np
import pytest

from src.envs.hexapod_mj_env import (ACTION_TAU, BODY_ZM_MM, BaseGait, HexapodMjEnv, analytic_gait_action,
                                     firmware_policy_params)
from src.sim.firmware_policy import FirmwarePolicy
from src.sim.mj_runtime import HexapodSim

HEADER = Path(__file__).resolve().parents[1] / "firmware" / "include" / "policy" / "hexapod_policy.h"
DT = 0.02
ALPHA = math.exp(-DT / ACTION_TAU)
LEVEL = ([0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])


def session(mode="residual_gait", history=1):
    policy = FirmwarePolicy(firmware_policy_params(mode, history, BaseGait(None)))
    policy.reset()
    return policy


def test_the_observation_is_laid_out_as_the_network_expects():
    policy = session()
    policy.set_command(0.2, -0.05, 0.3)
    obs = policy.observe([0.1, -0.2, 0.97], [0.3, -0.1, 0.05], [0.1, 0.05, -0.2], DT)
    assert obs.shape == (57,)
    np.testing.assert_allclose(obs[0:3], [-0.1, 0.2, -0.97], atol=1e-6)  # world down
    np.testing.assert_allclose(obs[3:6], [0.3, -0.1, 0.05], atol=1e-6)
    np.testing.assert_allclose(obs[6:9], [0.1, -0.05, 0.2], atol=1e-6)  # the DMP's pitch and yaw, negated
    np.testing.assert_allclose(obs[9:27], HexapodSim().stand_pose, atol=1e-5)  # standing on the stance
    np.testing.assert_allclose(obs[27:30], [0.0, 1.0, DT], atol=1e-6)  # phase 0, then dt
    np.testing.assert_allclose(obs[30:33], [0.2, -0.05, 0.3], atol=1e-6)
    assert not obs[33:].any()


def test_the_action_is_clipped_and_low_pass_filtered_and_then_observed():
    policy = session()
    base = analytic_gait_action([0.2, 0.0, 0.0])
    policy.observe(*LEVEL, DT)
    policy.act(np.full(24, 3.0), base)
    np.testing.assert_allclose(policy.applied_action(), 1.0 - ALPHA, rtol=1e-5)
    obs = policy.observe(*LEVEL, DT)
    np.testing.assert_allclose(obs[33:], policy.applied_action(), atol=1e-7)
    policy.act(np.full(24, 3.0), base)
    np.testing.assert_allclose(policy.applied_action(), 1.0 - ALPHA ** 2, rtol=1e-5)


def test_the_sixth_residual_gait_channel_raises_the_body():
    policy = session()
    policy.observe(*LEVEL, DT)
    action = np.zeros(24)
    action[5] = 1.0
    policy.act(action, analytic_gait_action([0.0, 0.0, 0.0]))
    assert policy.state()["body_zm"] == pytest.approx(-BODY_ZM_MM * (1.0 - ALPHA), rel=1e-5)


def test_residual_pure_moves_only_the_feet():
    policy = session("residual_pure")
    policy.observe(*LEVEL, DT)
    base = analytic_gait_action([0.0, 0.0, 0.0])
    still = policy.act(np.zeros(18), base)
    other = session("residual_pure")
    other.observe(*LEVEL, DT)
    lifted = np.zeros(18)
    lifted[2] = 1.0  # leg 0, up
    moved = other.act(lifted, base)
    assert np.abs(moved[3:] - still[3:]).max() < 1e-6
    assert np.abs(moved[:3] - still[:3]).max() > 1e-3
    assert other.state()["body_zm"] == policy.state()["body_zm"] == 0.0


def test_the_sensor_history_is_stride_frames_apart_and_starts_full():
    policy = session(history=3)  # HIST_STRIDE = 3
    gyro_z = []
    for k in range(8):
        obs = policy.observe([0.0, 0.0, 1.0], [0.0, 0.0, float(k)], [0.0, 0.0, 0.0], DT)
        gyro_z.append([obs[5], obs[11], obs[17]])
    assert gyro_z[0] == [0.0, 0.0, 0.0]
    assert gyro_z[2] == [2.0, 0.0, 0.0]  # older frames repeat the first until the buffer fills
    assert gyro_z[7] == [7.0, 4.0, 1.0]


def test_sessions_in_one_process_are_independent():
    a, b = session(), session()
    base = analytic_gait_action([0.2, 0.0, 0.0])
    a.observe(*LEVEL, DT)
    b.observe(*LEVEL, DT)
    a.act(np.ones(24), base)
    b.act(np.zeros(24), base)
    assert a.applied_action().any() and not b.applied_action().any()


def _header_floats(name):
    text = HEADER.read_text()
    m = re.search(rf"\b{name}\[\d+\] = \{{([^}}]*)\}}", text) or re.search(rf"\b{name} = (-?[0-9.e+-]+)f?[,;]", text)
    assert m, name
    return [float(v.strip().rstrip("f")) for v in m.group(1).split(",")]


def test_the_robots_header_carries_the_constants_the_environment_trains_with():
    mode = re.search(r"// run: \S+ mode: (\S+)", HEADER.read_text()).group(1)
    history = int(_header_floats("OBS_HISTORY")[0])
    p = firmware_policy_params(mode, history, BaseGait(None))
    expected = {
        "ACT_DIM": [p.action_dim], "HIST_STRIDE": [p.hist_stride], "ACTION_TAU_S": [p.action_tau_s],
        "FOOT_RESIDUAL_MM": [p.foot_residual_mm], "PG_STEP_XY": [p.step_xy_mm], "PG_STEP_ANGLE": [p.step_angle_rad],
        "PG_HEIGHT_MIN": [p.step_height_min_mm], "PG_HEIGHT_MAX": [p.step_height_max_mm],
        "PG_PHASE_RATE_MIN": [p.phase_rate_min], "PG_PHASE_RATE_MAX": [p.phase_rate_max],
        "STEP_DEPTH_MM": [p.step_depth_mm], "BODY_ZM_MM": [p.body_zm_mm], "BODY_RIDE_MM": [p.body_ride_mm],
        "GAIT_DELTA_GAIN": list(p.gait_delta_gain), "TRI_OFFSET": list(p.tri_offset), "BI_OFFSET": list(p.bi_offset),
        "TRI_STAND_FRAC": [p.tri_stand_frac], "BI_STAND_FRAC": [p.bi_stand_frac],
    }
    for name, values in expected.items():
        assert _header_floats(name) == pytest.approx(values, rel=1e-6), name


@pytest.mark.parametrize("option", [{"reflex": True}, {"arc_stance": True}, {"obs_contact": True},
                                    {"self_level": False}])
def test_the_robots_modes_refuse_what_the_robot_does_not_have(option):
    with pytest.raises(ValueError, match="WALK_NN"):
        HexapodMjEnv(control_mode="residual_gait", **option)


# The robot samples its MPU6050's 188 Hz low-passed gyro once per 5 ms tick, so the WALK_NN path observes the rate
# averaged over the 5 ms before, as FirmwareLoop feeds the firmware, not over the whole control period.
def test_the_firmware_path_observes_the_gyro_over_the_last_5_ms():
    env = HexapodMjEnv(control_mode="residual_gait")
    env.reset(seed=0)
    seen = []
    observe = env.firmware.observe
    env.firmware.observe = lambda down, gyro, rpy, dt: (seen.append(np.array(gyro)), observe(down, gyro, rpy, dt))[1]
    for _ in range(5):
        env.step(np.zeros(env.action_space.shape, dtype=np.float32))
    step = env.sim.model.opt.timestep
    assert step == pytest.approx(0.002) and env.dt > 0.005
    rates = [rate for _, rate in env.sim._rates]
    expected = (rates[-1] * 2 + rates[-2] * 2 + rates[-3] * 1) / 5  # 2 ms, 2 ms and the last 1 ms of the third step
    np.testing.assert_allclose(seen[-1], expected, rtol=1e-9, atol=1e-12)
