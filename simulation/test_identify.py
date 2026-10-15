"""System identification (identify.py) on synthetic recordings: the firmware gait walked in HexapodSim at known servo
parameters, actuator lag and IMU noise, and written through the RecordingWriter record.py uses.

The physics loop here is written independently of identify.Replay: each written angle reaches the servos at the first
physics step that starts at or after its write time plus the lag, as the real servo's next PWM frame would.
"""
import json
import sys

import mujoco
import numpy as np
import pytest

import identify
from src.envs.hexapod_mj_env import HexapodMjEnv
from src.platform_shared import message_pb2
from src.robot.firmware_gait import TRI_GATE
from src.sim.domain_rand import Ranges
from src.sim.firmware_loop import FIRMWARE_DT
from src.sim.virtual_robot import STAND, WALK, VirtualRobot
from src.sim.mj_runtime import SERVO_STALL, HexapodSim
from src.sim.recording import RecordingWriter, load

TICK_US = round(FIRMWARE_DT * 1e6)
COMPUTE_US = 900
IMU_EVERY_TICKS = 5  # the firmware polls the IMU every 25 ms
BATCH_TICKS = 5
PRE_SETTLE_S = 1.0  # the robot stood before the recording began
STAND, WALK = 3, 4  # MOTION_STATE positions

TRUTH = dict(stall=0.9, hold=1.1, kp=1.15, noload=1.0, deadband=0.012, lag_ms=8.0,
             gyro_bias=(0.02, -0.01, 0.005), gyro_noise=0.04, rpy_noise_deg=0.8, gravity_noise=0.01)


def joint_commands(stand_s: float, walk_s: float, stick) -> np.ndarray:
    """The firmware's STAND then WALK ticks (deg, as the telemetry carries them), on a level IMU: with no physics in
    the loop the commands are the same on every platform, where feeding back a simulated IMU would let the contacts
    amplify each platform's floating point into a different walk."""
    robot = VirtualRobot()
    robot.set_gait(TRI_GATE)
    robot.set_mode(STAND)
    lx, ly, rx = stick
    angles = []
    for _ in range(round(stand_s / FIRMWARE_DT)):
        robot.tick()
        angles.append(robot.servo_angles())
    robot.set_mode(WALK)
    for _ in range(round(walk_s / FIRMWARE_DT)):
        robot.command(lx=lx, ly=ly, rx=rx)
        robot.tick()
        angles.append(robot.servo_angles())
    return np.array(angles)


def write(path, *, stall, hold, kp, noload, deadband, lag_ms, gyro_bias, gyro_noise, rpy_noise_deg, gravity_noise,
          stand_s, walk_s, stick=(0.0, 0.6, 0.3), seed=0, deactivate_at_s=None):
    rng = np.random.default_rng(seed)
    angles = joint_commands(stand_s, walk_s, stick)
    n = len(angles)
    sim = HexapodSim()
    sim.reset_to_stand()
    sim.set_servo_scale(kp=kp, stall=stall, noload=noload, hold=hold)
    sim.deadband = deadband
    sim.set_joint_targets(np.radians(angles[0]))
    h_us = round(sim.model.opt.timestep * 1e6)
    sim.step_physics(round(PRE_SETTLE_S * 1e6 / h_us))
    base = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "base")

    tick_us = np.arange(n) * TICK_US
    acts_us = tick_us + COMPUTE_US + round(lag_ms * 1000)
    steps = int(np.ceil(tick_us[-1] / h_us)) + 1
    gyro, up = np.empty((steps + 1, 3)), np.empty((steps + 1, 3))
    gyro[0], up[0] = sim.gyro(), sim.data.xmat[base, 6:9]
    for step in range(1, steps + 1):
        written = np.searchsorted(acts_us, (step - 1) * h_us, side="right") - 1
        sim.set_joint_targets(np.radians(angles[max(written, 0)]))
        sim.step_physics(1)
        gyro[step], up[step] = sim.gyro(), sim.data.xmat[base, 6:9]
    times = np.arange(steps + 1) * h_us
    gyro = np.column_stack([np.interp(tick_us, times, gyro[:, c]) for c in range(3)])
    up = np.column_stack([np.interp(tick_us, times, up[:, c]) for c in range(3)])
    rpy = np.column_stack([np.arctan2(up[:, 1], up[:, 2]), -np.arcsin(np.clip(up[:, 0], -1.0, 1.0)), np.zeros(n)])
    gyro = gyro + np.asarray(gyro_bias) + rng.normal(0.0, gyro_noise, (n, 3))
    up = up + rng.normal(0.0, gravity_noise, (n, 3))
    rpy = rpy + np.radians(rng.normal(0.0, rpy_noise_deg, (n, 3)))
    fresh = np.arange(n) % IMU_EVERY_TICKS == 0
    held = np.maximum.accumulate(np.where(fresh, np.arange(n), 0))  # ticks between polls repeat the last sample
    deactivated = n if deactivate_at_s is None else round(deactivate_at_s / FIRMWARE_DT)

    writer = RecordingWriter(path, {"firmwareVersion": "synthetic"})
    for batch_seq, first in enumerate(range(0, n, BATCH_TICKS)):
        message = message_pb2.Message()
        batch = message.telemetry_batch
        batch.batch_seq = batch_seq
        for k in range(first, min(first + BATCH_TICKS, n)):
            tick = batch.ticks.add(seq=k, t_us=1_000_000 + tick_us[k], period_us=TICK_US, compute_us=COMPUTE_US,
                                   imu_fresh=bool(fresh[k]), command_age_ms=0, gait=TRI_GATE)
            tick.mode = 0 if k >= deactivated else (STAND if k * FIRMWARE_DT < stand_s else WALK)
            tick.rpy.extend(rpy[held[k]])
            tick.gyro.extend(gyro[held[k]])
            tick.gravity.extend(up[held[k]])
            tick.angles.extend(angles[k])
            tick.command.extend([0.0] * 8)
        writer.write(message.SerializeToString(), host_rx_us=batch_seq)
    writer.close()


@pytest.fixture(scope="module")
def walk(tmp_path_factory):
    path = tmp_path_factory.mktemp("identify") / "walk.hexrec"
    write(path, **TRUTH, stand_s=2.0, walk_s=10.0)
    return path, identify.identify(load(path))


def _within_four_sigma(estimate, truth):
    assert estimate.identified, estimate.note
    assert abs(estimate.value - truth) <= 4 * estimate.std, (estimate, truth)


def test_a_walk_recovers_the_servo_parameters_and_the_lag(walk):
    _, found = walk
    for name in ("stall", "deadband", "lag_ms"):
        _within_four_sigma(found[name], TRUTH[name])
    # The stated sigmas must be tight enough to matter: a value pinned no better than today's randomisation range
    # would have been refused.
    assert found["stall"].std < 0.05
    assert found["lag_ms"].std < 5.0


def test_what_the_walk_cannot_pin_is_refused_with_the_reason(walk):
    _, found = walk
    # The gait never drives a joint near its no-load speed, so the speed cap leaves no trace in the IMU.
    assert not found["noload"].identified
    assert "cannot pin it" in found["noload"].note
    for name in ("hold", "kp"):
        estimate = found[name]
        if estimate.identified:
            _within_four_sigma(estimate, TRUTH[name])
        else:
            assert "cannot pin it" in estimate.note


def test_the_still_start_gives_the_gyro_bias_and_the_imu_noise(walk):
    _, found = walk
    for axis, bias in zip("xyz", TRUTH["gyro_bias"]):
        _within_four_sigma(found[f"gyro_bias_{axis}"], bias)
    _within_four_sigma(found["gyro_noise_std"], TRUTH["gyro_noise"])
    _within_four_sigma(found["rpy_noise_deg"], TRUTH["rpy_noise_deg"])
    _within_four_sigma(found["gravity_noise_std"], TRUTH["gravity_noise"])
    assert not found["rpy_bias"].identified
    assert found["sensor_to_servo_ms_p50"].value == pytest.approx(COMPUTE_US / 1000)


def test_the_centre_narrows_the_randomisation_around_what_was_found(walk, tmp_path):
    path, found = walk
    centre = tmp_path / "centre.json"
    centre.write_text(json.dumps(identify.centre(found, str(path))))
    ranges = Ranges.identified(centre)
    default = Ranges()
    assert ranges.stall[0] < TRUTH["stall"] < ranges.stall[1]
    assert np.ptp(ranges.stall) < np.ptp(default.stall)
    assert ranges.deadband[0] < TRUTH["deadband"] < ranges.deadband[1]
    assert ranges.noload == default.noload  # refused, so it keeps today's range
    np.testing.assert_allclose(ranges.gyro_bias_mean, TRUTH["gyro_bias"], atol=0.01)
    assert ranges.gyro_bias_std == default.gyro_bias_std
    assert ranges.gyro_noise_std == pytest.approx(TRUTH["gyro_noise"], rel=0.1)
    # 0.9 ms from sensor to servo write and 8 ms of lag lie inside one 20 ms control step, with room for the three
    # sigmas the range adds: at 13 ms that sum sat within a millisecond of the step and each platform's floating point
    # put it on its own side.
    assert ranges.action_latency_steps == (0, 2)

    env = HexapodMjEnv(randomize=True, seed=0, dr_ranges=ranges)
    for _ in range(5):
        env.reset()
        assert ranges.stall[0] <= env.sim.stall / SERVO_STALL <= ranges.stall[1]
        assert ranges.deadband[0] <= env.sim.deadband <= ranges.deadband[1]
        assert ranges.action_latency_steps[0] <= env.dr.action_latency_steps < ranges.action_latency_steps[1]


def test_the_default_ranges_are_what_the_randomisation_drew_before():
    default = Ranges()
    assert default.stall == (0.80, 1.20) and default.hold == (0.80, 1.20)
    assert default.kp == (0.65, 1.35) and default.noload == (0.85, 1.15)
    assert default.deadband == (0.005, 0.026)
    assert default.action_latency_steps == (1, 4)
    assert (default.gyro_bias_std, default.gyro_noise_std, default.rpy_noise_deg) == (0.05, 0.05, 1.5)


# A robot that never moves says nothing about its servos: the tool must say so instead of reporting the search's
# starting point, and must still give the IMU noise.
def test_a_still_recording_identifies_the_imu_but_not_the_servos(tmp_path, monkeypatch, capsys):
    path = tmp_path / "still.hexrec"
    write(path, **TRUTH, stand_s=3.0, walk_s=0.0)
    monkeypatch.setattr(sys, "argv", ["identify.py", str(path)])
    identify.main()
    written = json.loads(path.with_suffix(".sysid.json").read_text())
    assert {"stall", "hold", "kp", "noload", "deadband", "lag_ms"} <= written["not_identifiable"].keys()
    assert written["estimates"]["gyro_noise_std"]["value"] == pytest.approx(TRUTH["gyro_noise"], rel=0.1)
    out = capsys.readouterr().out
    assert "stall" in out and "not identifiable" in out
    assert Ranges.identified(path.with_suffix(".sysid.json")).stall == Ranges().stall


@pytest.mark.parametrize("change, reason", [
    (dict(stand_s=0.0), "does not begin with the robot standing still"),
    (dict(deactivate_at_s=0.5), "does not begin with the robot standing still"),
    (dict(deactivate_at_s=1.5), "never exercised"),
])
def test_a_recording_the_replay_cannot_start_from_is_refused(tmp_path, change, reason):
    path = tmp_path / "refused.hexrec"
    write(path, **{**TRUTH, "stand_s": 2.0, "walk_s": 1.0, **change})
    found = identify.identify(load(path))
    for name in ("stall", "hold", "kp", "noload", "deadband", "lag_ms"):
        assert not found[name].identified and reason in found[name].note
