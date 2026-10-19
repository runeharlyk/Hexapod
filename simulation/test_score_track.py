"""score_track.py on a synthetic course: sticks a script would send, recorded as the robot's telemetry, a floor path a
robot with known errors would walk for them, and a camera's track of that path on a clock of its own."""
import math

import numpy as np
import pytest

import score_track
from score_track import Telemetry, Track
from src.hexapod import Gait, Mode
from src.hexapod.speed_model import Sticks, velocity_of
from src.platform_shared import message_pb2 as pb
from src.robot.firmware_gait import GAIT_COMMAND_TAU_S
from src.sim import recording
from src.sim.recording import RecordingWriter

TICK_S = 0.005
ROBOT_START_S = 1.0  # the robot's clock at the first tick
START = (0.4, 0.5, 0.3)  # x, y, yaw on the floor
TRACK_FPS = 30.0
TRACK_NOISE_M, TRACK_NOISE_RAD = 0.001, np.radians(0.3)
# (mode, seconds, lx, ly, rx): a turn in place to start with, then forward, a turn back and a strafe to the right.
# A counter-clockwise turn, a forward walk, a clockwise turn and a strafe to the right.
COURSE = [(Mode.STAND, 2.0, 0.0, 0.0, 0.0), (Mode.WALK, 3.0, 0.0, 0.0, -0.4), (Mode.STAND, 1.0, 0.0, 0.0, 0.0),
          (Mode.WALK, 4.0, 0.0, 0.5, 0.0), (Mode.STAND, 1.0, 0.0, 0.0, 0.0), (Mode.WALK, 2.0, 0.0, 0.0, 0.3),
          (Mode.STAND, 1.0, 0.0, 0.0, 0.0), (Mode.WALK, 3.0, 0.5, 0.0, 0.0), (Mode.STAND, 2.0, 0.0, 0.0, 0.0)]


def course_ticks():
    """Per tick: the mode and the sticks."""
    modes, sticks = [], []
    for mode, seconds, lx, ly, rx in COURSE:
        n = round(seconds / TICK_S)
        modes += [mode] * n
        sticks += [(lx, ly, rx)] * n
    return np.array(modes), np.array(sticks)


def walk(modes, sticks, translation_scale: float, turn_scale: float, start) -> np.ndarray:
    """x, y, yaw per tick of a robot that walks each stick's modelled velocity, eased in and out by the firmware's
    command ramp, times translation_scale and turn_scale."""
    alpha = 1 - math.exp(-TICK_S / GAIT_COMMAND_TAU_S)
    velocity = np.zeros(3)  # forward, left, counter-clockwise
    poses = np.zeros((len(modes), 3))
    poses[0] = start
    for k in range(1, len(modes)):
        target = np.zeros(3)
        if modes[k - 1] == Mode.WALK:
            v = velocity_of(Gait.TRI_GATE, Sticks(*sticks[k - 1]))
            target = np.array([v.vx * translation_scale, v.vy * translation_scale, v.yaw_rate * turn_scale])
        velocity += alpha * (target - velocity)
        x, y, yaw = poses[k - 1]
        right, forward = -velocity[1], velocity[0]
        poses[k] = (x + (math.cos(yaw) * right - math.sin(yaw) * forward) * TICK_S,
                    y + (math.sin(yaw) * right + math.cos(yaw) * forward) * TICK_S, yaw + velocity[2] * TICK_S)
    return poses


def write_recording(path, modes, sticks, strides=None, odometry=None, features=None) -> None:
    features = {"firmwareVersion": "0.1.1"} if features is None else features
    strides = np.zeros(len(modes)) if strides is None else strides
    odometry = np.zeros((len(modes), 3)) if odometry is None else odometry
    robot_us = np.round((ROBOT_START_S + np.arange(len(modes)) * TICK_S) * 1e6).astype(int)
    writer = RecordingWriter(path, features)
    for first in range(0, len(modes), 10):
        ticks = [pb.TickSample(seq=k, t_us=int(robot_us[k]), mode=int(modes[k]), gait=int(Gait.TRI_GATE),
                               command=[*sticks[k], 0.0, 0.0, 0.0, 0.0, 0.0, strides[k]],
                               odom_x=odometry[k, 0], odom_y=odometry[k, 1], odom_yaw=odometry[k, 2])
                 for k in range(first, min(first + 10, len(modes)))]
        batch = pb.Message(telemetry_batch=pb.TelemetryBatch(batch_seq=first // 10 + 1, ticks=ticks))
        writer.write(batch.SerializeToString(), int(robot_us[ticks[-1].seq]) + 25_000)
    writer.close()


def write_track(path, floor, track_less_robot: float, rng) -> None:
    """The floor path seen at TRACK_FPS with the camera's noise, on a clock track_less_robot ahead of the robot's."""
    robot_s = ROBOT_START_S + np.arange(len(floor)) * TICK_S
    t = np.arange(robot_s[0] + track_less_robot, robot_s[-1] + track_less_robot, 1 / TRACK_FPS)
    rows = np.column_stack([t] + [np.interp(t - track_less_robot, robot_s, floor[:, i]) for i in range(3)])
    rows[:, 1:3] += rng.normal(0.0, TRACK_NOISE_M, (len(t), 2))
    rows[:, 3] = score_track.wrap(rows[:, 3] + rng.normal(0.0, TRACK_NOISE_RAD, len(t)))
    with open(path, "w") as out:
        out.write("# synthetic; clock=video\nt,x,y,yaw\n")
        np.savetxt(out, rows, delimiter=",", fmt="%.6f")


def scored(tmp_path, translation_scale, turn_scale, track_less_robot, odometry=None):
    """The score, the floor path, and the path the sticks command; `odometry` (x, y, yaw per tick) is recorded in the
    ticks as the firmware records its odometry."""
    modes, sticks = course_ticks()
    floor = walk(modes, sticks, translation_scale, turn_scale, START)
    write_recording(tmp_path / "run.hexrec", modes, sticks, odometry=odometry)
    write_track(tmp_path / "track.csv", floor, track_less_robot, np.random.default_rng(3))
    rec = recording.load(tmp_path / "run.hexrec")
    result = score_track.score(Track.load(tmp_path / "track.csv"), Telemetry.from_recording(rec))
    return result, floor, walk(modes, sticks, 1.0, 1.0, (0.0, 0.0, 0.0))


def made_good(poses) -> float:
    return float(np.hypot(*(poses[-1, :2] - poses[0, :2])))


# A video's clock starts near zero, far from the robot's.
@pytest.mark.parametrize("track_less_robot", [-1003.21, 0.4])
def test_the_clock_offset_is_found_on_the_start_motion(tmp_path, track_less_robot):
    result, _, _ = scored(tmp_path, 1.0, 1.0, track_less_robot)
    assert result.offset == pytest.approx(track_less_robot, abs=0.008)
    assert result.correlation > 0.9


def test_a_robot_that_walks_as_commanded_scores_within_the_cameras_noise(tmp_path):
    result, floor, _ = scored(tmp_path, 1.0, 1.0, 12.5)
    assert set(result.stretches) == {"commanded"}  # the recording carries no odometry
    commanded = result.stretches["commanded"]
    assert commanded.position_rms < 0.003 and commanded.yaw_rms < 0.5
    assert commanded.path_scale == pytest.approx(1.0, abs=0.01)
    assert commanded.track_made_good == pytest.approx(made_good(floor), abs=0.003)
    assert commanded.estimate_made_good == pytest.approx(made_good(floor), abs=0.003)


def test_a_robot_that_walks_short_and_turns_wide_is_scored_by_how_much(tmp_path):
    result, floor, plan = scored(tmp_path, 0.9, 1.05, 12.5)
    commanded = result.stretches["commanded"]
    assert commanded.path_scale == pytest.approx(1 / 0.9, abs=0.01)
    turned = np.degrees(plan[-1, 2])
    assert commanded.yaw_end == pytest.approx(-0.05 * turned, abs=0.3)
    assert commanded.yaw_drift < 0  # the commanded yaw falls behind the floor's as the robot walks
    assert commanded.track_made_good == pytest.approx(made_good(floor), abs=0.003)
    assert commanded.estimate_made_good == pytest.approx(made_good(plan), abs=0.003)
    laid = score_track.rotate(START[2], plan[-1, :2]) + START[:2]
    assert commanded.position_end == pytest.approx(np.hypot(*(laid - floor[-1, :2])), abs=0.004)


def test_odometry_in_the_recording_is_scored_beside_the_commanded_motion(tmp_path):
    modes, sticks = course_ticks()
    # Odometry that tracks the floor exactly, in a frame of its own: it starts at the origin facing its +y.
    odometry = walk(modes, sticks, 0.9, 1.05, (0.0, 0.0, 0.0))
    result, _, _ = scored(tmp_path, 0.9, 1.05, 12.5, odometry)
    assert set(result.stretches) == {"commanded", "odometry"}
    exact, commanded = result.stretches["odometry"], result.stretches["commanded"]
    assert exact.position_rms < 0.003 and exact.yaw_rms < 0.5
    assert exact.path_scale == pytest.approx(1.0, abs=0.01)
    assert commanded.position_rms > 10 * exact.position_rms


def _stride_recording(tmp_path, mode):
    modes, sticks = course_ticks()
    strides = np.where(modes == mode, 0.5, 0.0)
    write_recording(tmp_path / "run.hexrec", modes, sticks, strides)
    return recording.load(tmp_path / "run.hexrec")


def test_a_recording_that_walked_at_a_nonzero_stride_is_refused(tmp_path):
    with pytest.raises(ValueError, match="nonzero stride"):
        Telemetry.from_recording(_stride_recording(tmp_path, Mode.WALK))


def test_a_stride_set_only_while_standing_does_not_change_the_walk(tmp_path):
    telemetry = Telemetry.from_recording(_stride_recording(tmp_path, Mode.STAND))
    modes, sticks = course_ticks()
    walked = walk(modes, sticks, 1.0, 1.0, (0.0, 0.0, 0.0))
    assert telemetry.estimates["commanded"] == pytest.approx(walked, abs=1e-6)  # the sticks cross the wire as float32


# Recordings from before the stick mapping of 2026-10-19 hold mirrored strafes and turns (docs/conventions.md); the
# header's firmware version, as record.py and the app write it, tells them apart where it can.
@pytest.mark.parametrize("features", [{}, {"firmwareName": "Hexapod"}, {"firmwareVersion": "0.1.0"},
                                      {"firmwareVersion": "0.0.1"}, {"firmwareVersion": "synthetic"}])
def test_a_recording_from_before_the_stick_mapping_is_refused(tmp_path, features):
    modes, sticks = course_ticks()
    write_recording(tmp_path / "run.hexrec", modes, sticks, features=features)
    with pytest.raises(ValueError, match="stick mapping"):
        Telemetry.from_recording(recording.load(tmp_path / "run.hexrec"))


@pytest.mark.parametrize("version", ["0.1.1", "0.1.2", "0.2.0", "1.0.0"])
def test_a_recording_from_a_firmware_that_may_map_the_sticks_so_is_scored(tmp_path, version):
    modes, sticks = course_ticks()
    write_recording(tmp_path / "run.hexrec", modes, sticks, features={"firmwareVersion": version})
    assert Telemetry.from_recording(recording.load(tmp_path / "run.hexrec")).estimates["commanded"].shape[1] == 3


def _ticks(command):
    return {"t_us": np.arange(4) * 5000, "mode": np.full(4, int(Mode.WALK)), "gait": np.zeros(4, dtype=int),
            "command": command}


def test_a_recording_without_a_stride_entry_is_scored():
    older = np.tile([0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], (4, 1))
    assert score_track.commanded_path(_ticks(older)).shape == (4, 3)


def test_a_recording_without_ticks_has_an_empty_path():
    assert score_track.commanded_path({"t_us": np.array([]), "mode": np.array([]), "gait": np.array([]),
                                       "command": np.array([])}).shape == (0, 3)
