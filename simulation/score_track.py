"""Scores how the robot's own idea of its motion held up against an overhead camera's track:
uv run python score_track.py recording.hexrec track.csv

The robot's idea is the commanded motion: each tick's sticks (the telemetry's command) through the gait's measured
speed model (src/hexapod/speed_model.py) while the robot is in WALK, eased in and out by the firmware's command ramp
(GAIT_COMMAND_TAU_S) and integrated over the ticks. WALK_NN and ANIMATE count as standing still, which they are not;
score a run that stays in STAND and WALK. The speed model is the speed-optimal stroke's (stride 0), so a recording that
walked at another stride is refused. Where the recording's ticks also carry the firmware's odometry (the columns
ODOMETRY_COLUMNS: x and y in metres, yaw in radians, in a frame of its own), that is scored beside it; the robot does
not report odometry yet, so today only the commanded motion is scored.

The track (ground_truth.py) and the telemetry recording (record.py) run on different clocks: the camera's frames carry
the host's monotonic time or a video's own, the ticks the robot's. The offset between the two is searched for: the
body's velocity and yaw rate, from the commanded motion and from the track, are correlated over the start motion, from
SYNC_LEAD_S before the commanded motion's first move to SYNC_S after it, at every offset that keeps it inside the
track. A course should therefore open with a distinct start motion from standing, such as a quarter turn in place.

Then each estimate is laid on the floor at the recording's start (the track's and the estimate's poses averaged over
its first ALIGN_S, while the robot stands) and compared with the track at every tick the track covers: the position
error's RMS and end value, the yaw error's RMS, end value and drift per metre walked (each end the mean over the last
ALIGN_S), the distance made good (start to end in a straight line) by the track and by the estimate, and the ratio of
the path the estimate counted to the path the track measured. Both paths are sampled every PATH_STEP_S and summed over
the steps in which the estimate moved at least WALKING_STEP_M, so that neither counts much of the gait's sway and the
camera's noise counts nothing while the robot stands.

Frames follow docs/conventions.md: x to the right, y forward, yaw counter-clockwise from above.
"""
import argparse
import math
import sys
from dataclasses import dataclass

import numpy as np

import ground_truth
from src.hexapod import Gait, Mode
from src.hexapod.speed_model import Sticks, velocity_of
from src.robot.firmware_gait import GAIT_COMMAND_TAU_S
from src.sim import recording

ODOMETRY_COLUMNS = ("odometry_x", "odometry_y", "odometry_yaw")
STRIDE = 8  # the stride's place in a tick's command (lx, ly, rx, ry, h, s, s1, fd, stride); older recordings end before it
GRID_S = 0.01
SMOOTH_S = 0.2
SYNC_S = 10.0
SYNC_LEAD_S = 1.0
SIGNAL_SCALES = (0.05, 0.05, 0.5)  # m/s, m/s, rad/s: a walk's and a turn's typical speeds weigh alike
MOVE_M, MOVE_RAD = 0.01, np.radians(3.0)  # what counts as the commanded motion's first move
ALIGN_S = 0.5
MAX_GAP_S = 0.2  # the track is interpolated across no longer a gap than this
PATH_STEP_S = 0.25
WALKING_STEP_M = 0.005
MIN_DRIFT_PATH_M = 0.2  # a yaw drift per metre needs at least this much walking


def wrap(angle):
    return np.angle(np.exp(1j * np.asarray(angle, dtype=float)))


def rotate(theta, vectors) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    vectors = np.asarray(vectors, dtype=float)
    return np.stack([c * vectors[..., 0] - s * vectors[..., 1], s * vectors[..., 0] + c * vectors[..., 1]], axis=-1)


@dataclass(frozen=True)
class Track:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray  # unwrapped

    @classmethod
    def load(cls, path) -> "Track":
        rows = ground_truth.load_track(path)
        return cls(rows[:, 0], rows[:, 1], rows[:, 2], np.unwrap(rows[:, 3]))

    def at(self, t) -> np.ndarray:
        """x, y and yaw (columns) at track times t; NaN outside the track and across gaps over MAX_GAP_S."""
        t = np.atleast_1d(np.asarray(t, dtype=float))
        poses = np.column_stack([np.interp(t, self.t, v) for v in (self.x, self.y, self.yaw)])
        after = np.clip(np.searchsorted(self.t, t), 1, len(self.t) - 1)
        covered = (t >= self.t[0]) & (t <= self.t[-1]) & (self.t[after] - self.t[after - 1] <= MAX_GAP_S)
        poses[~covered] = np.nan
        return poses


def commanded_path(ticks: dict[str, np.ndarray]) -> np.ndarray:
    """x, y and yaw (columns) per tick of a robot that walks exactly what its sticks command, from the origin facing
    +y: the speed model's velocity in WALK, nothing otherwise, through the firmware's first-order command ramp."""
    t = ticks["t_us"] / 1e6
    command = ticks["command"]
    if command.shape[1] > STRIDE and np.any(command[ticks["mode"] == Mode.WALK, STRIDE] != 0.0):
        raise ValueError("the recording walked at a nonzero stride, which the stride-0 speed model does not describe")
    velocities = {}
    velocity = np.zeros(3)  # forward, left, counter-clockwise
    poses = np.zeros((len(t), 3))
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        target = np.zeros(3)
        if ticks["mode"][k - 1] == Mode.WALK:
            key = (int(ticks["gait"][k - 1]), *(float(v) for v in ticks["command"][k - 1][:3]))
            if key not in velocities:
                v = velocity_of(Gait(key[0]), Sticks(*key[1:]))
                velocities[key] = np.array([v.vx, v.vy, v.yaw_rate])
            target = velocities[key]
        velocity += (1 - math.exp(-dt / GAIT_COMMAND_TAU_S)) * (target - velocity)
        right_forward = rotate(poses[k - 1, 2], [-velocity[1], velocity[0]])
        poses[k, :2] = poses[k - 1, :2] + right_forward * dt
        poses[k, 2] = poses[k - 1, 2] + velocity[2] * dt
    return poses


@dataclass(frozen=True)
class Telemetry:
    t: np.ndarray  # each tick on the robot's clock (s)
    estimates: dict[str, np.ndarray]  # name -> x, y and unwrapped yaw (columns) per tick

    @classmethod
    def from_recording(cls, rec: recording.Recording) -> "Telemetry":
        estimates = {"commanded": commanded_path(rec.ticks)}
        if all(column in rec.ticks for column in ODOMETRY_COLUMNS):
            x, y, yaw = (np.asarray(rec.ticks[column], dtype=float) for column in ODOMETRY_COLUMNS)
            estimates["odometry"] = np.column_stack([x, y, np.unwrap(yaw)])
        return cls(rec.ticks["t_us"] / 1e6, estimates)

    def at(self, name: str, t) -> np.ndarray:
        poses = self.estimates[name]
        return np.column_stack([np.interp(t, self.t, poses[:, i]) for i in range(3)])


def motion_signals(x, y, yaw) -> np.ndarray:
    """The body's rightward and forward speed and its yaw rate (rows) for poses sampled every GRID_S, smoothed over
    SMOOTH_S: in the body's frame, so the same in an estimate's frame as on the floor, and signed, so that noise does
    not add up to a speed while the robot stands."""
    width = round(SMOOTH_S / GRID_S)
    pad = (width // 2, width - 1 - width // 2)  # the ends held, so they do not sink towards zero
    smooth = [np.convolve(np.pad(v, pad, mode="edge"), np.ones(width) / width, mode="valid") for v in (x, y, yaw)]
    vx, vy, yaw_rate = (np.gradient(v, GRID_S) for v in smooth)
    right, forward = rotate(-smooth[2], np.stack([vx, vy], axis=-1)).T
    return np.vstack([right, forward, yaw_rate])


def first_move(t: np.ndarray, poses: np.ndarray) -> float:
    moved = (np.hypot(*(poses[:, :2] - poses[0, :2]).T) > MOVE_M) | (np.abs(poses[:, 2] - poses[0, 2]) > MOVE_RAD)
    if not moved.any():
        raise ValueError("the commanded motion never moves: there is no start motion to align the clocks on")
    return float(t[np.argmax(moved)])


def clock_offset(track: Track, telemetry: Telemetry, sync_s: float = SYNC_S) -> tuple[float, float]:
    """The offset to add to robot time for track time, and the correlation it reaches (1 at best)."""
    start = first_move(telemetry.t, telemetry.estimates["commanded"]) - SYNC_LEAD_S
    window_t = start + np.arange(round((SYNC_LEAD_S + sync_s) / GRID_S)) * GRID_S
    window = motion_signals(*telemetry.at("commanded", window_t).T)
    track_t = np.arange(track.t[0], track.t[-1], GRID_S)
    n = len(window_t)
    if len(track_t) < n:
        raise ValueError("the track is shorter than the start motion's window")
    signals = motion_signals(*(np.interp(track_t, track.t, v) for v in (track.x, track.y, track.yaw)))
    numerator = np.zeros(len(track_t) - n + 1)
    track_power, window_power = np.zeros_like(numerator), 0.0
    for w, s, scale in zip(window, signals, SIGNAL_SCALES):
        w, s = (w - w.mean()) / scale, s / scale
        numerator += np.correlate(s, w, mode="valid")
        sums, squares = (np.concatenate([[0.0], np.cumsum(v)]) for v in (s, s * s))
        track_power += (squares[n:] - squares[:-n]) - (sums[n:] - sums[:-n]) ** 2 / n
        window_power += float(w @ w)
    correlation = numerator / np.sqrt(window_power * np.maximum(track_power, 1e-12))
    best = int(np.argmax(correlation))
    shift = 0.0
    if 0 < best < len(correlation) - 1:
        before, peak, after = correlation[best - 1 : best + 2]
        shift = 0.5 * (before - after) / (before - 2 * peak + after)
    return float(track_t[0] + (best + shift) * GRID_S - window_t[0]), float(correlation[best])


@dataclass(frozen=True)
class Stretch:
    seconds: float
    position_rms: float  # m
    position_end: float
    yaw_rms: float  # degrees
    yaw_end: float  # the estimate's yaw less the track's
    yaw_drift: float  # degrees per metre walked, NaN under MIN_DRIFT_PATH_M
    track_made_good: float  # m, start to end in a straight line
    estimate_made_good: float
    track_path: float  # m
    estimate_path: float

    @property
    def path_scale(self) -> float:
        return self.estimate_path / self.track_path if self.track_path > 0 else float("nan")


def cumulative_path(xy: np.ndarray, walking: np.ndarray) -> np.ndarray:
    """The path up to each point over the walking steps, counting none from or to a point the track does not cover."""
    steps = np.where(walking, np.nan_to_num(np.hypot(*np.diff(xy, axis=0).T)), 0.0)
    return np.concatenate([[0.0], np.cumsum(steps)])


def score_estimate(track: Track, telemetry: Telemetry, name: str, offset: float) -> Stretch | None:
    """None when the track does not see the recording's start."""
    t = telemetry.t
    truth = track.at(t + offset)
    estimate = telemetry.estimates[name]
    covered = ~np.isnan(truth[:, 0])
    first = covered & (t < t[0] + ALIGN_S)
    if not first.any():
        return None
    last = covered & (t >= t[covered][-1] - ALIGN_S)
    truth0, estimate0 = truth[first].mean(axis=0), estimate[first].mean(axis=0)
    turn = truth0[2] - estimate0[2]
    position = rotate(turn, estimate[:, :2] - estimate0[:2]) + truth0[:2]
    position_error = np.hypot(*(position - truth[:, :2])[covered].T)
    yaw_error = np.degrees(wrap(estimate[covered, 2] + turn - truth[covered, 2]))
    samples = np.arange(t[0], t[-1], PATH_STEP_S)
    estimate_xy = telemetry.at(name, samples)[:, :2]
    walking = np.hypot(*np.diff(estimate_xy, axis=0).T) >= WALKING_STEP_M
    track_path = cumulative_path(track.at(samples + offset)[:, :2], walking)
    walked = np.interp(t[covered], samples, track_path)
    drift = np.polyfit(walked, yaw_error, 1)[0] if track_path[-1] >= MIN_DRIFT_PATH_M else np.nan
    rms = lambda values: float(np.sqrt(np.mean(np.square(values))))
    made_good = lambda poses: float(np.hypot(*(poses[last, :2].mean(axis=0) - poses[first, :2].mean(axis=0))))
    return Stretch(float(t[covered][-1] - t[0]), rms(position_error), float(position_error[last[covered]].mean()),
                   rms(yaw_error), float(yaw_error[last[covered]].mean()), float(drift), made_good(truth),
                   made_good(estimate), float(track_path[-1]), float(cumulative_path(estimate_xy, walking)[-1]))


@dataclass(frozen=True)
class Score:
    offset: float  # track time less robot time
    correlation: float
    stretches: dict[str, Stretch]  # per estimate the track saw the start of


def score(track: Track, telemetry: Telemetry, sync_s: float = SYNC_S) -> Score:
    offset, correlation = clock_offset(track, telemetry, sync_s)
    stretches = {name: score_estimate(track, telemetry, name, offset) for name in telemetry.estimates}
    return Score(offset, correlation, {name: s for name, s in stretches.items() if s is not None})


def report(result: Score) -> None:
    print(f"clock offset {result.offset:+.3f} s (track less robot), correlation {result.correlation:.2f}")
    for name, s in result.stretches.items():
        print(f"{name} over {s.seconds:.1f} s: made good {s.estimate_made_good:.3f} m for {s.track_made_good:.3f} m "
              f"on the floor; position {100 * s.position_rms:.1f} cm RMS, {100 * s.position_end:.1f} cm at the end; "
              f"yaw {s.yaw_rms:.1f} degrees RMS, {s.yaw_end:+.1f} at the end, drift {s.yaw_drift:+.2f} degrees/m; "
              f"path {s.estimate_path:.3f} m counted for {s.track_path:.3f} m walked (x{s.path_scale:.3f})")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("recording", help="the .hexrec telemetry recording (record.py)")
    parser.add_argument("track", help="the track file (ground_truth.py)")
    parser.add_argument("--sync-s", type=float, default=SYNC_S, help="how long the start motion lasts (s)")
    args = parser.parse_args()
    report(score(Track.load(args.track), Telemetry.from_recording(recording.load(args.recording)), args.sync_s))
    return 0


if __name__ == "__main__":
    sys.exit(main())
