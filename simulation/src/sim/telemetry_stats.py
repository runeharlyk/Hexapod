"""What a robot recording says about the assumptions the simulation trains with.

The control loop's timing is compared with the firmware's 5 ms period, and the IMU at rest with the
noise and bias domain_rand.py draws. Only ticks that read a fresh IMU sample count for the IMU, and
only stretches where the robot is still: the gait would otherwise pass for noise. The servo writes
and the dead-man stops are counted from every tick that arrived.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.platform_shared import message_pb2
from src.sim.recording import Recording

CONTROL_PERIOD_US = 5000
# A tick this much later than its slot is an overrun the firmware restarts its schedule after.
OVERRUN_US = 2 * CONTROL_PERIOD_US
# A body rate under this (rad/s) on every axis, and no stick command, counts as standing still.
STILL_GYRO_RAD_S = 0.2


@dataclass
class TickTiming:
    ticks: int
    period_mean_us: float
    period_p99_us: float
    overruns: int
    compute_p99_us: float


@dataclass
class ImuAtRest:
    samples: int
    gyro_bias: np.ndarray       # rad/s, mean per axis
    gyro_noise: np.ndarray      # rad/s, standard deviation per axis
    rpy_noise_deg: np.ndarray   # standard deviation per axis
    gravity_noise: np.ndarray   # standard deviation per axis


@dataclass
class BoardWrites:
    ok: int
    failed: int     # attempted and not acknowledged
    skipped: int    # not attempted: the board was backing off after failed writes
    restarted: int  # answered without its configuration after failing, and was started again
    failed_seqs: np.ndarray


@dataclass
class ServoWrites:
    boards: dict[str, BoardWrites]  # "right" (0x41, servos 0-8) and "left" (0x40, servos 9-17)
    write_p99_us: float             # over the ticks that put a write on the bus
    write_max_us: int
    write_max_seq: int


@dataclass
class DeadManStops:
    fired_seqs: np.ndarray        # the first tick each stop was in force on
    after_hole_seqs: np.ndarray   # those of them right after a hole in seq: the stop fired within the hole
    in_force_at_start: bool       # a stop from before the recording; not in fired_seqs
    ticks_in_force: int


def _contiguous(seq: np.ndarray) -> np.ndarray:
    """Per tick, whether it directly follows the tick before it."""
    return np.concatenate([[False], np.diff(seq.astype(np.int64)) == 1])


def tick_timing(rec: Recording) -> TickTiming:
    seq = rec.ticks["seq"].astype(np.int64)
    # A tick after a hole in seq has its period measured across the missing ticks; leave it out.
    contiguous = _contiguous(seq)
    period = rec.ticks["period_us"][contiguous].astype(float)
    compute = rec.ticks["compute_us"].astype(float)
    return TickTiming(
        ticks=len(seq),
        period_mean_us=float(period.mean()) if len(period) else float("nan"),
        period_p99_us=float(np.percentile(period, 99)) if len(period) else float("nan"),
        overruns=int(np.sum(period > OVERRUN_US)),
        compute_p99_us=float(np.percentile(compute, 99)) if len(compute) else float("nan"),
    )


def imu_at_rest(rec: Recording) -> ImuAtRest:
    fresh = rec.ticks["imu_fresh"].astype(bool)
    gyro = rec.ticks["gyro"]
    sticks = np.abs(rec.ticks["command"][:, :4]).max(axis=1) if len(fresh) else np.zeros(0)
    still = fresh & (np.abs(gyro).max(axis=1) < STILL_GYRO_RAD_S) & (sticks == 0)
    g, rpy, grav = gyro[still], rec.ticks["rpy"][still], rec.ticks["gravity"][still]
    if len(g) < 2:
        nan = np.full(3, np.nan)
        return ImuAtRest(int(len(g)), nan, nan, nan, nan)
    return ImuAtRest(
        samples=int(len(g)),
        gyro_bias=g.mean(axis=0),
        gyro_noise=g.std(axis=0, ddof=1),
        rpy_noise_deg=np.rad2deg(rpy.std(axis=0, ddof=1)),
        gravity_noise=grav.std(axis=0, ddof=1),
    )


def servo_writes(rec: Recording) -> ServoWrites | None:
    """What every tick's write to the two PCA9685 boards did; None for a recording without the servo write."""
    if "servo_write_us" not in rec.ticks:
        return None
    t, w = rec.ticks, message_pb2.ServoWrite
    results = {side: t[f"servo_write_{side}"] for side in ("right", "left")}
    boards = {side: BoardWrites(ok=int(np.sum(r == w.SERVO_WRITE_OK)), failed=int(np.sum(r == w.SERVO_WRITE_FAILED)),
                                skipped=int(np.sum(r == w.SERVO_WRITE_SKIPPED)),
                                restarted=int(np.sum(r == w.SERVO_WRITE_RESTARTED)),
                                failed_seqs=t["seq"][r == w.SERVO_WRITE_FAILED])
              for side, r in results.items()}
    attempted = (w.SERVO_WRITE_OK, w.SERVO_WRITE_FAILED, w.SERVO_WRITE_RESTARTED)
    on_bus = np.isin(results["right"], attempted) | np.isin(results["left"], attempted)
    times, seqs = t["servo_write_us"][on_bus], t["seq"][on_bus]
    if not len(times):
        return ServoWrites(boards, float("nan"), 0, -1)
    longest = int(np.argmax(times))
    return ServoWrites(boards, float(np.percentile(times, 99)), int(times[longest]), int(seqs[longest]))


def dead_man_stops(rec: Recording) -> DeadManStops | None:
    """Every tick the dead-man stop came into force on; None for a recording without the flag."""
    if "link_lost" not in rec.ticks:
        return None
    lost = rec.ticks["link_lost"].astype(bool)
    seq = rec.ticks["seq"]
    rose = np.concatenate([[False], lost[1:] & ~lost[:-1]])
    return DeadManStops(fired_seqs=seq[rose], after_hole_seqs=seq[rose & ~_contiguous(seq)],
                        in_force_at_start=bool(len(lost) and lost[0]), ticks_in_force=int(np.sum(lost)))
