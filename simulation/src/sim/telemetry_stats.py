"""What a robot recording says about the assumptions the simulation trains with.

The control loop's timing is compared with the firmware's 5 ms period, and the IMU at rest with the
noise and bias domain_rand.py draws. Only ticks that read a fresh IMU sample count for the IMU, and
only stretches where the robot is still: the gait would otherwise pass for noise.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

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


def tick_timing(rec: Recording) -> TickTiming:
    seq = rec.ticks["seq"].astype(np.int64)
    # A tick after a hole in seq has its period measured across the missing ticks; leave it out.
    contiguous = np.concatenate([[False], np.diff(seq) == 1])
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
