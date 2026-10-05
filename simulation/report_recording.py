"""Compares a robot recording (record.py) with the simulation's assumptions:

    uv run python report_recording.py recording-20261005-120000.hexrec

Record with the robot standing still for a while to measure the IMU at rest; walking stretches only
count for the control loop's timing.
"""
import argparse

import numpy as np

from src.sim import telemetry_stats as ts
from src.sim.recording import load

# What domain_rand.py draws (DomainRandomizer.reset_episode and noisy_imu).
SIM_GYRO_BIAS_SIGMA = 0.05      # rad/s, per-episode bias
SIM_GYRO_NOISE = 0.05           # rad/s, per-step noise
SIM_RPY_NOISE_DEG = 1.5
SIM_GRAVITY_NOISE = 0.02


def _vec(v) -> str:
    return "[" + ", ".join(f"{x:+.4f}" for x in v) + "]"


def report(path) -> str:
    rec = load(path)
    timing = ts.tick_timing(rec)
    imu = ts.imu_at_rest(rec)
    lines = [
        f"firmware {rec.features.get('firmwareVersion', '?')} ({rec.features.get('firmwareElfSha256', '')[:12]})",
        f"ticks {timing.ticks}, missing {rec.missing_ticks} (batches {rec.missing_batches}), "
        f"dropped on the robot {rec.dropped_ticks}, link gaps {rec.link_gaps}",
        f"control period: mean {timing.period_mean_us:.0f} us (nominal {ts.CONTROL_PERIOD_US}), "
        f"p99 {timing.period_p99_us:.0f} us, overruns {timing.overruns}",
        f"compute p99 {timing.compute_p99_us:.0f} us of the {ts.CONTROL_PERIOD_US} us budget",
        f"IMU at rest: {imu.samples} fresh samples",
    ]
    if imu.samples >= 2:
        lines += [
            f"  gyro bias  {_vec(imu.gyro_bias)} rad/s   sim draws N(0, {SIM_GYRO_BIAS_SIGMA}) per episode",
            f"  gyro noise {_vec(imu.gyro_noise)} rad/s   sim {SIM_GYRO_NOISE}",
            f"  rpy noise  {_vec(imu.rpy_noise_deg)} deg     sim {SIM_RPY_NOISE_DEG}",
            f"  gravity    {_vec(imu.gravity_noise)}         sim {SIM_GRAVITY_NOISE}",
        ]
        outside = [
            name
            for name, measured, assumed in (
                ("gyro bias", np.abs(imu.gyro_bias).max(), 3 * SIM_GYRO_BIAS_SIGMA),
                ("gyro noise", imu.gyro_noise.max(), 2 * SIM_GYRO_NOISE),
                ("rpy noise", imu.rpy_noise_deg.max(), 2 * SIM_RPY_NOISE_DEG),
                ("gravity noise", imu.gravity_noise.max(), 2 * SIM_GRAVITY_NOISE),
            )
            if measured > assumed
        ]
        lines.append("  outside the trained range: " + (", ".join(outside) if outside else "nothing"))
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("recording")
    print(report(parser.parse_args().recording))


if __name__ == "__main__":
    main()
