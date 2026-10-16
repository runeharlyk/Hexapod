"""Does the firmware's self-levelling term reduce body tilt, or amplify it?

motion.h adds the measured pitch/roll, in the body frame (imu_frame.h), to the commanded body attitude.
Whether that levels the robot or tips it further depends on the IMU mounting, the DMP's angle
convention and this robot's X-lateral body frame; imu_frame.h records the signs measured on the robot.
This measures which sign of the environment's own copy levels the body in the sim, because training
against an amplifying body would be worse than not modelling the term at all.

Runs the analytic gait over rough ground with levelling off, +1 and -1, and reports mean body tilt. This sets the
sign of the environment's own copy of the term, which the research modes use; residual_pure and residual_gait run
the firmware's levelling itself, so it measures in residual_sched, whose zero action is the same base gait.

  uv run python check_self_level.py
"""

import numpy as np

import src.envs.hexapod_mj_env as E
from src.envs.hexapod_mj_env import HexapodMjEnv


def mean_tilt(sign, self_level, terrain=0.06, steps=600, seed=0):
    E.LEVEL_SIGN = sign
    env = HexapodMjEnv(control_mode="residual_sched", randomize=False, seed=seed,
                       terrain=terrain, terrain_kind="bumps", self_level=self_level)
    env.reset(seed=seed)
    env.cmd = np.array([0.25, 0.0, 0.0], dtype=np.float32)  # steady forward walk
    tilts, fell = [], False
    for _ in range(steps):
        _, _, terminated, truncated, _ = env.step(np.zeros(env.action_space.shape, dtype=np.float32))
        tilts.append(env.sim.body_tilt_deg())
        if terminated:
            fell = True
            break
        if truncated:
            break
    env.close()
    return float(np.mean(tilts)), float(np.max(tilts)), fell, len(tilts)


def main():
    print(f"{'config':16s} {'mean tilt':>10s} {'max tilt':>9s} {'steps':>6s}  fell")
    rows = []
    for label, sign, on in (("off", 1.0, False), ("+1 (firmware)", 1.0, True), ("-1", -1.0, True)):
        m, mx, fell, n = mean_tilt(sign, on)
        rows.append((label, m, mx, n, fell))
        print(f"{label:16s} {m:9.2f}d {mx:8.2f}d {n:6d}  {'YES' if fell else 'no'}")

    off = rows[0][1]
    best = min(rows[1:], key=lambda r: r[1])
    print()
    if best[1] < off:
        print(f"-> LEVEL_SIGN = {'+1' if best[0].startswith('+') else '-1'} reduces tilt "
              f"({best[1]:.2f}d vs {off:.2f}d with levelling off)")
    else:
        print(f"-> neither sign reduces tilt below {off:.2f}d; the term is not levelling here")


if __name__ == "__main__":
    main()
