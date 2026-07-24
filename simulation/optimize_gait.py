"""Black-box optimize the open-loop gait map (command -> gait params).

Searches the analytic gait coefficients to maximize the zero-residual rollout reward across a
representative command set, so the learned residual policy only needs small corrections on top.
Differential evolution is used because evals are cheap (~0.1s rollouts), where BO's surrogate
overhead does not pay off. Result -> resources/gait_coef.json, loaded by env.analytic_gait_action.

  python optimize_gait.py --maxiter 30 --popsize 12
"""

import argparse
import json
import os
import numpy as np
from scipy.optimize import differential_evolution

from src.envs.hexapod_mj_env import HexapodMjEnv, GAIT_COEF, CMD_VX, CMD_VY, CMD_YAW

PARAMS = ["gx0", "gx1", "gy0", "gy1", "gyaw0", "gyaw1", "blend_speed", "pr_base", "pr_slope",
          "step_height", "yaw_comp", "step_depth", "pr_yaw"]
# blend_speed bounded high so tripod stays default and bipod only emerges at high speed. pr_yaw is
# a separate cadence gain for turning (in-place turns have ~0 translational speed).
BOUNDS = [(0.15, 0.60), (0.20, 0.70), (0.15, 0.60), (0.20, 0.70), (1.0, 3.5), (1.2, 4.0),
          (0.25, 0.45), (-0.5, 1.0), (0.0, 5.0), (-1.0, 0.2), (-3.0, 3.0), (0.0, 8.0), (0.0, 2.5)]

COMMANDS = [(-0.25, 0, 0), (-0.10, 0, 0), (0.15, 0, 0), (0.30, 0, 0), (0.45, 0, 0),
            (0, -0.10, 0), (0, 0.10, 0), (0, 0, -0.8), (0, 0, 0.8),
            (0.30, 0, 0.5), (0.20, 0.10, 0), (-0.15, 0, 0.4)]

OUT = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_coef.json")


def rollout_reward(env, seed, steps=200):
    o, _ = env.reset(seed=seed)  # fixed seed keeps the DR realization deterministic across evals
    total, zero = 0.0, np.zeros(18, dtype=np.float32)
    for _ in range(steps):
        _, r, term, _, _ = env.step(zero)  # zero residual -> pure analytic gait
        total += r
        if term:
            total -= 5.0
            break
    return total


_ENVS = None


def objective(theta):
    global _ENVS
    if _ENVS is None:  # build once, reuse: avoids reloading the MJCF every eval
        # randomize=True so robustness under disturbances counts toward the objective
        _ENVS = [HexapodMjEnv("residual_pure", command=c, randomize=True, seed=0) for c in COMMANDS]
    for k, v in zip(PARAMS, theta):
        GAIT_COEF[k] = float(v)
    # two fixed DR seeds per command for a less-noisy robustness estimate
    return -sum(rollout_reward(env, s) for env in _ENVS for s in (1, 2)) / (2 * len(_ENVS))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maxiter", type=int, default=30)
    ap.add_argument("--popsize", type=int, default=12)
    ap.add_argument("--workers", type=int, default=1, help="parallel workers (-1 = all cores)")
    args = ap.parse_args()

    base = objective([GAIT_COEF[k] for k in PARAMS])
    print(f"baseline (hand-tuned map) mean reward/step-rollout: {-base:.2f}")

    # parallel workers need deferred updating (a whole generation is evaluated per map call)
    updating = "deferred" if args.workers != 1 else "immediate"
    res = differential_evolution(objective, BOUNDS, maxiter=args.maxiter, popsize=args.popsize,
                                 seed=0, tol=1e-3, polish=True, disp=True,
                                 workers=args.workers, updating=updating)
    best = dict(zip(PARAMS, [round(float(v), 5) for v in res.x]))
    print(f"\noptimized mean reward: {-res.fun:.2f}  (baseline {-base:.2f})")
    print("optimized coefficients:")
    for k, v in best.items():
        print(f"  {k:12s} {v:+.4f}   (was {GAIT_COEF[k] if False else ''})")
    with open(OUT, "w", newline="\n") as f:
        json.dump(best, f, indent=2)
    print(f"\nsaved -> {OUT} (env.analytic_gait_action now uses these)")


if __name__ == "__main__":
    main()
