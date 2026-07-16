"""Black-box optimize the open-loop gait map (command -> gait params).

The hand-tuned analytic gait map (env.GAIT_COEF) has weak spots (e.g. backward yaw drift). Here
we search its ~10 coefficients to maximize the ZERO-RESIDUAL rollout reward across a representative
set of commands -- i.e. the best nominal open-loop gait (velocity/yaw tracking + low cost-of-transport
+ stability). The learned residual policy then only needs small corrections on top.

Evals are cheap (~0.1s sim rollouts), so differential evolution / CMA beats Bayesian optimization
here (BO's surrogate overhead only pays off when evals are expensive). Result -> resources/gait_coef.json,
which env.analytic_gait_action loads automatically.

  python optimize_gait.py --maxiter 30 --popsize 12
"""

import argparse
import json
import os
import numpy as np
from scipy.optimize import differential_evolution

from src.envs.hexapod_mj_env import HexapodMjEnv, GAIT_COEF, CMD_VX, CMD_VY, CMD_YAW

PARAMS = ["gx0", "gx1", "gy0", "gy1", "gyaw0", "gyaw1", "blend_speed", "pr_base", "pr_slope",
          "step_height", "yaw_comp", "step_depth"]
# blend_speed bounded HIGH so tripod (stable) is the default and bipod only emerges at high speed
# ("running"). step_depth lets the optimizer add stance-phase downward push for traction.
BOUNDS = [(0.15, 0.60), (0.20, 0.70), (0.15, 0.60), (0.20, 0.70), (1.0, 3.5), (1.2, 4.0),
          (0.25, 0.45), (-0.5, 1.0), (0.0, 5.0), (-1.0, 0.2), (-3.0, 3.0), (0.0, 8.0)]

# representative commands across the wider range (slow->fast forward/back/strafe/turn/diagonal)
COMMANDS = [(-0.25, 0, 0), (-0.10, 0, 0), (0.10, 0, 0), (0.25, 0, 0), (0.40, 0, 0),
            (0, -0.20, 0), (0, 0.20, 0), (0, 0, -1.2), (0, 0, 1.2),
            (0.30, 0, 0.6), (0.20, 0.15, 0), (-0.15, 0, 0.5)]

OUT = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_coef.json")


def rollout_reward(env, seed, steps=200):
    o, _ = env.reset(seed=seed)  # fixed seed -> deterministic DR realization (so DE sees a stable objective)
    total, zero = 0.0, np.zeros(18, dtype=np.float32)
    for _ in range(steps):
        _, r, term, _, _ = env.step(zero)  # zero residual -> pure analytic gait
        total += r
        if term:
            total -= 5.0  # extra penalty for falling
            break
    return total


_ENVS = None


def objective(theta):
    global _ENVS
    if _ENVS is None:  # build once, reuse (avoids reloading the MJCF every eval)
        # randomize=True so stability under disturbances/pushes counts -> tripod's robustness shows up
        _ENVS = [HexapodMjEnv("residual_pure", command=c, randomize=True, seed=0) for c in COMMANDS]
    for k, v in zip(PARAMS, theta):
        GAIT_COEF[k] = float(v)
    # two fixed DR seeds per command for a less-noisy robustness estimate
    return -sum(rollout_reward(env, s) for env in _ENVS for s in (1, 2)) / (2 * len(_ENVS))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maxiter", type=int, default=30)
    ap.add_argument("--popsize", type=int, default=12)
    args = ap.parse_args()

    base = objective([GAIT_COEF[k] for k in PARAMS])
    print(f"baseline (hand-tuned map) mean reward/step-rollout: {-base:.2f}")

    res = differential_evolution(objective, BOUNDS, maxiter=args.maxiter, popsize=args.popsize,
                                 seed=0, tol=1e-3, polish=True, disp=True)
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
