"""Does the training reward rank gaits the way the benchmark does?

A policy can only find a good gait if its reward prefers one. Under the original weights it barely
did: the searched gait and the legacy gait score 2.818 vs 2.756 in reward (a 2 % gap) while
`rollout.score` rates them 0.90 vs 0.17. Training started at a 0.94 gait and converged to 0.26 --
not because PPO failed, but because it succeeded at the objective it was given.

This measures the agreement directly. It draws a population of gaits (the named patterns, every
library entry, and random draws from the schedule bounds), evaluates each one BOTH ways on
identical terrain and commands, and reports Spearman rank correlation. Then it does the same for
candidate reward-weight sets, so the weights are chosen by measurement.

  python reward_alignment.py --n-random 24 --workers 12
  python reward_alignment.py --n-random 24 --workers 12 --apply vel8_angvel02
"""

import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np
from scipy.stats import spearmanr

from src.envs.hexapod_mj_env import HexapodMjEnv, REWARD_WEIGHTS
from src.robot.gait_schedule import BOUNDS, NAMES, GaitSchedule, NAMED_PATTERNS, seed_schedule
from src.sim.mj_runtime import CONTROL_DT
from src.sim.rollout import COMMAND_SETS, COMMANDS, EPISODE_S, ZERO_CFG, episode, zero_predict

LIB = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_library.json")

# Candidate weight sets. The hypothesis under test is that tracking is under-weighted relative to
# the shaping penalties, so each candidate raises `vel` and/or relaxes the terms that were
# cancelling it (`angvel`, `knock`, `power`).
CANDIDATES = {
    "default": {},
    "vel8_angvel02_knock01": {"vel": 8.0, "angvel": 0.02, "knock": 0.1},
    # `score` MULTIPLIES the velocity and yaw kernels, so a yaw error costs as much as a velocity
    # error. The reward ADDS them at 8 : 2, which under-prices yaw by 4x -- and yaw tracking is
    # exactly where the trained policies were weakest. These candidates test correcting that.
    "vel8_yaw4": {"vel": 8.0, "yaw": 4.0, "angvel": 0.02, "knock": 0.1},
    "vel8_yaw8": {"vel": 8.0, "yaw": 8.0, "angvel": 0.02, "knock": 0.1},
    "vel6_yaw6": {"vel": 6.0, "yaw": 6.0, "angvel": 0.02, "knock": 0.1},
    "angvel0_knock0": {"angvel": 0.0, "knock": 0.0},
}

CELLS = [("bumps", 0.0), ("bumps", 0.08), ("rocks", 0.08), ("steps", 0.12)]


def population(n_random, seed=0):
    """Gaits to rank: the four named patterns, every library entry, and random draws. The random
    draws matter -- a correlation measured only on good gaits says nothing about whether the reward
    can tell a good gait from a bad one."""
    pop = {}
    for name, (lr, ll, contra) in NAMED_PATTERNS.items():
        s = seed_schedule()
        s.lag_r, s.lag_l, s.contra = lr, ll, contra
        s.duty = {"tripod": 0.52, "bipod": 0.35, "wave": 0.84, "ripple": 0.84}[name]
        pop[f"named:{name}"] = s
    pop["legacy"] = None  # the env's built-in analytic map
    if os.path.exists(LIB):
        for k, g in json.load(open(LIB))["gaits"].items():
            if not g.get("curb"):
                pop[f"lib:{k}"] = GaitSchedule.from_dict(g["params"])
    rng = np.random.default_rng(seed)
    lo = np.array([BOUNDS[k][0] for k in NAMES])
    hi = np.array([BOUNDS[k][1] for k in NAMES])
    for i in range(n_random):
        pop[f"rand:{i}"] = GaitSchedule.from_vector(lo + rng.random(len(NAMES)) * (hi - lo))
    return pop


def _job(payload):
    """Both measurements for one gait: benchmark score, and mean training reward per weight set."""
    params, seeds, weight_sets, commands = payload
    sched = GaitSchedule.from_dict(params) if params else None
    steps = int(EPISODE_S / CONTROL_DT)

    scores = [episode(zero_predict(), ZERO_CFG, k, h, cmd, s, False, steps, gait_schedule=sched)
              ["score"] for (k, h) in CELLS for _, cmd in commands for s in seeds]

    rewards = {}
    for name, w in weight_sets.items():
        tot, n = 0.0, 0
        for (k, h) in CELLS:
            for _, cmd in commands:
                for s in seeds:
                    env = HexapodMjEnv("residual_sched", terrain=h, terrain_kind=k, obs_history=4,
                                       episode_seconds=EPISODE_S, gait_schedule=sched, seed=s,
                                       reward_weights=w)
                    env.fixed_command = np.asarray(cmd, dtype=np.float32)
                    env.reset(seed=s)
                    a = np.zeros(27, dtype=np.float32)
                    for _ in range(steps):
                        _, r, term, _, _ = env.step(a)
                        tot += r
                        n += 1
                        if term:
                            tot -= 1.0
                            break
        rewards[name] = tot / max(n, 1)
    return float(np.mean(scores)), rewards


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-random", type=int, default=24)
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--command-set", choices=list(COMMAND_SETS), default="bench")
    ap.add_argument("--apply", default=None, help="write this candidate's weights to resources/reward_weights.json")
    args = ap.parse_args()

    pop = population(args.n_random)
    seeds = list(range(600, 600 + args.seeds))
    commands = COMMAND_SETS[args.command_set]
    jobs = [(None if s is None else s.to_dict(), seeds, CANDIDATES, commands) for s in pop.values()]
    print(f"{len(pop)} gaits x {len(CELLS)} cells x {len(commands)} commands x {args.seeds} seeds, "
          f"{len(CANDIDATES)} weight sets")

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        results = list(ex.map(_job, jobs, chunksize=1))

    scores = np.array([r[0] for r in results])
    print(f"\n{'weight set':24s} {'spearman':>9s} {'p':>9s} {'top-5 overlap':>14s}")
    ranked_by_score = set(np.argsort(-scores)[:5])
    best = None
    for name in CANDIDATES:
        rew = np.array([r[1][name] for r in results])
        rho, p = spearmanr(scores, rew)
        overlap = len(ranked_by_score & set(np.argsort(-rew)[:5]))
        print(f"{name:24s} {rho:9.3f} {p:9.2e} {overlap:11d}/5")
        if best is None or rho > best[1]:
            best = (name, rho)
    print(f"\nbest agreement: {best[0]} (rho={best[1]:.3f})")

    names = list(pop)
    order = np.argsort(-scores)
    print(f"\n{'gait':26s} {'score':>7s} " + " ".join(f"{n[:12]:>13s}" for n in CANDIDATES))
    for i in list(order[:8]) + list(order[-4:]):
        print(f"{names[i]:26s} {scores[i]:7.3f} "
              + " ".join(f"{results[i][1][n]:13.3f}" for n in CANDIDATES))

    if args.apply:
        w = {**REWARD_WEIGHTS, **CANDIDATES[args.apply]}
        out = os.path.join(os.path.dirname(__file__), "src", "resources", "reward_weights.json")
        with open(out, "w", newline="\n") as f:
            json.dump(w, f, indent=2)
        print(f"\nwrote {out} ({args.apply})")


if __name__ == "__main__":
    main()
