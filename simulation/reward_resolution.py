"""How well does the training reward resolve gaits NEAR the optimum?

`reward_alignment.py` measures rank agreement over a broad population of gaits, and gets rho ~= 0.81.
That number is reassuring and misleading. A warm-started residual policy never visits a bad gait --
it lives in a small neighbourhood of an already-good one, and the question that decides whether it
can improve anything is whether the reward can order gaits *inside that neighbourhood*.

This perturbs the best known gait and correlates score against reward there. Global rank agreement
over a wide population says nothing about local resolution near the top.

  python reward_resolution.py --sigma 0.06 --n 24 --workers 14
"""

import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np
from scipy.stats import spearmanr

from src.envs.hexapod_mj_env import HexapodMjEnv
from src.robot.gait_schedule import BOUNDS, NAMES, GaitSchedule
from src.sim.mj_runtime import CONTROL_DT
from src.sim.rollout import COMMANDS_WIDE, EPISODE_S, ZERO_CFG, episode

LIB = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_library.json")
LO = np.array([BOUNDS[k][0] for k in NAMES])
HI = np.array([BOUNDS[k][1] for k in NAMES])
RW = {"vel": 8.0, "angvel": 0.02, "knock": 0.1}   # the weights sched_v4 trained with
CELLS = [("bumps", 0.0), ("rocks", 0.08), ("steps", 0.12)]
STEPS = int(EPISODE_S / CONTROL_DT)


def _job(vec):
    sched = GaitSchedule.from_vector(vec)
    from src.envs.hexapod_mj_env import ACT_DIM
    z24 = np.zeros(ACT_DIM[ZERO_CFG["control_mode"]], dtype=np.float32)
    z = np.zeros(ACT_DIM["residual_sched"], dtype=np.float32)
    scores = [episode(lambda o: z24, ZERO_CFG, k, h, cmd, 800 + i, False, STEPS, gait_schedule=sched)
              ["score"] for i, (k, h) in enumerate(CELLS) for _, cmd in COMMANDS_WIDE]
    tot, n = 0.0, 0
    for k, h in CELLS:
        for _, cmd in COMMANDS_WIDE:
            env = HexapodMjEnv("residual_sched", terrain=h, terrain_kind=k, obs_history=4,
                               episode_seconds=EPISODE_S, gait_schedule=sched, seed=800,
                               reward_weights=RW)
            env.fixed_command = np.asarray(cmd, dtype=np.float32)
            env.reset(seed=800)
            for _ in range(STEPS):
                _, r, term, _, _ = env.step(z)
                tot += r
                n += 1
                if term:
                    break
    return float(np.mean(scores)), tot / max(n, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--key", default="__single___wide3")
    ap.add_argument("--sigma", type=float, default=0.06, help="perturbation, as a fraction of each bound range")
    ap.add_argument("--n", type=int, default=24)
    ap.add_argument("--workers", type=int, default=14)
    args = ap.parse_args()

    base = GaitSchedule.from_dict(json.load(open(LIB))["gaits"][args.key]["params"]).to_vector()
    rng = np.random.default_rng(0)
    cands = [base.copy()] + [np.clip(base + rng.normal(0, args.sigma, len(NAMES)) * (HI - LO), LO, HI)
                             for _ in range(args.n - 1)]

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        res = list(ex.map(_job, cands))
    sc = np.array([r[0] for r in res])
    rw = np.array([r[1] for r in res])
    rho, p = spearmanr(sc, rw)

    print(f"near '{args.key}', {args.n} perturbations at sigma={args.sigma:.0%} of range")
    print(f"  score  range {sc.min():.3f} .. {sc.max():.3f}   (base {sc[0]:.3f})")
    print(f"  reward range {rw.min():.3f} .. {rw.max():.3f}   (base {rw[0]:.3f})")
    print(f"  spearman rho = {rho:+.3f}  p={p:.3f}")
    rank_of_reward_pick = int(np.argsort(np.argsort(-sc))[int(np.argmax(rw))]) + 1
    print(f"  the gait the REWARD prefers ranks {rank_of_reward_pick}/{args.n} by score")
    print(f"  score lost by following the reward: {sc.max() - sc[int(np.argmax(rw))]:+.3f}")


if __name__ == "__main__":
    main()
