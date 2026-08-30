"""Does sweeping the planted foot along the true arc beat the straight chord?

`stroke = v + omega x r` gives each foot the correct instantaneous velocity, but the stance then
moves it linearly, so mid-stance the foot sits up to 20 mm off the path the body actually takes.
A planted foot that cannot follow the body has to slip. This measures whether closing that gap
buys anything, on turn-heavy commands where the error is largest.

Paired: identical gait, terrain, seed and command, only the stance path differs.

  python arc_stance_test.py --gait __single___feas --workers 14
"""

import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np
from scipy.stats import wilcoxon

from src.robot.gait_schedule import GaitSchedule
from src.sim.mj_runtime import CONTROL_DT
from src.sim.rollout import EPISODE_S, ZERO_CFG, episode, zero_predict

LIB = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_library.json")
# turn-heavy: the arc-vs-chord gap grows with |yaw|, so a forward-only set would show nothing
COMMANDS = [
    ("spin_slow", (0.0, 0.0, 0.4)),
    ("spin_fast", (0.0, 0.0, 0.9)),
    ("arc_slow", (0.10, 0.0, 0.6)),
    ("arc_fast", (0.25, 0.0, 0.8)),
    ("arc_rev", (0.15, 0.0, -0.7)),
    ("fwd_only", (0.25, 0.0, 0.0)),     # control: arc must be a no-op here
]
CELLS = [("bumps", 0.0), ("bumps", 0.08), ("rocks", 0.12)]
SEEDS = (500, 501, 502, 503)


def _job(payload):
    params, kind, h, cmd, seed, arc = payload
    sched = GaitSchedule.from_dict(params)
    m = episode(zero_predict(), ZERO_CFG, kind, h, cmd, seed, False,
                int(EPISODE_S / CONTROL_DT), gait_schedule=sched, arc_stance=arc)
    return kind, h, cmd, seed, arc, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", default="__single___feas")
    ap.add_argument("--workers", type=int, default=14)
    args = ap.parse_args()

    params = json.load(open(LIB))["gaits"][args.gait]["params"]
    jobs = [(params, k, h, cmd, sd, arc)
            for (k, h) in CELLS for _, cmd in COMMANDS for sd in SEEDS for arc in (False, True)]
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        res = list(ex.map(_job, jobs))

    idx = {(k, h, c, s, a): m for k, h, c, s, a, m in res}
    print(f"gait '{args.gait}', {len(CELLS)} terrains x {len(SEEDS)} seeds, arc vs chord (paired)")
    print(f"\n{'command':11s} {'dscore':>8s} {'p':>7s} {'dyaw_err':>9s} {'dslip':>8s} "
          f"{'dknock':>8s} {'wins':>7s}")
    alld = []
    for label, cmd in COMMANDS:
        d, dy, dk = [], [], []
        for (k, h) in CELLS:
            for sd in SEEDS:
                a, b = idx[(k, h, cmd, sd, True)], idx[(k, h, cmd, sd, False)]
                d.append(a["score"] - b["score"])
                dy.append(a["yaw_err"] - b["yaw_err"])
                dk.append(a["knock"] - b["knock"])
        d = np.array(d)
        alld += list(d)
        p = wilcoxon(d).pvalue if np.any(d != 0) else 1.0
        print(f"{label:11s} {d.mean():+8.4f} {p:7.4f} {np.mean(dy):+9.4f} {'':>8s} "
              f"{np.mean(dk):+8.4f} {int((d > 0).sum()):4d}/{len(d):<3d}")
    alld = np.array(alld)
    p = wilcoxon(alld).pvalue if np.any(alld != 0) else 1.0
    print(f"\npooled dscore={alld.mean():+.4f} (p={p:.4f}), "
          f"wins={int((alld > 0).sum())}/{len(alld)}")
    print("fwd_only is the control: with step_angle=0 the arc code is bypassed, so it must read 0.")


if __name__ == "__main__":
    main()
