"""One-parameter-at-a-time sensitivity of a searched gait.

The library says which gait won; it does not say which parameters MATTER. A parameter whose profile
is flat over its whole range was never worth searching and is not worth defending in firmware; a
parameter with a sharp peak is one where the shipped value being slightly wrong is expensive.

This sweeps each named parameter across its bounds with everything else held at the optimum, and
reports the score profile, the span (best - worst) and where the peak sits relative to the value the
search chose.

  python gait_sensitivity.py --key __single___wide3 --params lag_r lag_l contra duty
  python gait_sensitivity.py --key __single___wide3 --all --points 9
"""

import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from src.robot.gait_schedule import BOUNDS, NAMES, GaitSchedule
from src.sim.mj_runtime import CONTROL_DT
from src.sim.rollout import COMMANDS_WIDE, EPISODE_S, ZERO_CFG, episode, zero_predict

LIB = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_library.json")
CELLS = [("bumps", 0.0), ("bumps", 0.08), ("rocks", 0.12)]
SEEDS = (500, 501)


def _job(payload):
    params, name, value = payload
    s = GaitSchedule.from_dict(params)
    if name is not None:
        setattr(s, name, float(value))
    steps = int(EPISODE_S / CONTROL_DT)
    predict = zero_predict()
    sc = [episode(predict, ZERO_CFG, k, h, cmd, sd, False, steps, gait_schedule=s)["score"]
          for (k, h) in CELLS for _, cmd in COMMANDS_WIDE for sd in SEEDS]
    return name, value, float(np.mean(sc))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--key", default="__single___wide3")
    ap.add_argument("--params", nargs="*", default=["lag_r", "lag_l", "contra", "duty"])
    ap.add_argument("--all", action="store_true", help="sweep every parameter")
    ap.add_argument("--points", type=int, default=11)
    ap.add_argument("--workers", type=int, default=14)
    args = ap.parse_args()

    entry = json.load(open(LIB))["gaits"][args.key]
    base = entry["params"]
    names = list(NAMES) if args.all else args.params

    jobs = [(base, None, 0.0)]
    for n in names:
        lo, hi = BOUNDS[n]
        for v in np.linspace(lo, hi, args.points):
            jobs.append((base, n, float(v)))

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        res = list(ex.map(_job, jobs))

    nominal = res[0][2]
    by = {}
    for name, value, sc in res[1:]:
        by.setdefault(name, []).append((value, sc))

    print(f"gait '{args.key}', nominal score {nominal:.3f} "
          f"({len(CELLS)} terrains x {len(COMMANDS_WIDE)} commands x {len(SEEDS)} seeds)")
    print(f"\n{'param':12s} {'chosen':>8s} {'best@':>8s} {'best':>6s} {'worst':>6s} {'span':>6s}  profile")
    rows = []
    for n in names:
        pts = sorted(by[n])
        vals = np.array([v for v, _ in pts])
        scs = np.array([s for _, s in pts])
        span = float(scs.max() - scs.min())
        rows.append((span, n, vals, scs))
    for span, n, vals, scs in sorted(rows, reverse=True):
        i = int(np.argmax(scs))
        bar = "".join(" .:-=+*#%@"[min(9, max(0, int(9 * (s - scs.min()) / (span + 1e-9))))] for s in scs)
        print(f"{n:12s} {base[n]:8.3f} {vals[i]:8.3f} {scs.max():6.3f} {scs.min():6.3f} "
              f"{span:6.3f}  |{bar}|")
    lo_hi = {n: BOUNDS[n] for n in names}
    print(f"\nprofile runs low->high over each parameter's bounds: "
          + ", ".join(f"{n}[{lo_hi[n][0]:g},{lo_hi[n][1]:g}]" for n in names[:4])
          + ("..." if len(names) > 4 else ""))
    print("span = best - worst over the sweep. A flat profile means the parameter does not matter.")


if __name__ == "__main__":
    main()
