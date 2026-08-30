"""Search the best OPEN-LOOP gait for each terrain, so the learned policy has a real opponent.

Comparing a trained policy against one hand-tuned gait proves only that the policy beats *that*
gait. The honest question is whether it beats the best open-loop gait that terrain admits -- an
oracle baseline that is told which ground it is on and gets to pick stride, cadence, step height,
duty factor, leg phasing and ride height accordingly. That is what this produces.

The search space is `src/robot/gait_schedule.py` (18 parameters, including the metachronal phase
family that contains tripod / bipod / wave / ripple). The objective is `src/sim/rollout.score`,
the same scalar `bench_terrain.py` reports, measured on the same rollout code.

  python optimize_gait_terrain.py --workers 16                     # whole suite, CMA-ES
  python optimize_gait_terrain.py --terrains bumps_80 rocks_80     # a subset
  python optimize_gait_terrain.py --optimizer tpe --budget 600     # Bayesian (Optuna TPE) instead
  python optimize_gait_terrain.py --curb                           # the curb-climb fixture

Search seeds (0..n-1) are disjoint from the benchmark seeds (500+), and every reported number is
re-measured on the benchmark seeds, so a gait that merely memorized its search terrain shows up as
a validation drop rather than as a win.
"""

import argparse
import json
import os
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from src.envs.hexapod_mj_env import GAIT_COEF
from src.robot.gait_schedule import (BOUNDS, BOUNDS_FREE, NAMES, NAMES_FREE, DIM, DIM_FREE,
                                     FREE_OFFSET_NAMES, GaitSchedule, seed_schedule, NAMED_PATTERNS)
from src.sim.mj_runtime import CONTROL_DT
from src.sim.rollout import (
    COMMANDS, COMMAND_SETS, TERRAINS, TERRAIN_BY_NAME, CURB_HEIGHTS, CURB_CMD, CURB_SECONDS,
    EPISODE_S, ZERO_CFG, curb_score, episode, zero_predict,
)

OUT = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_library.json")
# Active search space: swapped to the free-offset variant by --free-offsets (see gait_schedule).
ACTIVE_NAMES, ACTIVE_BOUNDS = NAMES, BOUNDS
LO = np.array([BOUNDS[k][0] for k in NAMES])
HI = np.array([BOUNDS[k][1] for k in NAMES])


def use_free_offsets():
    global ACTIVE_NAMES, ACTIVE_BOUNDS, LO, HI, FREE
    FREE = True
    ACTIVE_NAMES, ACTIVE_BOUNDS = NAMES_FREE, BOUNDS_FREE
    LO = np.array([BOUNDS_FREE[k][0] for k in NAMES_FREE])
    HI = np.array([BOUNDS_FREE[k][1] for k in NAMES_FREE])


def _space(free):
    """(names, lo, hi) for a search space. Taken by value rather than read from module globals
    because worker processes are spawned, not forked: they re-import this module with the default
    space and never see --free-offsets, so the flag has to travel in the job payload."""
    names = NAMES_FREE if free else NAMES
    bounds = BOUNDS_FREE if free else BOUNDS
    return (names,
            np.array([bounds[k][0] for k in names]),
            np.array([bounds[k][1] for k in names]))


def unit_to_params(u):
    return LO + np.clip(np.asarray(u, dtype=float), 0.0, 1.0) * (HI - LO)


def params_to_unit(p):
    return np.clip((np.asarray(p, dtype=float) - LO) / (HI - LO), 0.0, 1.0)


# --------------------------------------------------------------------------- evaluation
# A "condition" is (name, [(kind, height), ...], is_curb). Per-terrain conditions carry one cell;
# the `single` condition carries the whole suite, because the gait that has to serve every terrain
# at once is the one a policy is actually competing against.
def _evaluate(payload):
    """Mean score of one candidate over the (cell, command, seed) grid of a condition."""
    u, cells, seeds, randomize, is_curb = payload[:5]
    weights = payload[5] if len(payload) > 5 else None
    commands = payload[6] if len(payload) > 6 else COMMANDS
    fixed = payload[7] if len(payload) > 7 else None
    free = payload[8] if len(payload) > 8 else False
    feas_w = payload[9] if len(payload) > 9 else 0.0
    arc = payload[10] if len(payload) > 10 else False
    names, lo, hi = _space(free)
    sched = GaitSchedule(**dict(zip(names, lo + np.clip(u, 0.0, 1.0) * (hi - lo))))
    if fixed:  # pin some parameters (e.g. the leg phase pattern) and search only the rest
        for k, v in fixed.items():
            setattr(sched, k, float(v))
    predict = zero_predict()
    # Joint-feasibility penalty. MuJoCo clamps commands to jnt_range silently, so a gait can score
    # well while executing a trajectory its own IK never asked for -- and on hardware the clamp is
    # the servo calibration, which will not match exactly. Penalising the fraction of out-of-range
    # commands keeps the search inside what the legs can actually do.
    feas_pen = 0.0
    if feas_w > 0.0:
        from src.sim.ik_feasibility_lite import out_of_range_fraction
        feas_pen = feas_w * out_of_range_fraction(sched)
    if is_curb:
        steps = int(CURB_SECONDS / CONTROL_DT)
        ms = [episode(predict, ZERO_CFG, k, h, CURB_CMD, s, randomize, steps, gait_schedule=sched,
                      weights=weights, arc_stance=arc) for (k, h) in cells for s in seeds]
        return float(np.mean([curb_score(m, weights) for m in ms])) - feas_pen, ms
    steps = int(EPISODE_S / CONTROL_DT)
    ms = [episode(predict, ZERO_CFG, k, h, cmd, s, randomize, steps, gait_schedule=sched,
                  weights=weights, arc_stance=arc)
          for (k, h) in cells for _, cmd in commands for s in seeds]
    return float(np.mean([m["score"] for m in ms])) - feas_pen, ms


WEIGHTS = None    # score-weight override, set from --score-* in main (see rollout.score)
CMD_SET = COMMANDS  # command set the search optimizes over, set from --command-set in main
FIXED = None      # parameters pinned to fixed values, set from --fix-pattern in main
FREE = False      # searching all five leg offsets directly, set from --free-offsets in main
FEAS_W = 0.0      # weight on the out-of-joint-range fraction, set from --feasible-penalty in main
ARC = False       # arc stance path about the ICR, set from --arc-stance in main


def evaluate_batch(pool, cands, cond, seeds, randomize, weights=None):
    _, cells, is_curb = cond
    jobs = [(u, cells, seeds, randomize, is_curb, weights or WEIGHTS, CMD_SET, FIXED, FREE,
             FEAS_W, ARC) for u in cands]
    if pool is None:
        return [_evaluate(j) for j in jobs]
    return list(pool.map(_evaluate, jobs))


# --------------------------------------------------------------------------- optimizers
def run_cma(pool, cond, seeds, randomize, budget, popsize, x0, sigma, rng_seed):
    from cmaes import CMA

    d = len(x0)
    opt = CMA(mean=np.asarray(x0, dtype=float), sigma=sigma, population_size=popsize,
              bounds=np.column_stack([np.zeros(d), np.ones(d)]), seed=rng_seed)
    best_u, best_f, used, history = np.asarray(x0, dtype=float), -1e9, 0, []
    while used < budget:
        cands = [opt.ask() for _ in range(opt.population_size)]
        results = evaluate_batch(pool, cands, cond, seeds, randomize)
        used += len(cands)
        opt.tell([(c, -f) for c, (f, _) in zip(cands, results)])
        gen_best = int(np.argmax([f for f, _ in results]))
        if results[gen_best][0] > best_f:
            best_f, best_u = results[gen_best][0], cands[gen_best]
        history.append(best_f)
        if opt.should_stop():
            break
    return best_u, best_f, used, history


def run_tpe(pool, cond, seeds, randomize, budget, popsize, x0, rng_seed):
    """Optuna TPE -- a Bayesian (density-ratio) surrogate, batched to keep the pool busy."""
    import optuna

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(direction="maximize",
                                sampler=optuna.samplers.TPESampler(seed=rng_seed, n_startup_trials=2 * popsize))
    study.enqueue_trial({k: float(v) for k, v in zip(ACTIVE_NAMES, x0)})
    best_u, best_f, used, history = np.asarray(x0, dtype=float), -1e9, 0, []
    while used < budget:
        trials = [study.ask() for _ in range(popsize)]
        cands = [np.array([t.suggest_float(k, 0.0, 1.0) for k in ACTIVE_NAMES]) for t in trials]
        results = evaluate_batch(pool, cands, cond, seeds, randomize)
        used += len(cands)
        for t, (f, _) in zip(trials, results):
            study.tell(t, f)
            if f > best_f:
                best_f, best_u = f, np.array([t.params[k] for k in ACTIVE_NAMES])
        history.append(best_f)
    return best_u, best_f, used, history


def run_de(pool, cond, seeds, randomize, budget, popsize, rng_seed):
    """Differential evolution over the unit cube. `vectorized` hands scipy a whole generation at
    once, which is what lets the existing process pool evaluate it (a closure is not picklable, so
    scipy's own `workers` cannot be used here)."""
    from scipy.optimize import differential_evolution

    state = {"used": 0, "best_f": -1e9, "best_u": None, "history": []}

    def batch(x):                      # x: (DIM, S)
        cands = [x[:, i] for i in range(x.shape[1])]
        results = evaluate_batch(pool, cands, cond, seeds, randomize)
        state["used"] += len(cands)
        for c, (f, _) in zip(cands, results):
            if f > state["best_f"]:
                state["best_f"], state["best_u"] = f, np.asarray(c, dtype=float)
        state["history"].append(state["best_f"])
        return np.array([-f for f, _ in results])


    d = len(LO)
    maxiter = max(1, budget // max(1, popsize * d))
    differential_evolution(batch, [(0.0, 1.0)] * d, maxiter=maxiter, popsize=popsize,
                           seed=rng_seed, polish=False, vectorized=True, init="sobol", tol=1e-4)
    return state["best_u"], state["best_f"], state["used"], state["history"]


# --------------------------------------------------------------------------- candidate seeds
def seed_points():
    """Starting points the search is given for free: the tuned coefficients on each named gait
    pattern, at a duty factor appropriate to it. A search that cannot beat these is reported as
    such rather than silently returning something worse."""
    pts = {}
    for name, (lr, ll, contra) in NAMED_PATTERNS.items():
        s = seed_schedule(GAIT_COEF)
        s.lag_r, s.lag_l, s.contra = lr, ll, contra
        s.duty = {"tripod": 0.52, "bipod": 0.35, "wave": 0.84, "ripple": 0.84}[name]
        if ACTIVE_NAMES is NAMES_FREE:
            # the seeds are defined by the metachronal family; materialize them as explicit
            # offsets so the free search starts from the same gaits rather than from NaN
            for n, o in zip(FREE_OFFSET_NAMES, s.offsets()[1:]):
                setattr(s, n, float(o))
        pts[name] = params_to_unit([getattr(s, k) for k in ACTIVE_NAMES])
    return pts


# --------------------------------------------------------------------------- driver
def optimize_condition(pool, cond, args):
    name, cells, is_curb = cond
    search_seeds = list(range(args.search_seeds))
    val_seeds = [args.val_seed0 + i for i in range(args.val_seeds)]

    # 1. score the free seed points on the search seeds; start CMA from the best of them
    pts = seed_points()
    seed_scores = dict(zip(pts, [f for f, _ in evaluate_batch(pool, list(pts.values()), cond,
                                                              search_seeds, args.randomize)]))
    best_seed_name = max(seed_scores, key=seed_scores.get)
    x0 = pts[best_seed_name]
    note = "  (pattern pinned: seed labels denote duty presets only)" if FIXED else ""
    print(f"  seed points: " + "  ".join(f"{k}={v:+.3f}" for k, v in seed_scores.items())
          + f"   -> start from '{best_seed_name}'" + note)

    t0 = time.time()
    if args.optimizer == "cma":
        u, f, used, _ = run_cma(pool, cond, search_seeds, args.randomize, args.budget,
                                args.popsize, x0, args.sigma, args.seed)
    elif args.optimizer == "tpe":
        u, f, used, _ = run_tpe(pool, cond, search_seeds, args.randomize, args.budget,
                                args.popsize, x0, args.seed)
    else:
        u, f, used, _ = run_de(pool, cond, search_seeds, args.randomize, args.budget,
                               args.popsize, args.seed)

    # 2. re-measure winner AND every seed point on held-out seeds; keep whichever actually wins
    contenders = {"search": u, **pts}
    val = dict(zip(contenders, [r for r in evaluate_batch(pool, list(contenders.values()), cond,
                                                          val_seeds, args.randomize)]))
    val_scores = {k: v[0] for k, v in val.items()}
    winner = max(val_scores, key=val_scores.get)
    best_u = contenders[winner]
    sched = GaitSchedule(**dict(zip(ACTIVE_NAMES, unit_to_params(best_u))))
    if FIXED:  # the evaluated schedule had these pinned; the stored one must match
        for k, v in FIXED.items():
            setattr(sched, k, float(v))
    metrics = val[winner][1]

    print(f"  search {f:+.3f} (search seeds, {used} evals, {time.time()-t0:.0f} s)  |  held-out: "
          + "  ".join(f"{k}={v:+.3f}" for k, v in val_scores.items()) + f"  -> keep '{winner}'")
    return {
        "terrain": name, "cells": [[k, h] for k, h in cells], "curb": is_curb,
        "params": sched.to_dict(),
        "offsets": [round(float(x), 4) for x in sched.offsets()],
        "search_score": f, "search_from": best_seed_name, "winner": winner,
        "val_score": val_scores[winner], "val_scores": val_scores,
        "val_detail": {k: float(np.mean([m[k] for m in metrics]))
                       for k in ("progress", "stuck", "vel_err", "yaw_err", "tilt_deg", "knock",
                                 "fell", "dy")},
        # Per-entry provenance. The top-level `meta` only ever describes the LAST search that wrote
        # the file (--merge overwrites it), so entries must carry their own settings or a reader
        # will misattribute one search's budget and command set to all of them.
        "evals": used, "optimizer": args.optimizer, "budget": args.budget,
        "command_set": args.command_set, "score_weights": WEIGHTS,
        "search_seeds": args.search_seeds, "val_seeds": args.val_seeds,
        "randomize": bool(args.randomize), "fix_pattern": args.fix_pattern,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["per-terrain", "single", "curb"], default="per-terrain",
                    help="per-terrain: one gait per condition (oracle upper bound); "
                         "single: ONE gait maximizing the mean over the whole suite -- the honest "
                         "opponent for a policy, which is also a single controller; "
                         "curb: the step-climb fixture, swept by height")
    ap.add_argument("--terrains", nargs="*", default=None,
                    help="subset of the suite (default: all)")
    ap.add_argument("--curb-heights", nargs="*", type=float, default=list(CURB_HEIGHTS))
    ap.add_argument("--optimizer", choices=["cma", "tpe", "de"], default="cma")
    ap.add_argument("--budget", type=int, default=640, help="candidate evaluations per condition")
    ap.add_argument("--popsize", type=int, default=16, help="batch size (match --workers)")
    ap.add_argument("--sigma", type=float, default=0.25, help="CMA initial step (unit cube)")
    ap.add_argument("--search-seeds", type=int, default=4)
    ap.add_argument("--val-seeds", type=int, default=8)
    ap.add_argument("--val-seed0", type=int, default=500, help="must match bench_terrain --seed0")
    ap.add_argument("--randomize", action="store_true", help="domain randomization during search")
    ap.add_argument("--workers", type=int, default=16)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--merge", action="store_true", help="update an existing library instead of replacing")
    ap.add_argument("--score-knock", type=float, default=None, help="override rollout.score knock weight")
    ap.add_argument("--score-rate", type=float, default=None,
                    help="price body-rate RMS (smoothness) in the objective; off by default")
    ap.add_argument("--key-suffix", default="", help="appended to library keys (e.g. _smooth)")
    ap.add_argument("--command-set", choices=list(COMMAND_SETS), default="bench",
                    help="commands the search optimizes over; 'wide' spans the same envelope the "
                         "policy trains on, which is what makes the comparison fair")
    ap.add_argument("--arc-stance", action="store_true",
                    help="sweep the planted foot along the arc about the instantaneous centre of "
                         "rotation instead of the chord; only meaningful if the gait is SEARCHED "
                         "with it, since a chord-tuned gait compensates for the chord")
    ap.add_argument("--feasible-penalty", type=float, default=0.0,
                    help="subtract this times the fraction of commanded joint angles outside "
                         "jnt_range; keeps the search off gaits that only work because MuJoCo "
                         "clamps them (several library entries are at 4-7%)")
    ap.add_argument("--free-offsets", action="store_true",
                    help="search all five leg phase offsets directly instead of the 3-parameter "
                         "metachronal family, to test whether that family costs anything")
    ap.add_argument("--fix-pattern", choices=list(NAMED_PATTERNS), default=None,
                    help="pin the leg phase pattern and search only the other 15 parameters -- "
                         "answers 'what is the BEST tripod (or bipod)?', which is a different "
                         "question from 'what is the best gait?'")
    args = ap.parse_args()

    if args.free_offsets:
        use_free_offsets()
        print(f"free-offset search: {DIM_FREE} dims (adds {', '.join(FREE_OFFSET_NAMES)})")

    global WEIGHTS, CMD_SET, FIXED, FEAS_W, ARC
    ARC = args.arc_stance
    FEAS_W = args.feasible_penalty
    if FEAS_W:
        print(f"joint-feasibility penalty: {FEAS_W} x out-of-range fraction")
    CMD_SET = COMMAND_SETS[args.command_set]
    print(f"command set '{args.command_set}': {[c[0] for c in CMD_SET]}")
    if args.fix_pattern:
        lr, ll, contra = NAMED_PATTERNS[args.fix_pattern]
        FIXED = {"lag_r": lr, "lag_l": ll, "contra": contra}
        print(f"pattern pinned to '{args.fix_pattern}': {FIXED}")
    over = {k: v for k, v in (("knock", args.score_knock), ("rate", args.score_rate))
            if v is not None}
    WEIGHTS = over or None
    if WEIGHTS:
        print(f"score weight override: {WEIGHTS}")

    names = args.terrains or [n for n, _, _ in TERRAINS]
    if args.mode == "curb":
        conds = [(f"curb_{int(h*1000)}", [("curb", h)], True) for h in args.curb_heights]
    elif args.mode == "single":
        conds = [("__single__", [TERRAIN_BY_NAME[n] for n in names], False)]
    else:
        conds = [(n, [TERRAIN_BY_NAME[n]], False) for n in names]

    lib = {"meta": {}, "gaits": {}}
    if args.merge and os.path.exists(args.out):
        lib = json.load(open(args.out))
        lib.setdefault("gaits", {})

    pool = ProcessPoolExecutor(max_workers=args.workers) if args.workers > 1 else None
    t0 = time.time()
    try:
        for cond in conds:
            print(f"\n[{cond[0]}] cells={cond[1]} optimizer={args.optimizer} budget={args.budget}")
            lib["gaits"][cond[0] + args.key_suffix] = optimize_condition(pool, cond, args)
            lib["meta"] = {"note": "describes the MOST RECENT search only; per-entry settings "
                                   "live on each gait entry",
                           "optimizer": args.optimizer, "budget": args.budget,
                           "search_seeds": args.search_seeds, "val_seeds": args.val_seeds,
                           "val_seed0": args.val_seed0, "randomize": bool(args.randomize),
                           "score_weights": WEIGHTS, "command_set": args.command_set,
                           "params": list(ACTIVE_NAMES)}
            with open(args.out, "w", newline="\n") as fh:
                json.dump(lib, fh, indent=1)
    finally:
        if pool is not None:
            pool.shutdown()

    print(f"\ntotal {time.time()-t0:.0f} s -> {args.out}")
    print(f"\n{'terrain':12s} {'winner':8s} {'val':>7s} {'default-tripod':>15s} {'progress':>9s} "
          f"{'stuck':>6s} {'duty':>5s} {'step_h':>7s} {'ride':>6s}")
    for n, g in lib["gaits"].items():
        p = g["params"]
        print(f"{n:12s} {g['winner']:8s} {g['val_score']:7.3f} {g['val_scores'].get('tripod', float('nan')):15.3f} "
              f"{g['val_detail']['progress']:9.2f} {g['val_detail']['stuck']:6.2f} "
              f"{p['duty']:5.2f} {p['step_height']:7.2f} {p['ride_mm']:6.1f}")


if __name__ == "__main__":
    main()
