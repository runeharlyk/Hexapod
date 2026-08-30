"""Rough-terrain benchmark: policies vs the best open-loop gait, over a paired terrain matrix.

Falls do not discriminate on a statically stable hexapod -- it high-centers and stalls instead of
tipping over -- so the headline metrics are PROGRESS (distance made good along the commanded
direction, as a fraction of the commanded distance) and SCORE (`src/sim/rollout.score`: tracking
minus stalling minus plowing, zeroed for the part of an episode spent fallen).

Arms:
  --baseline                 the legacy analytic gait (gait_coef.json + firmware tripod/bipod blend)
  --gait-library <json>      adds `tuned_oracle` (the per-terrain gait from optimize_gait_terrain.py,
                             i.e. an open-loop baseline that is TOLD which terrain it is on) and
                             `tuned_single` (the one gait with the best mean across the suite -- what
                             you could actually ship without terrain classification)
  --runs A B                 trained policies
  --reflex                   duplicates every arm with contact reflexes in the gait engine

A policy that only beats `--baseline` has beaten one hand-tuned gait, not the gait family. The
comparison that matters is against `tuned_oracle`.

  python bench_terrain.py --runs rough_nocontact --baseline --gait-library src/resources/gait_library.json --workers 14
  python bench_terrain.py --runs A B --kinds bumps steps --heights 0.04 0.08 --seeds 8
  python bench_terrain.py --runs A --curb --gait-library src/resources/gait_library.json
"""

import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np
from scipy.stats import binomtest, wilcoxon

from src.envs.hexapod_mj_env import load_env_config
from src.robot.gait_schedule import GaitSchedule
from src.sim.mj_runtime import CONTROL_DT
from src.sim.rollout import (
    COMMANDS, COMMANDS_HELDOUT, COMMAND_SETS, TERRAINS, CURB_HEIGHTS, CURB_CMD, CURB_SECONDS,
    CURB_CLEARED, EPISODE_S, ZERO_CFG, episode, zero_predict,
)
from src.sim.terrain import MIX_KINDS as KINDS

HEIGHTS = tuple(sorted({h for _, _, h in TERRAINS}))

_POLICY_CACHE = {}


def _load_policy(logdir, run):
    """(predict_fn, env_config) for a run dir, or the open-loop gait when run is None.
    Cached per worker process -- model loading dominates the cost of a 10 s episode otherwise."""
    if (logdir, run) not in _POLICY_CACHE:
        _POLICY_CACHE[(logdir, run)] = _build_policy(logdir, run)
    return _POLICY_CACHE[(logdir, run)]


def _build_policy(logdir, run):
    from src.envs.hexapod_mj_env import ACT_DIM, make_env

    if run is None:
        return zero_predict(ACT_DIM[ZERO_CFG["control_mode"]]), dict(ZERO_CFG)

    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    rundir = os.path.join(logdir, run)
    cfg = {"control_mode": "residual_gait"}
    cfg.update(load_env_config(rundir))
    model = PPO.load(os.path.join(rundir, "final_model.zip"), device="cpu",
                     custom_objects={"lr_schedule": lambda _: 0.0, "clip_range": lambda _: 0.2})
    sched = cfg.get("gait_schedule")
    vn = VecNormalize.load(os.path.join(rundir, "vecnormalize.pkl"), DummyVecEnv([make_env(
        cfg["control_mode"], obs_contact=cfg["obs_contact"], obs_history=cfg["obs_history"],
        gait_schedule=GaitSchedule.from_dict(sched) if sched else None)]))
    mean, var, clip, eps = vn.obs_rms.mean, vn.obs_rms.var, vn.clip_obs, vn.epsilon

    def predict(o):
        n = np.clip((o - mean) / np.sqrt(var + eps), -clip, clip).astype(np.float32)
        return model.predict(n, deterministic=True)[0]

    return predict, cfg


def _job(payload):
    logdir, label, run, kind, height, cmd, seed, randomize, steps, reflex, sched_params = payload
    predict, cfg = _load_policy(logdir, run)
    # an arm-level schedule wins; otherwise a policy walks on whatever base gait it was trained on
    params = sched_params or cfg.get("gait_schedule")
    sched = GaitSchedule.from_dict(params) if params else None
    return (label, kind, height, cmd[0], seed,
            episode(predict, cfg, kind, height, tuple(cmd[1]), seed, randomize, steps, reflex,
                    gait_schedule=sched))


# --------------------------------------------------------------------------- arms
def _terrain_key(kind, height):
    return "flat" if height <= 0 else f"{kind}_{int(round(height*1000))}"


SINGLE_KEY = "__single__"


def _library_arms(args):
    """(label, run, schedule-resolver) for the tuned open-loop arms."""
    if not args.gait_library:
        return []
    lib = json.load(open(args.gait_library))["gaits"]
    arms = []

    single_key = args.single_from or SINGLE_KEY
    single = lib.get(single_key)
    if single is None:
        raise SystemExit(f"{args.gait_library} has no '{single_key}' entry -- run "
                         f"optimize_gait_terrain.py --mode single (or pass --single-from)")
    single_params = single["params"]
    arms.append(("tuned_single", None, lambda k, h: single_params))

    for key in args.extra_gaits:
        entry = lib.get(key)
        if entry is None:
            raise SystemExit(f"{args.gait_library} has no '{key}' entry")
        params = entry["params"]
        arms.append((f"gait[{key.strip('_')}]", None, lambda k, h, p=params: p))

    # The oracle is told which terrain it is on. Where the library has no entry for a cell it falls
    # back to the single gait, NOT to the legacy gait -- an oracle silently degrading to the old
    # baseline would read as "terrain knowledge is worthless".
    if any(k not in (single_key,) and not v.get("curb") for k, v in lib.items()):
        missing = set()

        def oracle(kind, height):
            key = _terrain_key(kind, height)
            entry = lib.get(key)
            if entry is None:
                if key not in missing:
                    missing.add(key)
                    print(f"note: no per-terrain gait for '{key}'; oracle falls back to tuned_single")
                return single_params
            return entry["params"]

        arms.append(("tuned_oracle", None, oracle))
    return arms


_ARM_CACHE = {}


def _arms(args):
    """(label, run, schedule-resolver, reflex). Reflex duplicates every arm so the with/without
    comparison is paired on identical terrain."""
    if id(args) in _ARM_CACHE:
        return _ARM_CACHE[id(args)]
    base = [(r, r, None) for r in args.runs]
    if args.baseline:
        base.append(("analytic_gait", None, None))
    base += _library_arms(args)
    pairs = [(lbl, run, res, False) for lbl, run, res in base]
    if args.reflex:
        pairs += [(lbl + "+reflex", run, res, True) for lbl, run, res in base]
    _ARM_CACHE[id(args)] = pairs
    return pairs


def _make_jobs(args, cells, commands, steps):
    jobs = []
    for label, run, resolve, rx in _arms(args):
        for kind, h in cells:
            params = resolve(kind, h) if resolve else None
            for cmd in commands:
                for s in range(args.seeds):
                    jobs.append((args.logdir, label, run, kind, h, cmd, args.seed0 + s,
                                 args.randomize, steps, rx, params))
    return jobs


def _run(jobs, workers):
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            return list(ex.map(_job, jobs, chunksize=2))
    return [_job(j) for j in jobs]


# --------------------------------------------------------------------------- curb
def curb_sweep(args):
    """How tall a step can it actually climb? A full-width curb 0.6 m ahead, height swept upward."""
    from src.sim.terrain import CURB_AT

    steps = int(CURB_SECONDS / CONTROL_DT)
    cells = [("curb", h) for h in args.curb_heights]
    jobs = _make_jobs(args, cells, [("curb", CURB_CMD)], steps)
    print(f"curb sweep: {len(jobs)} episodes, cmd={CURB_CMD}, {CURB_SECONDS:.0f} s")
    rows = _run(jobs, args.workers)

    target = CURB_AT + CURB_CLEARED
    print(f"\n=== curb climbed (reached y > {target:.02f} m) ===")
    print(f"{'arm':34s} " + "  ".join(f"{int(h*1000):3d}mm" for h in args.curb_heights))
    for arm in dict.fromkeys(r[0] for r in rows):
        cells_out = []
        for h in args.curb_heights:
            sel = [r for r in rows if r[0] == arm and r[2] == h]
            cells_out.append(f"{sum(r[5]['dy'] > target for r in sel):3d}/{len(sel):<2d}")
        print(f"{arm:34s} " + "  ".join(cells_out))
    print(f"\n=== mean forward reach (m; edge at {CURB_AT:.02f}) ===")
    print(f"{'arm':34s} " + "  ".join(f"{int(h*1000):3d}mm" for h in args.curb_heights))
    for arm in dict.fromkeys(r[0] for r in rows):
        cells_out = []
        for h in args.curb_heights:
            sel = [r for r in rows if r[0] == arm and r[2] == h]
            cells_out.append(f"{_agg(sel, 'dy'):6.2f}")
        print(f"{arm:34s} " + "  ".join(cells_out))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", nargs="*", default=[], help="run dirs under --logdir to compare")
    ap.add_argument("--baseline", action="store_true", help="also evaluate the legacy analytic gait")
    ap.add_argument("--gait-library", default=None,
                    help="gait_library.json from optimize_gait_terrain.py -> tuned_oracle/tuned_single arms")
    ap.add_argument("--single-from", default=None,
                    help="library entry to use as tuned_single (default __single__)")
    ap.add_argument("--extra-gaits", nargs="*", default=[],
                    help="additional library entries to benchmark as their own open-loop arms")
    ap.add_argument("--logdir", default="./runs")
    ap.add_argument("--kinds", nargs="*", default=list(KINDS))
    ap.add_argument("--heights", nargs="*", type=float, default=list(HEIGHTS))
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--seed0", type=int, default=500)
    ap.add_argument("--randomize", action="store_true", help="also apply domain randomization + pushes")
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--json", default=None)
    ap.add_argument("--reflex", action="store_true",
                    help="also evaluate every arm with contact reflexes in the gait engine")
    ap.add_argument("--curb", action="store_true",
                    help="sweep a single full-width step instead of the rough-field matrix")
    ap.add_argument("--curb-heights", nargs="*", type=float, default=list(CURB_HEIGHTS))
    ap.add_argument("--held-out-commands", action="store_true",
                    help="shorthand for --command-set heldout")
    ap.add_argument("--command-set", choices=list(COMMAND_SETS), default=None,
                    help="'bench' = the 3 commands the gait search tunes on; 'wide' = the 7 "
                         "spanning the policy's training envelope; 'heldout' = 5 neither has seen")
    args = ap.parse_args()
    commands = COMMAND_SETS[args.command_set] if args.command_set \
        else (COMMANDS_HELDOUT if args.held_out_commands else COMMANDS)

    if args.curb:
        rows = curb_sweep(args)
    else:
        steps = int(EPISODE_S / CONTROL_DT)
        # at height 0 the ground is a flat plane, so the kind is meaningless -- run it once
        cells = [(kind, h) for h in args.heights
                 for kind in (args.kinds if h > 0 else args.kinds[:1])]
        jobs = _make_jobs(args, cells, commands, steps)
        print(f"{len(jobs)} episodes: {len(_arms(args))} arms x {len(cells)} terrain cells x "
              f"{len(commands)} commands x {args.seeds} seeds")
        rows = _run(jobs, args.workers)
        report(rows, args, commands)

    if args.json:
        with open(args.json, "w") as f:
            json.dump([{"arm": a, "kind": k, "height": h, "cmd": c, "seed": s, **m}
                       for a, k, h, c, s, m in rows], f, indent=1)
        print(f"wrote {args.json}")


def _agg(rows, key):
    return float(np.mean([r[5][key] for r in rows])) if rows else float("nan")


def report(rows, args, commands=COMMANDS):
    arms = list(dict.fromkeys(r[0] for r in rows))
    heights = sorted({r[2] for r in rows})

    print("\n=== score (tracking - stalling - plowing, scaled by fraction survived; 1.0 = perfect) ===")
    hdr = "  ".join(f"h={h:.02f}" for h in heights)
    print(f"{'arm':34s} {'kind':7s} {hdr}")
    for arm in arms:
        for kind in sorted({r[1] for r in rows}):
            cells = []
            for h in heights:
                sel = [r for r in rows if r[0] == arm and r[1] == kind and r[2] == h]
                cells.append(f"{_agg(sel, 'score'):6.2f}" if sel else f"{'--':>6s}")
            print(f"{arm:34s} {kind:7s} " + "  ".join(cells))

    print("\n=== progress (mean fraction of commanded speed achieved; 1.0 = perfect) ===")
    print(f"{'arm':34s} {'kind':7s} {hdr}")
    for arm in arms:
        for kind in sorted({r[1] for r in rows}):
            cells = []
            for h in heights:
                sel = [r for r in rows if r[0] == arm and r[1] == kind and r[2] == h]
                cells.append(f"{_agg(sel, 'progress'):6.2f}" if sel else f"{'--':>6s}")
            print(f"{arm:34s} {kind:7s} " + "  ".join(cells))

    print("\n=== per-arm summary (all kinds/commands pooled, by height) ===")
    print(f"{'arm':34s} {'h':>5s} {'score':>6s} {'prog':>6s} {'stuck':>6s} {'vel_err':>8s} "
          f"{'yaw_err':>8s} {'tilt~':>6s} {'tiltmx':>7s} {'rate':>5s} {'knock':>6s} {'falls':>7s}")
    for arm in arms:
        for h in heights:
            sel = [r for r in rows if r[0] == arm and r[2] == h]
            print(f"{arm:34s} {h:5.02f} {_agg(sel,'score'):6.2f} {_agg(sel,'progress'):6.2f} "
                  f"{_agg(sel,'stuck'):6.2f} {_agg(sel,'vel_err'):8.3f} {_agg(sel,'yaw_err'):8.3f} "
                  f"{_agg(sel,'tilt_deg'):6.1f} {_agg(sel,'tilt_max'):7.1f} {_agg(sel,'tilt_rate'):5.2f} "
                  f"{_agg(sel,'knock'):6.3f} {sum(r[5]['fell'] for r in sel):4.0f}/{len(sel):<3d}")

    print("\n=== progress by command (rows) x height (cols) ===")
    for arm in arms:
        print(f"{arm}:")
        for label, _ in commands:
            cells = []
            for h in heights:
                sel = [r for r in rows if r[0] == arm and r[2] == h and r[3] == label]
                cells.append(f"{_agg(sel, 'progress'):6.2f}" if sel else f"{'--':>6s}")
            print(f"   {label:10s} " + "  ".join(cells))

    print("\n=== gait adaptation (what the arm asked the gait engine for) ===")
    print(f"{'arm':34s} {'h':>5s} {'step_h_mm':>10s} {'cadence':>8s} {'duty':>6s} {'body_zm':>8s} "
          f"{'blend':>6s}")
    for arm in arms:
        for h in heights:
            sel = [r for r in rows if r[0] == arm and r[2] == h]
            print(f"{arm:34s} {h:5.02f} {_agg(sel,'step_h'):10.1f} {_agg(sel,'cadence'):8.2f} "
                  f"{_agg(sel,'duty'):6.3f} {_agg(sel,'body_zm'):8.1f} {_agg(sel,'gait_blend'):6.2f}")

    if len(arms) > 1:
        for base in dict.fromkeys([a for a in arms if a.startswith("tuned_oracle")] or arms[:1]):
            _paired(rows, arms, base, heights)


def _paired(rows, arms, base, heights):
    print(f"\n=== paired deltas vs {base} (identical terrain seed) ===")
    for arm in arms:
        if arm == base:
            continue
        print(f"{arm}:")
        for h in list(heights) + ["all"]:
            pairs = []
            for r in rows:
                if r[0] != arm or (h != "all" and r[2] != h):
                    continue
                match = [q for q in rows if q[0] == base and q[1:5] == r[1:5]]
                if match:
                    pairs.append((r[5], match[0][5]))
            if not pairs:
                continue
            ds = np.array([a["score"] - b["score"] for a, b in pairs])
            dp = np.mean([a["progress"] - b["progress"] for a, b in pairs])
            dt = np.mean([a["tilt_deg"] - b["tilt_deg"] for a, b in pairs])
            wins = int(np.sum(ds > 0))
            decided = int(np.sum(ds != 0))
            sign_p = binomtest(wins, decided, 0.5).pvalue if decided else 1.0
            try:
                w_p = wilcoxon(ds).pvalue if decided else 1.0
            except ValueError:
                w_p = 1.0
            hl = f"{h:.02f}" if h != "all" else " all"
            print(f"   h={hl}  dscore={ds.mean():+.3f}  dprogress={dp:+.3f}  dtilt={dt:+.2f}deg  "
                  f"wins={wins}/{len(pairs)}  sign p={sign_p:.4f}  wilcoxon p={w_p:.4f}")


if __name__ == "__main__":
    main()
