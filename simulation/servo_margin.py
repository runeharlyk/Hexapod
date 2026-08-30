"""How much servo margin does each gait have?

`servo_feasibility.py` shows the tuned gaits carry ~8x the joint tracking lag of the firmware gait
(6.2 deg mean vs 0.75 deg) and sit at the torque limit ~43% of the time. The sim already prices that
in -- the gaits were optimized through this servo model -- so lag by itself is not a verdict.

What decides portability is MARGIN: a real MG92B is weaker than nominal when it is hot, when the
battery sags, or when the gears are worn. A gait that only works at 100% of the modelled envelope
will not survive the robot. So sweep the stall-torque scale downward and watch the score.

Reported as the fraction of nominal score retained, so gaits at different absolute scores are
comparable, plus the scale at which each drops below 80% of its own nominal.

  python servo_margin.py --workers 14
"""

import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from src.robot.gait_schedule import GaitSchedule
from src.sim.mj_runtime import CONTROL_DT
from src.sim.rollout import COMMANDS_WIDE, EPISODE_S, ZERO_CFG, episode, score, zero_predict

LIB = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_library.json")
CELLS = [("bumps", 0.0), ("bumps", 0.08), ("rocks", 0.12)]
SCALES = (1.0, 0.9, 0.8, 0.7, 0.6, 0.5)
SEEDS = (700, 701, 702)


def _job(payload):
    key, params, run, scale = payload
    from src.envs.hexapod_mj_env import ACT_DIM, HexapodMjEnv, load_env_config, make_env

    if run is None:
        cfg = dict(ZERO_CFG)
        sched = GaitSchedule.from_dict(params) if params else None
        predict = zero_predict(ACT_DIM[cfg["control_mode"]])
    else:
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
        cfg = {"control_mode": "residual_gait"}
        cfg.update(load_env_config(os.path.join("runs", run)))
        sched = GaitSchedule.from_dict(cfg["gait_schedule"]) if cfg.get("gait_schedule") else None
        model = PPO.load(os.path.join("runs", run, "final_model.zip"), device="cpu",
                         custom_objects={"lr_schedule": lambda _: 0.0, "clip_range": lambda _: 0.2})
        vn = VecNormalize.load(os.path.join("runs", run, "vecnormalize.pkl"), DummyVecEnv([make_env(
            cfg["control_mode"], obs_contact=cfg["obs_contact"], obs_history=cfg["obs_history"],
            gait_schedule=sched)]))
        mu, var, clip, eps = vn.obs_rms.mean, vn.obs_rms.var, vn.clip_obs, vn.epsilon
        predict = (lambda o: model.predict(
            np.clip((o - mu) / np.sqrt(var + eps), -clip, clip).astype(np.float32),
            deterministic=True)[0])

    steps = int(EPISODE_S / CONTROL_DT)
    scores, falls = [], 0
    for kind, h in CELLS:
        for _, cmd in COMMANDS_WIDE:
            for sd in SEEDS:
                env = HexapodMjEnv(cfg["control_mode"], terrain=h, terrain_kind=kind,
                                   obs_contact=cfg["obs_contact"], obs_history=cfg["obs_history"],
                                   episode_seconds=EPISODE_S, gait_schedule=sched, seed=sd)
                env.fixed_command = np.asarray(cmd, dtype=np.float32)
                o, _ = env.reset(seed=sd)
                env.sim.set_servo_scale(stall=scale)
                fwd, lat, yaw, knock, n = [], [], [], [], 0
                for _ in range(steps):
                    o, _, term, _, info = env.step(predict(o))
                    fwd.append(info["bvx"]); lat.append(info["bvy"])
                    yaw.append(float(env.sim.data.qvel[5])); knock.append(info["knock"])
                    n += 1
                    if term:
                        falls += 1
                        break
                m = {"alive": n / steps,
                     "vel_err": float(np.hypot(np.mean(fwd) - cmd[0], np.mean(lat) - cmd[1])),
                     "yaw_err": float(abs(np.mean(yaw) - cmd[2])),
                     "stuck": float(np.mean(np.asarray(fwd) / cmd[0] < 0.3)) if abs(cmd[0]) > 1e-6 else 0.0,
                     "knock": float(np.mean(knock)), "tilt_rate": 0.0}
                scores.append(score(m))
    return key, scale, float(np.mean(scores)), falls, len(scores)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=14)
    ap.add_argument("--runs", nargs="*", default=["sched_v5"])
    ap.add_argument("--gaits", nargs="*", default=["__single___wide3", "__single__"])
    ap.add_argument("--scales", nargs="*", type=float, default=list(SCALES))
    args = ap.parse_args()

    lib = json.load(open(LIB))["gaits"]
    arms = [("firmware_gait", None, None)]
    arms += [(k, lib[k]["params"], None) for k in args.gaits if k in lib]
    arms += [(r, None, r) for r in args.runs]
    jobs = [(k, p, r, s) for (k, p, r) in arms for s in args.scales]

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        res = list(ex.map(_job, jobs))

    by = {}
    for key, scale, sc, falls, n in res:
        by.setdefault(key, {})[scale] = (sc, falls, n)

    print(f"stall-torque sweep, {len(CELLS)} terrains x {len(COMMANDS_WIDE)} commands x {len(SEEDS)} seeds")
    print(f"\n{'arm':20s} " + "  ".join(f"x{s:<5.2f}" for s in args.scales) + "   knee")
    for key in by:
        nom = by[key][args.scales[0]][0]
        cells, knee = [], "-"
        for s in args.scales:
            sc = by[key][s][0]
            cells.append(f"{sc / nom if nom else 0:6.2f}")
            if knee == "-" and nom and sc / nom < 0.80:
                knee = f"x{s:.2f}"
        print(f"{key:20s} " + "  ".join(cells) + f"   {knee}")
    print("\n(fraction of each arm's OWN nominal score retained; 'knee' = first scale below 0.80)")

    print(f"\n{'arm':20s} " + "  ".join(f"x{s:<5.2f}" for s in args.scales) + "   (absolute score)")
    for key in by:
        print(f"{key:20s} " + "  ".join(f"{by[key][s][0]:6.3f}" for s in args.scales))
    print(f"\n{'arm':20s} " + "  ".join(f"x{s:<5.2f}" for s in args.scales) + "   (falls)")
    for key in by:
        print(f"{key:20s} " + "  ".join(f"{by[key][s][1]:3d}/{by[key][s][2]:<2d}" for s in args.scales))


if __name__ == "__main__":
    main()
