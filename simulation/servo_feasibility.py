"""Can the real servos actually run these gaits?

Everything in the gait study was optimized inside the sim's MG92B model, and CMA-ES will happily
spend whatever torque and joint speed that model offers. Before any of it is worth porting to the
firmware, the winning gaits have to be checked against the envelope the hardware really has:

  torque      fraction of the 0.50 N.m stall envelope used, and the fraction of samples AT the limit
  speed       joint rate as a fraction of the 16 rad/s no-load cap
  tracking    commanded joint angle minus achieved -- the direct readout of "the servo could not
              keep up", which is what actually breaks sim-to-real for an open-loop machine

A gait that spends much of its cycle pinned at stall or at no-load speed is a sim artifact: on
hardware it will lag, overheat, and stop reproducing the trajectory the search assumed.

  python servo_feasibility.py --workers 12
"""

import argparse
import json
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from src.envs.hexapod_mj_env import HexapodMjEnv
from src.robot.gait_schedule import GaitSchedule
from src.sim.mj_runtime import SERVO_STALL, SERVO_NOLOAD, CONTROL_DT
from src.sim.rollout import COMMANDS_WIDE, EPISODE_S

LIB = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_library.json")
CELLS = [("bumps", 0.0), ("bumps", 0.08), ("rocks", 0.12)]
NEAR_LIMIT = 0.95   # fraction of the envelope that counts as "pinned"


def _job(payload):
    key, params, run = payload[:3]
    stall_scale = payload[3] if len(payload) > 3 else 1.0
    steps = int(EPISODE_S / CONTROL_DT)
    tau, vel, err = [], [], []

    predict, cfg, sched = _controller(params, run)
    for kind, h in CELLS:
        for _, cmd in COMMANDS_WIDE:
            env = HexapodMjEnv(cfg["control_mode"], terrain=h, terrain_kind=kind,
                               obs_contact=cfg["obs_contact"], obs_history=cfg["obs_history"],
                               episode_seconds=EPISODE_S, gait_schedule=sched, seed=700)
            env.fixed_command = np.asarray(cmd, dtype=np.float32)
            o, _ = env.reset(seed=700)
            if stall_scale != 1.0:  # reset() re-applies nominal scaling, so weaken AFTER it
                env.sim.set_servo_scale(stall=stall_scale)
            for _ in range(steps):
                o, _, term, _, _ = env.step(predict(o))
                sim = env.sim
                d = sim.data
                tau.append(np.abs(d.actuator_force).copy())
                vel.append(np.abs(d.qvel[sim.qvel_adr]).copy())
                # Commanded vs achieved joint angle, BOTH in radians (set_joint_targets takes rad,
                # and joint_target is post-latency, i.e. what the servo was actually asked for).
                # This is the servo lag an open-loop robot inherits directly as a gait error.
                err.append(np.rad2deg(np.abs(sim.joint_target - d.qpos[sim.qpos_adr])))
                if term:
                    break
    tau, vel, err = np.asarray(tau), np.asarray(vel), np.asarray(err)
    return key, {
        "stall_scale": stall_scale,
        "tau_mean_frac": float(tau.mean() / SERVO_STALL),
        "tau_p99_frac": float(np.percentile(tau, 99) / SERVO_STALL),
        "tau_pinned_pct": float(100.0 * (tau > NEAR_LIMIT * SERVO_STALL).mean()),
        "vel_mean_frac": float(vel.mean() / SERVO_NOLOAD),
        "vel_p99_frac": float(np.percentile(vel, 99) / SERVO_NOLOAD),
        "vel_pinned_pct": float(100.0 * (vel > NEAR_LIMIT * SERVO_NOLOAD).mean()),
        "track_err_mean_deg": float(err.mean()),
        "track_err_p99_deg": float(np.percentile(err, 99)),
    }


def _controller(params, run):
    from src.envs.hexapod_mj_env import ACT_DIM, load_env_config, make_env
    if run is None:
        cfg = {"control_mode": "residual_gait", "obs_contact": False, "obs_history": 1}
        z = np.zeros(ACT_DIM[cfg["control_mode"]], dtype=np.float32)
        sched = GaitSchedule.from_dict(params) if params else None
        return (lambda o: z), cfg, sched
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
    return (lambda o: model.predict(np.clip((o - mu) / np.sqrt(var + eps), -clip, clip)
                                    .astype(np.float32), deterministic=True)[0]), cfg, sched


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--runs", nargs="*", default=["sched_v5"])
    ap.add_argument("--gaits", nargs="*",
                    default=["__single___wide3", "__single__", "flat_tripod", "curb_140"])
    args = ap.parse_args()

    lib = json.load(open(LIB))["gaits"]
    jobs = [("firmware_gait", None, None)]
    jobs += [(k, lib[k]["params"], None) for k in args.gaits if k in lib]
    jobs += [(r, None, r) for r in args.runs]

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        results = list(ex.map(_job, jobs))

    print(f"servo envelope: stall {SERVO_STALL} N.m, no-load {SERVO_NOLOAD} rad/s; "
          f"{len(CELLS)} terrains x {len(COMMANDS_WIDE)} commands")
    print(f"\n{'arm':20s} {'tau~':>6s} {'tau99':>6s} {'tau@lim':>8s} {'vel~':>6s} {'vel99':>6s} "
          f"{'vel@lim':>8s} {'lag~deg':>8s} {'lag99deg':>9s}")
    for key, m in results:
        print(f"{key:20s} {m['tau_mean_frac']:6.2f} {m['tau_p99_frac']:6.2f} "
              f"{m['tau_pinned_pct']:7.1f}% {m['vel_mean_frac']:6.2f} {m['vel_p99_frac']:6.2f} "
              f"{m['vel_pinned_pct']:7.1f}% {m['track_err_mean_deg']:8.2f} {m['track_err_p99_deg']:9.2f}")
    print("\ntau/vel are fractions of the envelope; '@lim' is the % of samples above 95% of it.")


if __name__ == "__main__":
    main()
