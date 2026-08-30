"""Evaluate / watch a trained hexapod jump policy (vertical hop or directional charge).

  python eval_jump.py --env vertical --run jump_v2                 # viewer, loops
  python eval_jump.py --env vertical --run jump_v2 --headless      # apex metrics
  python eval_jump.py --env charge   --run jump_charge --headless  # leap-distance metrics
  python eval_jump.py --env charge   --run jump_charge --dir 0     # watch a fixed heading
  python eval_jump.py --env vertical --run jump_v2 --video jump.mp4
"""

import argparse
import os
import time
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from src.envs.hexapod_jump_env import HexapodJumpEnv, make_jump_env, STAND_Z
from src.envs.hexapod_charge_jump_env import (
    HexapodChargeJumpEnv, make_charge_jump_env, make_body_jump_env)
from src.sim.mj_runtime import CONTROL_DT


def _norm_base(args):
    if args.env == "vertical":
        return make_jump_env()
    return make_body_jump_env() if args.action_mode == "body" else make_charge_jump_env()


def make_env(args, seed):
    if args.env == "vertical":
        return HexapodJumpEnv(episode_seconds=args.episode_seconds, seed=seed)
    theta = None if args.dir is None else np.radians(args.dir)
    return HexapodChargeJumpEnv(episode_seconds=args.episode_seconds, seed=seed,
                                auto_release=True, direction=theta, action_mode=args.action_mode)


def load(args):
    rundir = os.path.join(args.logdir, args.run)
    model_path = args.model or os.path.join(rundir, "best", "best_model.zip")
    if not os.path.exists(model_path):
        model_path = os.path.join(rundir, "final_model.zip")
    vn_path = args.vecnorm or os.path.join(rundir, "vecnormalize.pkl")
    model = PPO.load(model_path, device="cpu",
                     custom_objects={"lr_schedule": lambda _: 0.0, "clip_range": lambda _: 0.2})
    vn = VecNormalize.load(vn_path, DummyVecEnv([_norm_base(args)]))
    mean, var, clip, eps = vn.obs_rms.mean, vn.obs_rms.var, vn.clip_obs, vn.epsilon
    norm = lambda o: np.clip((o - mean) / np.sqrt(var + eps), -clip, clip).astype(np.float32)
    return model, norm


def _run_episode(model, norm, env, on_step=None):
    """One episode; returns the env's peak metric (apex m, or leap m). on_step may abort."""
    o, _ = env.reset()
    apex = 0.0
    done = False
    while not done:
        a, _ = model.predict(norm(o), deterministic=True)
        o, _, term, trunc, _ = env.step(a)
        apex = max(apex, env.sim.base_height() - STAND_Z)
        if on_step and not on_step():
            break
        done = term or trunc
    return getattr(env, "peak_dist", apex)  # charge -> leap distance; vertical -> apex


def run_headless(args):
    model, norm = load(args)
    unit = "leap" if args.env == "charge" else "apex"
    vals = []
    for i in range(args.episodes):
        env = make_env(args, args.seed + i)
        vals.append(_run_episode(model, norm, env))
        print(f"episode {i}: {unit} = {vals[-1]*1000:6.1f} mm")
    vals = np.array(vals)
    print(f"\n{unit} over {len(vals)} eps: mean {vals.mean()*1000:.1f} mm  "
          f"max {vals.max()*1000:.1f} mm  min {vals.min()*1000:.1f} mm")


def run_viewer(args):
    import mujoco.viewer

    model, norm = load(args)
    env = make_env(args, args.seed)
    unit = "leap" if args.env == "charge" else "apex"
    with mujoco.viewer.launch_passive(env.sim.model, env.sim.data) as viewer:
        def on_step():
            t0 = time.time()
            viewer.sync()
            dt = CONTROL_DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
            return viewer.is_running()

        while viewer.is_running():
            val = _run_episode(model, norm, env, on_step)
            print(f"{unit} = {val * 1000:.1f} mm")


def run_video(args):
    import mediapy
    import mujoco

    model, norm = load(args)
    env = make_env(args, args.seed)
    renderer = mujoco.Renderer(env.sim.model, height=480, width=640)
    frames = []

    def on_step():
        renderer.update_scene(env.sim.data, camera=-1)
        frames.append(renderer.render())
        return True

    val = _run_episode(model, norm, env, on_step)
    mediapy.write_video(args.video, frames, fps=int(1 / CONTROL_DT))
    print(f"wrote {args.video} ({len(frames)} frames), {'leap' if args.env=='charge' else 'apex'} = {val*1000:.1f} mm")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", choices=["vertical", "charge"], default="vertical")
    ap.add_argument("--run", default="jump_v2")
    ap.add_argument("--model", default=None)
    ap.add_argument("--vecnorm", default=None)
    ap.add_argument("--logdir", default="./runs")
    ap.add_argument("--episode-seconds", type=float, default=None)
    ap.add_argument("--dir", type=float, default=None, help="charge: fixed heading (deg); default random")
    ap.add_argument("--action-mode", choices=["joint", "body"], default="joint",
                    help="charge: action space of the policy ('body' for jump_body*, 'joint' for jump_charge*)")
    ap.add_argument("--seed", type=int, default=123)
    ap.add_argument("--episodes", type=int, default=10, help="headless: episodes to average")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--video", default=None)
    args = ap.parse_args()

    if args.episode_seconds is None:
        args.episode_seconds = 2.0 if args.env == "vertical" else 3.0

    if args.video:
        run_video(args)
    elif args.headless:
        run_headless(args)
    else:
        run_viewer(args)
