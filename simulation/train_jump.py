"""Train a hexapod JUMP policy in MuJoCo with SB3 PPO (sim-only demo).

Direct 18-joint position control (bypasses the walking gait engine). Two tasks:
  --env vertical : crouch, explode straight up, land upright (hexapod_jump_env).
  --env charge   : loaded directional leap -- charge/lean-back, then launch in a
                   commanded direction on release (hexapod_charge_jump_env).

  python train_jump.py --env vertical --timesteps 3_000_000 --num-envs 16 --tag jump_v2
  python train_jump.py --env charge   --timesteps 5_000_000 --num-envs 16 --tag jump_charge
  python train_jump.py --smoke        # quick end-to-end sanity run

The actor output layer is zero-initialized, so the deterministic policy starts
standing at the stand pose (a stable start); exploration comes from --init-std.
Watch terms/* in tensorboard (peak_air = apex m above stand; dist = leap m).
"""

import argparse
import os

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv, VecMonitor, VecNormalize
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, BaseCallback

from src.envs.hexapod_jump_env import make_jump_env
from src.envs.hexapod_charge_jump_env import make_charge_jump_env, make_body_jump_env

# "bodyjump": directional charge-jump driven by 7-D body kinematics (height/x/y/rpy/foot-spread)
# instead of 18 joints -- same phased reward + curriculum, much smaller action space.
ENV_FACTORY = {"vertical": make_jump_env, "charge": make_charge_jump_env, "bodyjump": make_body_jump_env}
# per-env scalar to surface as terms/peak (best achievement this window)
PEAK_KEY = {"vertical": "peak_air", "charge": "dist", "bodyjump": "dist"}


class Curriculum(BaseCallback):
    """Ramp charge-env difficulty 0->1 over `full_at` steps (vertical jump+land -> directional leap
    from a random start). Applied to the TRAIN env only; eval stays at full difficulty."""

    def __init__(self, full_at):
        super().__init__()
        self.full_at = max(1, full_at)

    def _on_rollout_start(self):
        level = min(1.0, self.num_timesteps / self.full_at)
        self.training_env.env_method("set_curriculum", level)
        self.logger.record("curriculum/level", level)

    def _on_step(self):
        return True


class JumpLogger(BaseCallback):
    """Log mean of every r_/p_ reward component + the best peak metric this window."""

    def __init__(self, peak_key, window=4000):
        super().__init__()
        self.peak_key = peak_key
        self.window, self.acc, self.peak, self.n = window, {}, 0.0, 0

    def _on_step(self):
        for info in self.locals["infos"]:
            keys = [k for k in info if k.startswith(("r_", "p_")) and np.isscalar(info[k])]
            if not keys:
                continue
            for k in keys:
                self.acc[k] = self.acc.get(k, 0.0) + info[k]
            self.peak = max(self.peak, info.get(self.peak_key, 0.0))
            self.n += 1
        if self.n >= self.window:
            for k, v in self.acc.items():
                self.logger.record(f"terms/{k}", v / self.n)
            self.logger.record(f"terms/{self.peak_key}", self.peak)  # best this window
            self.acc, self.peak, self.n = {}, 0.0, 0
        return True


def build_vecenv(env, n, seed, subproc, episode_seconds, randomize=False, vn_load=None):
    factory = ENV_FACTORY[env]
    extra = {} if env == "vertical" else {"randomize": randomize}  # vertical env has no DR
    fns = [factory(episode_seconds=episode_seconds, seed=seed + i, **extra) for i in range(n)]
    venv = SubprocVecEnv(fns) if (subproc and n > 1) else DummyVecEnv(fns)
    venv = VecMonitor(venv)
    if vn_load and os.path.exists(vn_load):  # continue obs/reward normalization from the warm-start run
        vn = VecNormalize.load(vn_load, venv)
        vn.training, vn.norm_reward = True, True
        return vn
    return VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", choices=["vertical", "charge", "bodyjump"], default="vertical")
    ap.add_argument("--timesteps", type=int, default=5_000_000)
    ap.add_argument("--num-envs", type=int, default=16)
    ap.add_argument("--episode-seconds", type=float, default=None,
                    help="episode length (s); default 2.0 vertical / 2.5 charge")
    ap.add_argument("--init-std", type=float, default=0.5,
                    help="initial action std (exploration around the standing start)")
    ap.add_argument("--target-kl", type=float, default=None,
                    help="PPO early-stop KL per update (e.g. 0.02) -- caps update size, prevents "
                         "catastrophic collapse from large sparse terminal rewards")
    ap.add_argument("--curriculum", action="store_true",
                    help="charge: ramp difficulty (vertical jump+land -> directional leap + random start)")
    ap.add_argument("--randomize", action="store_true",
                    help="charge/bodyjump: domain randomization (mass/friction/servo/latency/IMU/pushes)")
    ap.add_argument("--init-from", default=None,
                    help="warm-start from a run dir (loads weights + vecnormalize) and refine")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--logdir", default="./runs")
    ap.add_argument("--no-subproc", action="store_true")
    ap.add_argument("--tag", default="jump")
    ap.add_argument("--smoke", action="store_true")
    args = ap.parse_args()

    if args.smoke:
        args.timesteps, args.num_envs, args.no_subproc = 4000, 2, True
    if args.episode_seconds is None:
        args.episode_seconds = 2.0 if args.env == "vertical" else 3.0

    rundir = os.path.join(args.logdir, args.tag)
    os.makedirs(rundir, exist_ok=True)

    vn_load = os.path.join(args.init_from, "vecnormalize.pkl") if args.init_from else None
    train_env = build_vecenv(args.env, args.num_envs, args.seed, not args.no_subproc,
                             args.episode_seconds, randomize=args.randomize, vn_load=vn_load)
    eval_env = build_vecenv(args.env, 1, args.seed + 1000, False, args.episode_seconds)  # nominal eval
    eval_env.training = False
    eval_env.norm_reward = False

    model = PPO(
        "MlpPolicy",
        train_env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=4096 if args.num_envs >= 8 else 256,
        n_epochs=5,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,
        target_kl=args.target_kl,
        policy_kwargs=dict(net_arch=dict(pi=[256, 128, 64], vf=[256, 128, 64])),
        tensorboard_log=rundir,
        seed=args.seed,
        verbose=1,
    )

    import torch
    if args.init_from:  # warm-start: keep the already-learned jump, refine (skip zero-init)
        model.set_parameters(os.path.join(args.init_from, "final_model.zip"))
        with torch.no_grad():
            model.policy.log_std.data.fill_(float(np.log(args.init_std)))
        print(f"warm-started from {args.init_from}; action std reset to {args.init_std}")
    else:  # start standing: zero the actor output so the deterministic action is 0 (= stand pose)
        with torch.no_grad():
            model.policy.action_net.weight.data.zero_()
            model.policy.action_net.bias.data.zero_()
            model.policy.log_std.data.fill_(float(np.log(args.init_std)))
        print(f"zero-init actor; action std = {args.init_std} (starts standing, explores from there)")

    callbacks = [
        EvalCallback(
            eval_env,
            best_model_save_path=os.path.join(rundir, "best"),
            eval_freq=max(20000 // args.num_envs, 1),
            n_eval_episodes=5,
            deterministic=True,
        ),
        CheckpointCallback(
            save_freq=max(100000 // args.num_envs, 1),
            save_path=os.path.join(rundir, "ckpt"),
            name_prefix="ppo",
            save_vecnormalize=True,
        ),
        JumpLogger(PEAK_KEY[args.env]),
    ]
    if args.curriculum:
        callbacks.append(Curriculum(full_at=int(0.5 * args.timesteps)))

    model.learn(total_timesteps=args.timesteps, callback=callbacks, progress_bar=not args.smoke)

    model.save(os.path.join(rundir, "final_model"))
    train_env.save(os.path.join(rundir, "vecnormalize.pkl"))
    print(f"\nsaved model + vecnormalize stats to {rundir}")


if __name__ == "__main__":
    main()
