"""Train a hexapod walking policy in MuJoCo with SB3 PPO (PyTorch).

Stack: MuJoCo (C engine, CPU) env stepping in parallel via SubprocVecEnv; PPO policy/value
nets on GPU (your torch+cu118). The action space is selected by --control-mode:
  phase_gait    : 6-D gait settings + phase control (constrained, transfer-safe) [default]
  foot          : 18-D foot-position offsets (expressive, learns the whole gait)
  residual      : 6-D gait settings + 18-D foot residuals
  residual_pure : 18-D foot residuals on top of the analytic command->gait map (deploy target)

Examples:
  python train_mj.py --control-mode phase_gait --timesteps 5_000_000 --num-envs 16
  python train_mj.py --control-mode foot --randomize --num-envs 16
  python train_mj.py --control-mode residual_pure --randomize --zero-final --init-std 0.3
  python train_mj.py --smoke           # quick end-to-end sanity run
"""

import argparse
import os

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv, VecMonitor, VecNormalize
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, BaseCallback

from src.envs.hexapod_mj_env import make_env

_TERM_KEYS = ("r_vel", "r_yaw", "p_upright", "p_height", "p_vz", "p_energy", "p_power", "p_arate", "p_slip",
              "p_angvel", "p_res")


class Curriculum(BaseCallback):
    """Ramp env command difficulty 0->1 over `full_at` steps (forward first, then turn/strafe/back)."""

    def __init__(self, full_at):
        super().__init__()
        self.full_at = max(1, full_at)

    def _on_rollout_start(self):
        level = min(1.0, self.num_timesteps / self.full_at)
        self.training_env.env_method("set_curriculum", level)
        self.logger.record("curriculum/level", level)

    def _on_step(self):
        return True


class TermLogger(BaseCallback):
    """Log mean reward components + achieved speed to tensorboard so we can SEE whether
    the policy actually walks (r_vel high) vs games the reward by standing still."""

    def __init__(self, window=4000):
        super().__init__()
        self.window, self.acc, self.n = window, {}, 0

    def _on_step(self):
        for info in self.locals["infos"]:
            if "r_vel" not in info:
                continue
            for k in _TERM_KEYS:
                self.acc[k] = self.acc.get(k, 0.0) + info[k]
            self.acc["abs_bvx"] = self.acc.get("abs_bvx", 0.0) + abs(info.get("bvx", 0.0))
            self.acc["gait_blend"] = self.acc.get("gait_blend", 0.0) + info.get("gait_blend", 0.0)
            self.n += 1
        if self.n >= self.window:
            for k, v in self.acc.items():
                self.logger.record(f"terms/{k}", v / self.n)
            self.acc, self.n = {}, 0
        return True


def build_vecenv(mode, n, randomize, seed, subproc, vn_load=None, resample_steps=0, terrain=0.0):
    fns = [make_env(control_mode=mode, randomize=randomize, seed=seed + i, resample_steps=resample_steps,
                    terrain=terrain)
           for i in range(n)]
    venv = SubprocVecEnv(fns) if (subproc and n > 1) else DummyVecEnv(fns)
    venv = VecMonitor(venv)
    if vn_load and os.path.exists(vn_load):
        vn = VecNormalize.load(vn_load, venv)  # continue from prior normalization stats
        vn.training = True
        vn.norm_reward = True
        return vn
    return VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--control-mode", choices=["phase_gait", "foot", "residual", "residual_pure"],
                    default="phase_gait")
    ap.add_argument("--timesteps", type=int, default=5_000_000)
    ap.add_argument("--num-envs", type=int, default=16)
    ap.add_argument("--randomize", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--logdir", default="./runs")
    ap.add_argument("--no-subproc", action="store_true", help="use DummyVecEnv (single process)")
    ap.add_argument("--init-from", default=None, help="warm-start from a run dir (loads weights + vecnormalize)")
    ap.add_argument("--init-std", type=float, default=None,
                    help="set policy action std after warm-start (small e.g. 0.2 keeps RL near the BC policy)")
    ap.add_argument("--zero-final", action="store_true",
                    help="zero-init the actor output layer (residual modes: start exactly at the analytic gait)")
    ap.add_argument("--target-kl", type=float, default=None, help="PPO early-stop KL (e.g. 0.02)")
    ap.add_argument("--resample-steps", type=int, default=0, help="resample command every N control steps")
    ap.add_argument("--terrain", type=float, default=0.0,
                    help="max bump height (m) of per-episode random heightfield terrain (e.g. 0.02)")
    ap.add_argument("--curriculum", action="store_true", help="ramp command difficulty forward->omnidirectional")
    ap.add_argument("--tag", default=None, help="override output run-dir name")
    ap.add_argument("--smoke", action="store_true", help="tiny run to verify the pipeline")
    args = ap.parse_args()

    if args.smoke:
        args.timesteps, args.num_envs, args.no_subproc = 4000, 2, True

    tag = args.tag or f"{args.control_mode}{'_dr' if args.randomize else ''}"
    rundir = os.path.join(args.logdir, tag)
    os.makedirs(rundir, exist_ok=True)

    vn_load = os.path.join(args.init_from, "vecnormalize.pkl") if args.init_from else None
    train_env = build_vecenv(args.control_mode, args.num_envs, args.randomize, args.seed,
                             not args.no_subproc, vn_load=vn_load, resample_steps=args.resample_steps,
                             terrain=args.terrain)
    eval_env = build_vecenv(args.control_mode, 1, args.randomize, args.seed + 1000, False, vn_load=vn_load,
                            terrain=args.terrain)
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

    if args.init_from:
        model.set_parameters(os.path.join(args.init_from, "final_model.zip"))
        print(f"warm-started policy weights from {args.init_from}")
    if args.zero_final:
        import torch
        with torch.no_grad():
            model.policy.action_net.weight.data.zero_()
            model.policy.action_net.bias.data.zero_()
        print("zero-initialized actor output layer (deterministic action starts at 0)")
    if args.init_std is not None:
        import torch
        with torch.no_grad():
            model.policy.log_std.data.fill_(float(np.log(args.init_std)))
        print(f"set policy action std to {args.init_std} (stay near warm-start policy)")

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
        TermLogger(),
    ]
    if args.curriculum:
        callbacks.append(Curriculum(full_at=int(0.45 * args.timesteps)))

    model.learn(total_timesteps=args.timesteps, callback=callbacks, progress_bar=not args.smoke)

    model.save(os.path.join(rundir, "final_model"))
    train_env.save(os.path.join(rundir, "vecnormalize.pkl"))
    print(f"\nsaved model + vecnormalize stats to {rundir}")


if __name__ == "__main__":
    main()
