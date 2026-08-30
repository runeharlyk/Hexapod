"""Train a hexapod walking policy in MuJoCo with SB3 PPO (PyTorch).

Action space is selected by --control-mode:
  phase_gait    : 6-D gait settings + phase control (constrained, transfer-safe) [default]
  foot          : 18-D foot-position offsets (expressive, learns the whole gait)
  residual      : 6-D gait settings + 18-D foot residuals
  residual_pure : 18-D foot residuals on top of the analytic command->gait map (deploy target)
  residual_gait : analytic base + policy adjusts stride/height/blend/cadence (5) + 18 foot residuals
                  (zero action = analytic gait)

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

from src.envs.hexapod_mj_env import make_env, save_env_config

_TERM_KEYS = ("r_vel", "r_yaw", "p_upright", "p_height", "p_vz", "p_energy", "p_power", "p_arate", "p_slip",
              "p_angvel", "p_res", "p_knock")


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


class TerrainCurriculum(BaseCallback):
    """Ramp terrain roughness 0 -> max_height over `full_at` steps (learn on flat, then rougher)."""

    def __init__(self, max_height, full_at):
        super().__init__()
        self.max_height = max_height
        self.full_at = max(1, full_at)

    def _on_rollout_start(self):
        h = self.max_height * min(1.0, self.num_timesteps / self.full_at)
        self.training_env.env_method("set_terrain", h)
        self.logger.record("curriculum/terrain_m", h)

    def _on_step(self):
        return True


class AdaptiveTerrain(BaseCallback):
    """Raise terrain roughness only while the robot is actually coping, and back off when it is not.

    A fixed ramp to a chosen maximum spends its final stretch on ground that may be physically
    impassable (a sharp edge taller than the leg can lift), which teaches stalling rather than
    skill. Promotion is gated on measured competence instead, so difficulty settles at the edge of
    the robot's real capability.

    Two conditions, both required to promote. Tracking quality alone is not enough: commands are
    themselves scaled down on rough ground (see TERRAIN_SPEED_*), so a robot crawling at 0.06 m/s
    can still score well against a 0.1 m/s command and pin the curriculum at its ceiling. The
    absolute-speed floor is what keeps "coping" from meaning "barely moving".
    """

    def __init__(self, max_height, step=0.005, up=0.60, down=0.40, speed_floor=0.10, window=6000):
        super().__init__()
        self.max_height, self.step = max_height, step
        self.up, self.down, self.window = up, down, window
        self.speed_floor = speed_floor
        self.height, self.acc, self.speed, self.n = 0.0, 0.0, 0.0, 0

    def _on_step(self):
        # `track_vel` is the unweighted velocity kernel. The weighted `r_vel` term is not usable
        # here: --reward-vel changes its scale, and --reward-mode score folds the yaw kernel into
        # it as a product, which is systematically smaller and pins the curriculum near flat.
        for info in self.locals["infos"]:
            if "track_vel" in info:
                self.acc += info["track_vel"]
                self.speed += abs(info.get("bvx", 0.0))
                self.n += 1
        if self.n >= self.window:
            score, speed = self.acc / self.n, self.speed / self.n
            if score > self.up and speed > self.speed_floor:
                self.height = min(self.max_height, self.height + self.step)
            elif score < self.down or speed < 0.5 * self.speed_floor:
                self.height = max(0.0, self.height - self.step)
            self.training_env.env_method("set_terrain", self.height)
            self.logger.record("curriculum/terrain_m", self.height)
            self.logger.record("curriculum/track_score", score)
            self.logger.record("curriculum/mean_speed", speed)
            self.acc, self.speed, self.n = 0.0, 0.0, 0
        return True


class TermLogger(BaseCallback):
    """Log mean reward components + achieved speed so walking (high r_vel) is distinguishable
    from gaming the reward by standing still."""

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
            self.acc["knock"] = self.acc.get("knock", 0.0) + info.get("knock", 0.0)
            self.acc["contacts"] = self.acc.get("contacts", 0.0) + info.get("contacts", 0.0)
            self.n += 1
        if self.n >= self.window:
            for k, v in self.acc.items():
                self.logger.record(f"terms/{k}", v / self.n)
            self.acc, self.n = {}, 0
        return True


def build_vecenv(mode, n, randomize, seed, subproc, vn_load=None, resample_steps=0, terrain=0.0,
                 terrain_kind="bumps", obs_contact=False, obs_history=1, gait_schedule=None,
                 terrain_speed_floor=1.0, reward_weights=None, reward_mode="shaped"):
    fns = [make_env(control_mode=mode, randomize=randomize, seed=seed + i, resample_steps=resample_steps,
                    terrain=terrain, terrain_kind=terrain_kind, obs_contact=obs_contact,
                    obs_history=obs_history, gait_schedule=gait_schedule,
                    terrain_speed_floor=terrain_speed_floor, reward_weights=reward_weights,
                    reward_mode=reward_mode)
           for i in range(n)]
    venv = SubprocVecEnv(fns) if (subproc and n > 1) else DummyVecEnv(fns)
    venv = VecMonitor(venv)
    if vn_load and os.path.exists(vn_load):
        vn = VecNormalize.load(vn_load, venv)
        vn.training = True
        vn.norm_reward = True
        return vn
    return VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--control-mode",
                    choices=["phase_gait", "foot", "residual", "residual_pure", "residual_gait",
                             "residual_sched"],
                    default="phase_gait")
    ap.add_argument("--gait-from", default=None,
                    help="gait_library.json: train on top of a searched base gait instead of the "
                         "legacy analytic map, so zero action = that gait")
    ap.add_argument("--gait-key", default="__single__",
                    help="which library entry --gait-from should use")
    ap.add_argument("--ent-coef", type=float, default=0.005,
                    help="PPO entropy bonus. MEASURED: on a strong base gait the reward landscape "
                         "is flat (see reward_alignment.py), the bonus outweighs the policy "
                         "gradient, action std GROWS from 0.3 to 0.51 and the deterministic mean "
                         "drifts to a much worse point than the stochastic policy (0.23 vs 0.49). "
                         "Use 0 for residual modes on a tuned base gait.")
    ap.add_argument("--reward-mode", choices=["shaped", "score"], default="shaped",
                    help="'score' replaces the shaped reward with a per-step analogue of "
                         "rollout.score: multiplied tracking kernels plus only the stall and knock "
                         "penalties the evaluation objective actually contains")
    ap.add_argument("--reward-vel", type=float, default=None, help="override the velocity-tracking weight")
    ap.add_argument("--reward-angvel", type=float, default=None, help="override the body-rate penalty")
    ap.add_argument("--reward-knock", type=float, default=None, help="override the shin/belly penalty")
    ap.add_argument("--terrain-speed-floor", type=float, default=1.0,
                    help="fraction of the flat-ground command range that survives at "
                         "TERRAIN_SPEED_REF roughness; 1.0 = no throttle (default)")
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
    ap.add_argument("--terrain-curriculum", action="store_true",
                    help="ramp terrain roughness 0 -> --terrain over training (learn flat first)")
    ap.add_argument("--terrain-adaptive", action="store_true",
                    help="promote/demote terrain roughness by tracking quality instead of a fixed ramp")
    ap.add_argument("--terrain-kind", default="bumps",
                    choices=["bumps", "rocks", "steps", "waves", "mixed"],
                    help="heightfield type; 'mixed' samples a kind per episode (rough-terrain recipe)")
    ap.add_argument("--contact-obs", action="store_true",
                    help="include 6 per-foot contact sensors in the observation (needs foot switches/FSRs)")
    ap.add_argument("--obs-history", type=int, default=1,
                    help="stacked sensor frames (gravity+gyro[+contact]) spaced 3 control steps apart")
    ap.add_argument("--curriculum", action="store_true", help="ramp command difficulty forward->omnidirectional")
    ap.add_argument("--tag", default=None, help="override output run-dir name")
    ap.add_argument("--smoke", action="store_true", help="tiny run to verify the pipeline")
    args = ap.parse_args()

    if args.smoke:
        args.timesteps, args.num_envs, args.no_subproc = 4000, 2, True

    tag = args.tag or f"{args.control_mode}{'_dr' if args.randomize else ''}"
    rundir = os.path.join(args.logdir, tag)
    os.makedirs(rundir, exist_ok=True)

    gait_schedule = None
    if args.gait_from:
        import json
        from src.robot.gait_schedule import GaitSchedule
        entry = json.load(open(args.gait_from))["gaits"][args.gait_key]
        gait_schedule = GaitSchedule.from_dict(entry["params"])
        print(f"base gait = {args.gait_from}:{args.gait_key} "
              f"(duty={gait_schedule.duty:.3f}, offsets={np.round(gait_schedule.offsets(),3)})")

    save_env_config(rundir, control_mode=args.control_mode, obs_contact=args.contact_obs,
                    obs_history=args.obs_history, gait_schedule=gait_schedule)

    rw = {k: v for k, v in (("vel", args.reward_vel), ("angvel", args.reward_angvel),
                            ("knock", args.reward_knock)) if v is not None}
    if rw:
        print(f"reward weight override: {rw}")

    obs_kw = dict(terrain_kind=args.terrain_kind, obs_contact=args.contact_obs,
                  obs_history=args.obs_history, gait_schedule=gait_schedule,
                  terrain_speed_floor=args.terrain_speed_floor,
                  reward_weights=rw or None, reward_mode=args.reward_mode)
    vn_load = os.path.join(args.init_from, "vecnormalize.pkl") if args.init_from else None
    train_env = build_vecenv(args.control_mode, args.num_envs, args.randomize, args.seed,
                             not args.no_subproc, vn_load=vn_load, resample_steps=args.resample_steps,
                             terrain=args.terrain, **obs_kw)
    eval_env = build_vecenv(args.control_mode, 1, args.randomize, args.seed + 1000, False, vn_load=vn_load,
                            terrain=args.terrain, **obs_kw)
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
        ent_coef=args.ent_coef,
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
    assert not (args.terrain_curriculum and args.terrain_adaptive), \
        "--terrain-curriculum and --terrain-adaptive both drive terrain height; pick one"
    if args.terrain_curriculum:
        callbacks.append(TerrainCurriculum(args.terrain, full_at=int(0.6 * args.timesteps)))
    if args.terrain_adaptive:
        callbacks.append(AdaptiveTerrain(args.terrain))

    model.learn(total_timesteps=args.timesteps, callback=callbacks, progress_bar=not args.smoke)

    model.save(os.path.join(rundir, "final_model"))
    train_env.save(os.path.join(rundir, "vecnormalize.pkl"))
    print(f"\nsaved model + vecnormalize stats to {rundir}")


if __name__ == "__main__":
    main()
