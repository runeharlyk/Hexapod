"""MuJoCo Gymnasium env for a hexapod VERTICAL JUMP (sim-only demo).

Goal: a single in-place hop -- crouch, explosively extend all legs, reach maximum
apex height, land upright near the start. This is a different task from
`hexapod_mj_env` (velocity tracking through the gait engine): a jump is a
ballistic, fully-synchronized leg extension that the gait/Bezier engine cannot
express, so this env drives the 18 joints DIRECTLY.

Action (18-D, direct joint position targets):
  target = clip(stand_pose + action * ACTION_SCALE, joint_limits)
  Centered on the stand pose so a zero-init policy starts standing (a stable
  starting point, exactly like `--zero-final` for the walking policy), then learns
  to fold into a crouch (negative) and shoot to extension (positive).

Observation (privileged -- this is a sim demo, NOT a sim-to-real target, so it may
  read base height / vertical velocity that the real robot cannot sense):
  gravity in body (3) + gyro (3) + rpy (3) + base height (1) + base vz (1)
  + previous commanded joint angles (18) + normalized episode time (1)
  + previous action (18) = 48.

Reward (see `_reward_and_done`): dense takeoff shaping (upward velocity while a foot
  is still planted) + flight altitude (ONLY while airborne -- this is what blocks the
  trivial "stand as tall as possible" exploit) + a terminal peak-apex bonus, minus
  tilt / spin / horizontal-drift penalties. Crash-terminates on flip or body-to-ground.
"""

from __future__ import annotations

import numpy as np
import gymnasium as gym

from src.sim.mj_runtime import HexapodSim, CONTROL_DT
from src.envs.hexapod_mj_env import _quat_to_rpy, _gravity_in_body

STAND_Z = 0.066  # nominal body height (m); apex is measured above this

# Per-joint action authority (rad) around the stand pose: [coxa, femur, tibia] x6.
# Coxa is kept tight (a vertical hop needs almost no yaw sweep); femur/tibia get
# large authority so the policy can both fold into a deep crouch and fully extend.
ACTION_SCALE = np.tile([0.4, 1.4, 1.4], 6).astype(np.float32)

# --- reward weights ---
W_TAKEOFF = 0.5    # upward body velocity while grounded (shapes the push-off)
W_FLIGHT = 10.0    # altitude above stand while airborne (dense airtime x height)
W_PEAK = 200.0     # terminal bonus on the highest apex reached (the true objective)
W_UPRIGHT = 1.0    # keep the body level (clean hop + survivable landing)
W_SPIN = 0.01      # discourage tumbling in the air
W_DRIFT = 5.0      # "in place": penalize horizontal displacement from the start
W_SLIP = 1.0       # planted feet must not slide (grippy silicone feet -> no scrubbing)
ALIVE = 0.02
CRASH_PENALTY = 10.0

VZ_CLIP = 3.0  # cap the takeoff-velocity reward so it can't be farmed unboundedly


class HexapodJumpEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, episode_seconds: float = 2.0, seed: int | None = None):
        super().__init__()
        self.sim = HexapodSim()
        self.stand_pose = self.sim.stand_pose.astype(np.float32)
        self.jnt_lo, self.jnt_hi = self.sim.joint_limits
        self.max_steps = int(episode_seconds / CONTROL_DT)
        self.np_random_, _ = gym.utils.seeding.np_random(seed)

        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(18,), dtype=np.float32)
        obs_dim = 3 + 3 + 3 + 1 + 1 + 18 + 1 + 18
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)

        self.prev_action = np.zeros(18, dtype=np.float32)
        self.prev_joint_cmd = self.stand_pose.copy()
        self.peak_air = 0.0
        self.current_step = 0
        self._cur_action = np.zeros(18, dtype=np.float32)

    # ------------------------------------------------------------------ reset
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.np_random_, _ = gym.utils.seeding.np_random(seed)
        self.sim.reset_to_stand()
        self.prev_action[:] = 0.0
        self.prev_joint_cmd = self.stand_pose.copy()
        self.peak_air = 0.0
        self.current_step = 0
        self._cur_action[:] = 0.0
        return self._get_obs(), {}

    # ------------------------------------------------------------------ step
    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        self._cur_action = action
        joint_cmd = np.clip(self.stand_pose + action * ACTION_SCALE, self.jnt_lo, self.jnt_hi)
        self.sim.set_joint_targets(joint_cmd)
        self.sim.step_physics()
        self.current_step += 1

        obs = self._get_obs()
        reward, terminated, terms = self._reward_and_done()
        truncated = self.current_step >= self.max_steps
        if terminated or truncated:  # credit the highest apex on episode end
            bonus = W_PEAK * self.peak_air
            reward += bonus
            terms["r_peak"] = bonus

        self.prev_action = action
        self.prev_joint_cmd = joint_cmd.astype(np.float32)
        return obs, float(reward), bool(terminated), bool(truncated), terms

    # ------------------------------------------------------------------ obs
    def _get_obs(self):
        q = self.sim.base_quat()
        obs = np.concatenate([
            _gravity_in_body(q),
            self.sim.gyro(),
            _quat_to_rpy(q),
            [self.sim.base_height()],
            [self.sim.base_vz()],
            self.prev_joint_cmd,
            [self.current_step / self.max_steps],
            self.prev_action,
        ])
        return obs.astype(np.float32)

    # ------------------------------------------------------------------ reward
    def _reward_and_done(self):
        d = self.sim.data
        q = self.sim.base_quat()
        grav = _gravity_in_body(q)
        gyro = self.sim.gyro()
        h = self.sim.base_height()
        vz = self.sim.base_vz()
        airborne = not self.sim.feet_in_contact().any()
        if airborne:
            self.peak_air = max(self.peak_air, h - STAND_Z)

        r_takeoff = W_TAKEOFF * float(np.clip(vz, 0.0, VZ_CLIP)) * (0.0 if airborne else 1.0)
        r_flight = W_FLIGHT * max(0.0, h - STAND_Z) * (1.0 if airborne else 0.0)
        pen_upright = grav[0] ** 2 + grav[1] ** 2
        pen_spin = float(np.sum(gyro ** 2))
        pen_drift = d.qpos[0] ** 2 + d.qpos[1] ** 2
        pen_slip = self.sim.foot_slip_sq()  # horizontal speed of planted feet

        terms = {
            "r_takeoff": r_takeoff,
            "r_flight": r_flight,
            "p_upright": -W_UPRIGHT * pen_upright,
            "p_spin": -W_SPIN * pen_spin,
            "p_drift": -W_DRIFT * pen_drift,
            "p_slip": -W_SLIP * pen_slip,
            "alive": ALIVE,
            "height": h,
            "airborne": float(airborne),
            "peak_air": self.peak_air,
        }
        reward = sum(v for k, v in terms.items() if k.startswith(("r_", "p_", "alive")))

        terminated = bool(grav[2] > 0.0 or h < 0.012)  # flipped past horizontal, or slammed down
        if terminated:
            reward -= CRASH_PENALTY
        return reward, terminated, terms


def make_jump_env(episode_seconds=2.0, seed=0):
    """Factory for SubprocVecEnv (must be picklable / module-level)."""
    def _thunk():
        return HexapodJumpEnv(episode_seconds=episode_seconds, seed=seed)
    return _thunk
