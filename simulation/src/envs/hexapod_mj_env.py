"""MuJoCo Gymnasium env for hexapod locomotion, designed for sim-to-real.

Command: body-frame velocity vector [vx, vy] (m/s) + yaw rate (rad/s) -- what the reward tracks.

Control modes (action space):
  - "foot":       18-D per-leg foot-position offsets (m) from the default stance -> IK -> joints.
                  Full per-leg authority; the policy learns the whole gait.
  - "phase_gait": 6-D = [step_x, step_z, step_angle, step_height, gait_blend, phase_rate].
                  Policy drives the Bezier gait engine + advances phase; gait_blend morphs
                  tripod(0)<->bipod(1). Low-dim, constrained, transfer-safe.
  - "residual":   24-D = phase_gait (6) + per-leg foot XYZ residuals (18) added on top of the
                  gait before IK (spot_mini_mini "D2" style). Keeps the gait structure but adds
                  corrective freedom for balance / disturbance rejection / terrain. BC-init at 0 residual.

Observation (hardware-available ONLY -- open-loop servos have no encoders):
  gravity vector in body frame (3) + gyro (3) + rpy (3) + previous commanded joint angles (18)
  + gait phase clock [sin, cos] (2) + command (3) + previous action (act_dim).
Deliberately excludes base position, base linear velocity, measured joint pos/vel (unobservable
on the real robot). Those are used for REWARD only.
"""

from __future__ import annotations

import json
import os
from collections import deque
import numpy as np
import gymnasium as gym
import mujoco

from src.sim.mj_runtime import HexapodSim, CONTROL_DT, TERRAIN_MODEL_PATH
from src.sim.domain_rand import DomainRandomizer
from src.sim.terrain import randomize_hfield
from src.robot.firmware_gait import (
    GaitController,
    GaitState,
    BodyState,
    set_gait,
    DEFAULT_FEET,
    TRI_OFFSET,
    TRI_STAND_FRAC,
    BI_OFFSET,
    BI_STAND_FRAC,
)

ACT_DIM = {"foot": 18, "phase_gait": 6, "residual": 24, "residual_pure": 18}

# --- action scaling ---
FOOT_XY_RANGE = 0.060  # m, foot offset authority (foot mode)
FOOT_Z_RANGE = 0.050   # m
FOOT_RESIDUAL = 0.020  # m, per-leg residual authority on top of the gait (residual mode)
# phase_gait scales: [step_x(mm), step_z(mm), step_angle(rad), step_height(mm), stand_frac, phase_rate(/s)]
PG_STEP_XY = 100.0
PG_STEP_ANGLE = 0.8
PG_HEIGHT = (10.0, 50.0)
PG_STAND_FRAC = (0.35, 0.85)
PG_PHASE_RATE = (0.0, 2.5)

# --- command sampling ranges ---
# Robot's measured top speed (bipod, high cadence): ~0.59 m/s fwd, ~0.66 lateral, ~3.7 rad/s yaw.
# Command up to a margin below that so the policy actually uses the speed.
CMD_VX = (-0.30, 0.45)
CMD_VY = (-0.25, 0.25)
CMD_YAW = (-1.5, 1.5)
ZERO_CMD_PROB = 0.05

# velocity-tracking kernel widths, scaled to the command range (~0.5 * max, ANYmal-style)
VEL_SIGMA = 0.04
YAW_SIGMA = 0.30

STAND_Z = 0.066  # target body height (m)

# Analytic command -> gait-params map (deterministic; used by residual_pure mode and BC).
# Coefficients live in GAIT_COEF so they can be tuned by optimize_gait.py (DE/CMA search) and
# persisted to resources/gait_coef.json. gx/gy/gyaw are velocity gains at tripod(0)/bipod(1);
# yaw_comp adds a velocity-proportional yaw correction (cancels the gait's backward yaw drift).
GAIT_COEF = {
    "gx0": 0.259, "gx1": 0.346, "gy0": 0.294, "gy1": 0.360, "gyaw0": 1.602, "gyaw1": 2.153,
    "blend_speed": 0.30, "pr_base": 0.2, "pr_slope": 2.0, "step_height": -0.5, "yaw_comp": 0.0,
    "step_depth": 0.002,  # mm; stance-phase downward push (traction). ~0 = off.
}
_coef_file = os.path.join(os.path.dirname(__file__), "..", "resources", "gait_coef.json")
if os.path.exists(_coef_file):
    try:
        GAIT_COEF.update(json.load(open(_coef_file)))
    except Exception:
        pass


def analytic_gait_action(cmd):
    """Deterministic command -> 6 gait-param actions (tripod->bipod with speed). The robot's
    open-loop gait, identical in spirit to what the firmware computes from a CommandMsg."""
    c = GAIT_COEF
    vx, vy, yaw = float(cmd[0]), float(cmd[1]), float(cmd[2])
    speed = np.hypot(vx, vy)
    b = float(np.clip(speed / c["blend_speed"], 0.0, 1.0))
    gx = c["gx0"] + (c["gx1"] - c["gx0"]) * b
    gy = c["gy0"] + (c["gy1"] - c["gy0"]) * b
    gyaw = c["gyaw0"] + (c["gyaw1"] - c["gyaw0"]) * b
    step_angle = np.clip(yaw / gyaw + c["yaw_comp"] * vx, -1, 1)
    phase_rate = np.clip(c["pr_base"] + c["pr_slope"] * speed, -1, 1)  # cadence rises with speed
    return np.array([np.clip(vx / gx, -1, 1), np.clip(vy / gy, -1, 1),
                     step_angle, c["step_height"], b * 2 - 1, phase_rate], dtype=np.float32)


def _quat_to_rpy(q):
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def _gravity_in_body(q):
    """World down [0,0,-1] expressed in body frame (= what an accelerometer-free IMU infers)."""
    grav = np.zeros(3)
    conj = np.array([q[0], -q[1], -q[2], -q[3]])
    mujoco.mju_rotVecQuat(grav, np.array([0.0, 0.0, -1.0]), conj)
    return grav


class HexapodMjEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, control_mode: str = "phase_gait", randomize: bool = False,
                 episode_seconds: float = 20.0, seed: int | None = None,
                 command: tuple | None = None, resample_steps: int = 0,
                 terrain: float = 0.0):
        super().__init__()
        assert control_mode in ("foot", "phase_gait", "residual", "residual_pure")
        self.control_mode = control_mode
        self.randomize = randomize
        self.terrain = terrain  # >0: max bump height (m), per-episode random heightfield
        self.fixed_command = None if command is None else np.asarray(command, dtype=np.float32)
        self.curriculum = 1.0  # 0 = forward only, 1 = full command range (training ramps this)
        self.gait_blend = 0.0  # last applied tripod(0)<->bipod(1) blend (for logging)
        self.resample_steps = resample_steps  # >0: resample command mid-episode (ANYmal-style)
        self.max_steps = int(episode_seconds / CONTROL_DT)

        self.sim = HexapodSim(TERRAIN_MODEL_PATH) if terrain > 0 else HexapodSim()
        self.gc = GaitController()
        self.body = BodyState()
        self.np_random_, _ = gym.utils.seeding.np_random(seed)
        self.dr = DomainRandomizer(self.sim.model) if randomize else None
        self._cmd_buffer: deque = deque(maxlen=8)

        self.act_dim = ACT_DIM[control_mode]
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(self.act_dim,), dtype=np.float32)

        self.prev_action = np.zeros(self.act_dim, dtype=np.float32)
        self.prev_joint_cmd = self.sim.stand_pose.astype(np.float32)
        self.cmd = np.zeros(3, dtype=np.float32)
        self.gait_phase = 0.0

        obs_dim = 3 + 3 + 3 + 18 + 2 + 3 + self.act_dim
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)
        self.current_step = 0

    # ------------------------------------------------------------------ reset
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.np_random_, _ = gym.utils.seeding.np_random(seed)
        if self.randomize:
            self.dr.reset_episode(self.sim.model, self.np_random_)
        if self.terrain > 0:
            randomize_hfield(self.sim.model, self.np_random_, self.terrain)
        self.sim.reset_to_stand()
        self.gc = GaitController()
        self.body = BodyState()
        self.gait_phase = 0.0
        self.prev_action[:] = 0.0
        self.prev_joint_cmd = self.sim.stand_pose.astype(np.float32)
        self._cmd_buffer.clear()
        self._sample_command()
        self.current_step = 0
        return self._get_obs(), {}

    def set_curriculum(self, level):
        self.curriculum = float(np.clip(level, 0.0, 1.0))

    def _sample_command(self):
        if self.fixed_command is not None:
            self.cmd[:] = self.fixed_command
            return
        r = self.np_random_
        L = self.curriculum
        if r.random() < ZERO_CMD_PROB:
            self.cmd[:] = 0.0
        else:
            # forward is always available; backward, lateral and yaw phase in with curriculum L
            self.cmd[0] = r.uniform(CMD_VX[0] * L, CMD_VX[1])
            self.cmd[1] = r.uniform(CMD_VY[0] * L, CMD_VY[1] * L)
            self.cmd[2] = r.uniform(CMD_YAW[0] * L, CMD_YAW[1] * L)

    # ------------------------------------------------------------------ step
    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        self._cur_action = action
        joint_cmd = self._action_to_joints(action)

        # action latency (DR): apply a delayed command to the servos
        self._cmd_buffer.append(joint_cmd)
        if self.randomize and self.dr.action_latency_steps > 0:
            idx = max(0, len(self._cmd_buffer) - 1 - self.dr.action_latency_steps)
            effective = self._cmd_buffer[idx]
        else:
            effective = joint_cmd
        self.sim.set_joint_targets(effective)

        if self.randomize:
            self.dr.maybe_push(self.sim.model, self.sim.data, self.np_random_, self.current_step)
        self.sim.step_physics()

        obs = self._get_obs()
        reward, terminated, terms = self._reward_and_done()
        self.current_step += 1
        if self.resample_steps and self.fixed_command is None and self.current_step % self.resample_steps == 0:
            self._sample_command()  # mid-episode command change -> more diversity, learns transitions
        truncated = self.current_step >= self.max_steps

        self.prev_action = action
        self.prev_joint_cmd = joint_cmd.astype(np.float32)
        return obs, float(reward), bool(terminated), bool(truncated), terms

    def _action_to_joints(self, action):
        if self.control_mode == "foot":
            feet = DEFAULT_FEET.copy()
            off = action.reshape(6, 3)
            feet[:, 0] += off[:, 0] * FOOT_XY_RANGE * 1000.0  # m->mm (firmware units)
            feet[:, 1] += off[:, 1] * FOOT_XY_RANGE * 1000.0
            feet[:, 2] += off[:, 2] * FOOT_Z_RANGE * 1000.0
            self.body.feet = feet
            # advance a clock for the observation (periodicity prior)
            self.gait_phase = (self.gait_phase + CONTROL_DT * 1.0) % 1.0
        elif self.control_mode == "residual_pure":
            # gait params computed analytically from the command (like the firmware);
            # the policy is PURE residuals -> cleaner, smaller, matches deployment.
            self._apply_gait_params(analytic_gait_action(self.cmd))
            self._add_foot_residual(action)
        else:  # phase_gait or residual (first 6 dims = gait params)
            self._apply_gait_params(action[:6])
            if self.control_mode == "residual":
                self._add_foot_residual(action[6:24])
        return self.sim.body_targets_from_feet(self.body)

    def _residual_part(self, action):
        """Foot-residual slice of the action (empty for modes without residuals)."""
        if self.control_mode == "residual_pure":
            return action
        if self.control_mode == "residual":
            return action[6:24]
        return np.zeros(0, dtype=np.float32)

    def _add_foot_residual(self, res18):
        """spot_mini_mini-style per-leg foot XYZ residuals added on top of the gait."""
        res = np.asarray(res18).reshape(6, 3)
        self.body.feet[:, 0] += res[:, 0] * FOOT_RESIDUAL * 1000.0
        self.body.feet[:, 1] += res[:, 1] * FOOT_RESIDUAL * 1000.0
        self.body.feet[:, 2] += res[:, 2] * FOOT_RESIDUAL * 1000.0

    def _apply_gait_params(self, a):
        gait = GaitState()
        gait.step_x = a[0] * PG_STEP_XY
        gait.step_z = a[1] * PG_STEP_XY
        gait.step_angle = a[2] * PG_STEP_ANGLE
        gait.step_height = np.interp(a[3], [-1, 1], PG_HEIGHT)
        gait.step_depth = GAIT_COEF["step_depth"]  # stance-phase downward push (traction)
        # a[4] = gait_blend in [0,1]: 0 -> tripod (slow/stable), 1 -> bipod (fast/dynamic)
        blend = float(np.interp(a[4], [-1, 1], [0.0, 1.0]))
        gait.offset = (1.0 - blend) * TRI_OFFSET + blend * BI_OFFSET
        gait.stand_frac = (1.0 - blend) * TRI_STAND_FRAC + blend * BI_STAND_FRAC
        self.gait_blend = blend
        phase_rate = np.interp(a[5], [-1, 1], PG_PHASE_RATE)
        self.gait_phase = (self.gait_phase + CONTROL_DT * phase_rate) % 1.0
        self.gc.set_phase(self.gait_phase)
        self.gc.generate_feet(gait, self.body)

    # ------------------------------------------------------------------ obs
    def _get_obs(self):
        q = self.sim.base_quat()
        grav = _gravity_in_body(q)
        gyro = self.sim.gyro()
        rpy = _quat_to_rpy(q)
        if self.randomize:
            grav, gyro, rpy = self.dr.noisy_imu(grav, gyro, rpy, self.np_random_)
        phase_clock = np.array([np.sin(2 * np.pi * self.gait_phase), np.cos(2 * np.pi * self.gait_phase)])
        obs = np.concatenate([grav, gyro, rpy, self.prev_joint_cmd, phase_clock, self.cmd, self.prev_action])
        return obs.astype(np.float32)

    # ------------------------------------------------------------------ reward
    def _reward_and_done(self):
        d = self.sim.data
        q = self.sim.base_quat()
        yaw = _quat_to_rpy(q)[2]
        wvx, wvy, wvz = d.qvel[0], d.qvel[1], d.qvel[2]
        # world -> body-heading frame (command is body-relative)
        bvx = np.cos(yaw) * wvx + np.sin(yaw) * wvy
        bvy = -np.sin(yaw) * wvx + np.cos(yaw) * wvy
        yaw_rate = d.qvel[5]
        grav = _gravity_in_body(q)

        r_vel = np.exp(-((bvx - self.cmd[0]) ** 2 + (bvy - self.cmd[1]) ** 2) / VEL_SIGMA)
        r_yaw = np.exp(-((yaw_rate - self.cmd[2]) ** 2) / YAW_SIGMA)
        pen_upright = grav[0] ** 2 + grav[1] ** 2
        pen_height = (self.sim.base_height() - STAND_Z) ** 2
        pen_vz = wvz ** 2
        pen_energy = np.sum(d.actuator_force ** 2)
        pen_arate = np.sum((self._cur_action - self.prev_action) ** 2)
        pen_slip = self.sim.foot_slip_sq()
        pen_power = self.sim.actuator_power()  # cost-of-transport: drives efficient stride/freq/gait
        gyro = self.sim.gyro()  # clean sim value (reward never sees the DR-noised obs)
        pen_angvel = gyro[0] ** 2 + gyro[1] ** 2  # roll/pitch oscillation, not just static tilt
        pen_res = np.sum(self._residual_part(self._cur_action) ** 2)

        terms = {
            "r_vel": 1.5 * r_vel,
            "r_yaw": 0.5 * r_yaw,
            "p_upright": -1.0 * pen_upright,
            "p_height": -0.5 * pen_height,
            "p_vz": -1.0 * pen_vz,
            "p_energy": -2e-4 * pen_energy,
            "p_power": -3e-3 * pen_power,
            "p_arate": -0.01 * pen_arate,
            "p_slip": -0.05 * pen_slip,
            "p_angvel": -0.05 * pen_angvel,
            "p_res": -0.02 * pen_res,
            "alive": 0.1,
            "bvx": bvx,
            "bvy": bvy,
            "gait_blend": self.gait_blend,
        }
        reward = sum(v for k, v in terms.items() if k.startswith(("r_", "p_", "alive")))

        terminated = bool((grav[2] > -0.5) or (self.sim.base_height() < 0.03))
        if terminated:
            reward -= 1.0
        return reward, terminated, terms


def make_env(control_mode="phase_gait", randomize=False, seed=0, resample_steps=0, terrain=0.0):
    """Factory for SubprocVecEnv (must be picklable / module-level)."""
    def _thunk():
        return HexapodMjEnv(control_mode=control_mode, randomize=randomize, seed=seed,
                            resample_steps=resample_steps, terrain=terrain)
    return _thunk
