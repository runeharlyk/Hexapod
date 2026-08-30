"""MuJoCo Gymnasium env for hexapod locomotion, designed for sim-to-real.

Command: body-frame velocity vector [vx, vy] (m/s) + yaw rate (rad/s) -- what the reward tracks.

Control modes (action space):
  - "foot":       18-D per-leg foot-position offsets (m) from the default stance -> IK -> joints.
                  Full per-leg authority; the policy learns the whole gait.
  - "phase_gait": 6-D = [step_x, step_y, step_angle, step_height, gait_blend, phase_rate].
                  Policy drives the Bezier gait engine + advances phase; gait_blend morphs
                  tripod(0)<->bipod(1). Low-dim, constrained, transfer-safe.
  - "residual":   24-D = phase_gait (6) + per-leg foot XYZ residuals (18) added on top of the
                  gait before IK (spot_mini_mini "D2" style). Keeps the gait structure but adds
                  corrective freedom for balance / disturbance rejection / terrain. BC-init at 0 residual.

Observation (hardware-available ONLY -- open-loop servos have no encoders):
  a HISTORY of the fast sensor frame -- gravity vector in body frame (3) + gyro (3)
  [+ per-foot contact (6) when obs_contact] at `obs_history` frames spaced HIST_STRIDE control
  steps apart -- then rpy (3) + previous commanded joint angles (18) + gait phase clock
  [sin, cos] (2) + control dt (1) + command (3) + previous action (act_dim).
  The history is what makes blind rough-terrain walking work: a single IMU frame cannot tell a
  slope from a bump mid-stance, whereas the recent trajectory can (implicit terrain estimation).
  dt is included (and randomized in training) so the policy is robust to the real control rate.
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

from src.sim.mj_runtime import HexapodSim, TERRAIN_MODEL_PATH
from src.sim.domain_rand import DomainRandomizer
from src.sim.terrain import randomize_hfield, KINDS as TERRAIN_KINDS
from src.sim.rollout_weights import SCORE_W_STUCK, SCORE_W_KNOCK
from src.robot.gait_schedule import GaitSchedule
from src.robot.firmware_gait import (
    GaitController,
    GaitState,
    BodyState,
    ReflexConfig,
    set_gait,
    DEFAULT_FEET,
    TRI_OFFSET,
    TRI_STAND_FRAC,
    BI_OFFSET,
    BI_STAND_FRAC,
)

# residual_gait: analytic base sets velocity/turn direction, policy adjusts gait params + body height (6)
# and adds foot residuals (18); zero action = the analytic gait, so it stays deploy-safe.
# residual_sched: residual_gait plus authority over the coordination pattern itself -- duty factor
# and the metachronal leg phasing. The per-terrain gait search finds duty ~0.7-0.75 optimal on rough
# ground, which residual_gait cannot reach: its blend axis only interpolates tripod(0.52)->bipod(0.35),
# so it can make the duty LOWER and never higher.
ACT_DIM = {"foot": 18, "phase_gait": 6, "residual": 24, "residual_pure": 18, "residual_gait": 24,
           "residual_sched": 27}

# --- action scaling (m unless noted) ---
FOOT_XY_RANGE = 0.060
FOOT_Z_RANGE = 0.050
FOOT_RESIDUAL = 0.015  # per-leg residual authority; small so the policy nudges the analytic gait rather
                       # than fighting it (transfer)
BODY_ZM_MM = 25.0      # mm, body ride-height authority (residual_gait): +action = taller/more clearance
# residual_gait adds its deltas to the analytic gait's NORMALIZED params before clipping to [-1,1].
# The analytic base parks step_height near the bottom of the range (~-0.95), so a unit delta could
# only ever reach the lower half of the available lift -- hence extra gain on that one channel.
# Order: [step_x, step_y, step_height, gait_blend, phase_rate].
GAIT_DELTA_GAIN = (1.0, 1.0, 2.0, 1.0, 1.0)
# residual_sched coordination authority, as deltas on the base schedule (zero action = base gait).
SCHED_DUTY_RANGE = 0.22    # +-, on stance fraction, clipped to PG_STAND_FRAC
SCHED_LAG_RANGE = 0.25     # +-, on the metachronal lag and the left/right offset, in cycles
# Leg phase offsets enter the gait as (clock + offset) mod 1, so a step change in an offset
# teleports that foot along its trajectory. The policy therefore commands a TARGET phasing and the
# env slews the actual offsets toward it -- retiming a leg is a physical act with a speed limit.
SCHED_LAG_RATE = 0.6       # cycles/s of offset change
# phase_gait scales: [step_x(mm), step_y(mm), step_angle(rad), step_height(mm), stand_frac, phase_rate(/s)]
PG_STEP_XY = 100.0
PG_STEP_ANGLE = 0.8
# Foot lift is limited by the femur servo reaching 90 deg, and that limit MOVES with ride height:
# 54 mm of lift at the nominal 66 mm stance, but 78 mm with the body raised 25 mm. Capping the
# action at 50 mm therefore threw away the robot's real obstacle clearance; 80 mm exposes it, and
# the policy has to raise the body to actually get it (the IK saturates otherwise).
PG_HEIGHT = (10.0, 80.0)
PG_STAND_FRAC = (0.35, 0.85)
PG_PHASE_RATE = (0.0, 3.5)  # cyc/s; servo supports ~3.0-3.5 for fast walking (~0.55 m/s stable)

# --- command sampling ranges (within the robot's achievable envelope so tracking is meaningful) ---
CMD_VX = (-0.25, 0.45)  # servo does ~0.55 m/s fwd; command near it so the policy pushes cadence
CMD_VY = (-0.12, 0.12)
CMD_YAW = (-1.0, 1.0)
ZERO_CMD_PROB = 0.05
# Rough ground lowers the speed the robot can actually make good; commanding what it cannot reach
# parks the tracking kernel in its flat tail (no gradient) and teaches thrashing instead of care.
# So the sampled translation range shrinks with terrain roughness.
# MEASURED 2026-08-02: this throttle was calibrated against the legacy analytic gait, which could
# not make good on fast commands over rough ground. It is wrong for a competent base gait -- the
# searched gait achieves 0.92 of a 0.30 m/s command on 120 mm terrain -- and it was actively
# crippling training: pinned at 0.12 m by the adaptive curriculum, it capped commands at 0.225 m/s
# and the policy trained at a mean |vx| of 0.085 m/s, then had to be benchmarked at 0.15-0.30 m/s.
# Default is now OFF (floor 1.0); pass terrain_speed_floor<1 to restore the old behaviour.
TERRAIN_SPEED_REF = 0.09   # m of roughness at which the top commanded speed is cut to the floor
TERRAIN_SPEED_FLOOR = 1.0  # fraction of the flat-ground command range that survives

# control-rate domain randomization: train across control timesteps so the policy is robust to the real
# loop rate / jitter. dt is an observation and the gait phase advances by the actual dt.
CTRL_DT_RANGE = (0.0125, 0.025)  # s -> 40..80 Hz control (nominal 50 Hz = 0.02)
ACTION_TAU = 0.056               # s, output-filter time constant (alpha = exp(-dt/tau) ~ 0.7 at 50 Hz)

# --- proprioceptive history (implicit terrain estimation) ---
HIST_STRIDE = 3   # control steps between stacked frames: 4 frames span ~180 ms at 50 Hz

# --- foot contact sensing (candidate hardware: FSR or microswitch under each silicone foot) ---
# Faults are a property of the sensor, not of domain randomization, so they are ALWAYS applied --
# evaluating a contact policy on a noise-free sensor would overstate what the hardware buys. They
# are drawn from a dedicated RNG so that adding contact sensors does not shift the terrain/push
# stream, keeping the with/without comparison paired.
CONTACT_FORCE_THRESH = 0.3   # N; one foot of a tripod carries ~2.2 N, so this is a light touch
CONTACT_DROP_MAX = 0.10      # per-step probability a loaded foot reads open (bad contact, hysteresis)
CONTACT_FALSE_MAX = 0.04     # per-step probability an unloaded foot reads closed (vibration, wiring)
CONTACT_DEAD_PROB = 0.03     # per-foot, per-episode: sensor dead all episode (must not be relied on)
CONTACT_FAULT_NOMINAL = (0.5 * CONTACT_DROP_MAX, 0.5 * CONTACT_FALSE_MAX)  # fixed rates without DR
# A real switch/FSR in a silicone foot reports late. This matters far more for a touchdown-triggered
# reflex than for an observation, so the delay is modelled explicitly.
CONTACT_LATENCY_STEPS = 1

# velocity-tracking kernel widths, scaled to the command range (~0.5 * max, ANYmal-style)
VEL_SIGMA = 0.04
R_VEL_WEIGHT = 3.5
# Sharp yaw kernel: a wide one is ~flat for small yaw errors -> no gradient, so heading drifts.
YAW_SIGMA = 0.08

STAND_Z = 0.066  # target body height (m)

# Reward weights, overridable per-env so they can be TUNED AGAINST THE EVALUATION OBJECTIVE rather
# than guessed. MEASURED 2026-08-02: under these defaults the searched gait and the legacy gait
# score 2.818 vs 2.756 -- a 2 % gap -- while `rollout.score` rates them 0.90 vs 0.17 at 80 mm. The
# tracking advantage (+0.51 r_vel) is cancelled by agitation, knock and yaw-drift penalties, so PPO
# has almost no gradient toward the better gait and plenty toward suppressing body rate. That is
# why policies started at the good gait converged well below it. See docs/terrain-locomotion.md.
REWARD_WEIGHTS = {
    "vel": R_VEL_WEIGHT, "yaw": 2.0, "upright": 2.0, "height": 0.5, "vz": 1.0,
    "energy": 1e-4, "power": 1e-3, "arate": 0.01, "slip": 0.05, "angvel": 0.10,
    "res": 0.02, "knock": 0.3, "alive": 0.1,
}

# Analytic command -> gait-params map (deterministic; used by residual_pure mode and BC).
# Coefficients live in GAIT_COEF so they can be tuned by optimize_gait.py (DE/CMA search) and
# persisted to resources/gait_coef.json. gx/gy/gyaw are velocity gains at tripod(0)/bipod(1);
# yaw_comp adds a velocity-proportional yaw correction (cancels the gait's backward yaw drift).
GAIT_COEF = {
    "gx0": 0.259, "gx1": 0.346, "gy0": 0.294, "gy1": 0.360, "gyaw0": 1.602, "gyaw1": 2.153,
    "blend_speed": 0.30, "pr_base": 0.2, "pr_slope": 2.0, "step_height": -0.5, "yaw_comp": 0.0,
    "step_depth": 0.002,  # mm; stance-phase downward push (traction). ~0 = off.
    "pr_yaw": 1.5,  # cadence gain for TURNING: an in-place turn has speed~0, so without this it barely
                    # steps. Mirrors firmware advance_phase max(|len|/25, |angle|*1.5).
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
    # cadence rises with translational speed AND with |yaw| (in-place turns need to step, too)
    phase_rate = np.clip(c["pr_base"] + c["pr_slope"] * speed + c.get("pr_yaw", 0.0) * abs(yaw), -1, 1)
    # Robot faces +Y: forward vx -> step_y (body-Y), lateral vy -> step_x (body-X); matches firmware motion.h.
    return np.array([np.clip(vy / gy, -1, 1),    # a[0] = step_x (body-X) <- lateral vy
                     np.clip(vx / gx, -1, 1),    # a[1] = step_y (body-Y) <- forward vx
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
                 terrain: float = 0.0, terrain_kind: str = "bumps", terrain_feature: float = 1.0,
                 obs_contact: bool = False, obs_history: int = 1, reflex: bool = False,
                 gait_schedule: "GaitSchedule | None" = None,
                 terrain_speed_floor: float = TERRAIN_SPEED_FLOOR,
                 reward_weights: dict | None = None, reward_mode: str = "shaped",
                 arc_stance: bool = False):
        super().__init__()
        assert control_mode in ACT_DIM, control_mode
        assert terrain_kind in TERRAIN_KINDS + ("mixed",), terrain_kind
        self.control_mode = control_mode
        self.randomize = randomize
        self.terrain = terrain  # >0: max bump height (m), per-episode random heightfield
        self.terrain_kind = terrain_kind        # one of terrain.KINDS, or "mixed" (sampled per episode)
        self.terrain_feature = terrain_feature  # bumpiness (eval); training randomizes it per episode
        self.obs_contact = bool(obs_contact)    # per-foot contact sensors in the observation
        self.obs_history = max(1, int(obs_history))
        self.reflex = bool(reflex)              # contact reflexes inside the analytic gait engine
        # Open-loop gait override. None = the legacy analytic map (GAIT_COEF + tripod<->bipod blend),
        # which is what the residual policies were trained on. A GaitSchedule replaces the whole
        # command->gait map AND supplies duty factor, leg phase offsets and body ride height, so a
        # per-terrain search can build an open-loop baseline with the same authority as the policy.
        self.gait_schedule = gait_schedule
        self.terrain_speed_floor = float(terrain_speed_floor)
        self.reward_weights = {**REWARD_WEIGHTS, **(reward_weights or {})}
        assert reward_mode in ("shaped", "score"), reward_mode
        self.reward_mode = reward_mode
        # sweep the planted foot along the arc about the instantaneous centre of rotation
        # instead of the chord between the stroke endpoints (see firmware_gait)
        self.arc_stance = bool(arc_stance)
        self.episode_terrain_kind = terrain_kind  # resolved kind of the current episode's heightfield
        self.fixed_command = None if command is None else np.asarray(command, dtype=np.float32)
        self.curriculum = 1.0  # 0 = forward only, 1 = full command range
        self.gait_blend = 0.0  # last applied tripod(0)<->bipod(1) blend
        self.resample_steps = resample_steps  # >0: resample command mid-episode
        self.episode_seconds = episode_seconds

        self.sim = HexapodSim(TERRAIN_MODEL_PATH) if terrain > 0 else HexapodSim()
        # control-rate: base 50 Hz, randomized per episode (dt is also an observation)
        self._physics_dt = float(self.sim.model.opt.timestep)
        self._base_frame_skip = int(self.sim.frame_skip)
        self.frame_skip_ep = self._base_frame_skip
        self.dt = self.frame_skip_ep * self._physics_dt
        self.max_steps = int(self.episode_seconds / self.dt)
        self.gc = GaitController(reflex=ReflexConfig() if self.reflex else None,
                                 arc_stance=self.arc_stance)
        self.body = BodyState()
        self.np_random_, _ = gym.utils.seeding.np_random(seed)
        self.dr = DomainRandomizer(self.sim.model) if randomize else None
        self._cmd_buffer: deque = deque(maxlen=8)

        self.act_dim = ACT_DIM[control_mode]
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(self.act_dim,), dtype=np.float32)

        self.prev_action = np.zeros(self.act_dim, dtype=np.float32)
        self.filt_action = np.zeros(self.act_dim, dtype=np.float32)  # exp-filtered action actually applied
        self.action_alpha = float(np.exp(-self.dt / ACTION_TAU))
        self.prev_joint_cmd = self.sim.stand_pose.astype(np.float32)
        self.cmd = np.zeros(3, dtype=np.float32)
        self.gait_phase = 0.0
        self.applied_step_height = 0.0
        self.applied_cadence = 0.0
        self.applied_duty = 0.0
        self._zm_delta = 0.0
        self._duty_delta = 0.0
        self._lag_delta = (0.0, 0.0)
        self._offsets = None   # live leg phase offsets (residual_sched); None = pinned to the base

        # contact-sensor fault state (per episode) and the sensor-frame history buffer
        self._contact_rng = np.random.default_rng(0 if seed is None else seed ^ 0xC0FFEE)
        self._contact_dead = np.zeros(6, dtype=bool)
        self._contact_drop, self._contact_false = CONTACT_FAULT_NOMINAL
        self.frame_dim = 6 + (6 if self.obs_contact else 0)  # grav + gyro [+ foot contacts]
        self._hist: deque = deque(maxlen=(self.obs_history - 1) * HIST_STRIDE + 1)
        self._foot_force = np.zeros(6)
        self._nonfoot_force = 0.0
        self._slip = 0.0
        self._contact_buffer: deque = deque(maxlen=CONTACT_LATENCY_STEPS + 1)
        self._contact_sensed = np.zeros(6, dtype=np.float32)

        obs_dim = self.obs_history * self.frame_dim + 3 + 18 + 2 + 1 + 3 + self.act_dim
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)
        self.current_step = 0

    # ------------------------------------------------------------------ reset
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.np_random_, _ = gym.utils.seeding.np_random(seed)
            self._contact_rng = np.random.default_rng(seed ^ 0xC0FFEE)
        if self.randomize:
            self.dr.reset_episode(self.sim.model, self.np_random_)
            r = self.np_random_
            # servo-strength/stiffness DR (avoids overfitting one servo model):
            self.sim.set_servo_scale(kp=float(r.uniform(0.65, 1.35)),      # stiffness spread
                                     stall=float(r.uniform(0.80, 1.20)),   # torque spread
                                     noload=float(r.uniform(0.85, 1.15)))  # speed spread
            self.sim.deadband = float(r.uniform(0.005, 0.026))  # ~0.3-1.5 deg gear lash / PWM deadband
            cr = self._contact_rng
            self._contact_drop = float(cr.uniform(0.0, CONTACT_DROP_MAX))
            self._contact_false = float(cr.uniform(0.0, CONTACT_FALSE_MAX))
            self._contact_dead = cr.random(6) < CONTACT_DEAD_PROB
        else:
            self.sim.set_servo_scale()  # nominal
            self.sim.deadband = 0.0
            self._contact_drop, self._contact_false = CONTACT_FAULT_NOMINAL
            self._contact_dead[:] = False
        if self.randomize:
            self.frame_skip_ep = max(1, int(round(
                float(self.np_random_.uniform(*CTRL_DT_RANGE)) / self._physics_dt)))
        else:
            self.frame_skip_ep = self._base_frame_skip
        self.dt = self.frame_skip_ep * self._physics_dt
        self.action_alpha = float(np.exp(-self.dt / ACTION_TAU))
        self.max_steps = int(self.episode_seconds / self.dt)
        if self.terrain > 0:
            # training randomizes bumpiness per episode; eval uses a fixed value
            feat = float(self.np_random_.uniform(1.0, 2.5)) if self.randomize else self.terrain_feature
            self.episode_terrain_kind = randomize_hfield(
                self.sim.model, self.np_random_, self.terrain, kind=self.terrain_kind, feature=feat)
        self.sim.reset_to_stand()
        self.gc = GaitController(reflex=ReflexConfig() if self.reflex else None,
                                 arc_stance=self.arc_stance)
        self.body = BodyState()
        self.gait_phase = 0.0
        self._offsets = None
        self._duty_delta = 0.0
        self._lag_delta = (0.0, 0.0)
        self._zm_delta = 0.0
        self.prev_action[:] = 0.0
        self.filt_action[:] = 0.0
        self.prev_joint_cmd = self.sim.stand_pose.astype(np.float32)
        self._cmd_buffer.clear()
        self._sample_command()
        self.current_step = 0
        self._foot_force, self._nonfoot_force, self._slip = self.sim.contact_state()
        self._contact_buffer.clear()
        self._update_contact_sensor()
        self._hist.clear()
        return self._get_obs(), {}

    def set_curriculum(self, level):
        self.curriculum = float(np.clip(level, 0.0, 1.0))

    def set_terrain(self, height):
        """Set the next reset's max bump height (m). Only effective if built with terrain>0."""
        self.terrain = float(max(0.0, height))

    def _terrain_speed_scale(self):
        floor = self.terrain_speed_floor
        if self.terrain <= 0.0 or floor >= 1.0:
            return 1.0
        span = 1.0 - floor
        return float(np.clip(1.0 - span * self.terrain / TERRAIN_SPEED_REF, floor, 1.0))

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
            v = self._terrain_speed_scale()
            self.cmd[0] = r.uniform(CMD_VX[0] * L * v, CMD_VX[1] * v)
            self.cmd[1] = r.uniform(CMD_VY[0] * L * v, CMD_VY[1] * L * v)
            self.cmd[2] = r.uniform(CMD_YAW[0] * L, CMD_YAW[1] * L)

    # ------------------------------------------------------------------ step
    def step(self, action):
        raw = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        # exponential output filter: raw NN chatter doesn't transfer to hardware
        self.filt_action = (self.action_alpha * self.filt_action
                            + (1.0 - self.action_alpha) * raw).astype(np.float32)
        action = self.filt_action
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
        self.sim.step_physics(self.frame_skip_ep)

        self._foot_force, self._nonfoot_force, self._slip = self.sim.contact_state()
        self._update_contact_sensor()
        obs = self._get_obs()
        reward, terminated, terms = self._reward_and_done()
        self.current_step += 1
        if self.resample_steps and self.fixed_command is None and self.current_step % self.resample_steps == 0:
            self._sample_command()  # mid-episode command change -> learns transitions
        truncated = self.current_step >= self.max_steps

        self.prev_action = action
        self.prev_joint_cmd = joint_cmd.astype(np.float32)
        return obs, float(reward), bool(terminated), bool(truncated), terms

    def _action_to_joints(self, action):
        self._zm_delta = 0.0     # mm of ride height the policy asks for on top of the schedule
        self._duty_delta = 0.0   # stance-fraction delta
        self._lag_delta = (0.0, 0.0)  # (metachronal lag, left/right offset) deltas, in cycles
        if self.control_mode == "foot":
            feet = DEFAULT_FEET.copy()
            off = action.reshape(6, 3)
            feet[:, 0] += off[:, 0] * FOOT_XY_RANGE * 1000.0  # m->mm (firmware units)
            feet[:, 1] += off[:, 1] * FOOT_XY_RANGE * 1000.0
            feet[:, 2] += off[:, 2] * FOOT_Z_RANGE * 1000.0
            self.body.feet = feet
            # advance a clock for the observation (periodicity prior)
            self.gait_phase = (self.gait_phase + self.dt * 1.0) % 1.0
        elif self.control_mode == "residual_pure":
            # analytic gait params from the command (like the firmware); policy is PURE residuals.
            self._apply_gait_params(self._base_gait_action())
            self._add_foot_residual(action)
        elif self.control_mode == "residual_gait":
            a = self._base_gait_action().copy()
            g = GAIT_DELTA_GAIN
            a[0] = np.clip(a[0] + g[0] * action[0], -1.0, 1.0)  # step_x = body-X = LATERAL stride
            a[1] = np.clip(a[1] + g[1] * action[1], -1.0, 1.0)  # step_y = body-Y = FORWARD stride
            a[3] = np.clip(a[3] + g[2] * action[2], -1.0, 1.0)  # step_height
            a[4] = np.clip(a[4] + g[3] * action[3], -1.0, 1.0)  # tripod(0)<->bipod(1) blend
            a[5] = np.clip(a[5] + g[4] * action[4], -1.0, 1.0)  # phase_rate
            self._zm_delta = float(action[5]) * BODY_ZM_MM  # + = taller / more clearance
            self._apply_gait_params(a)
            self._add_foot_residual(action[6:24])
        elif self.control_mode == "residual_sched":
            a = self._base_gait_action().copy()
            g = GAIT_DELTA_GAIN
            a[0] = np.clip(a[0] + g[0] * action[0], -1.0, 1.0)  # step_x = body-X = LATERAL stride
            a[1] = np.clip(a[1] + g[1] * action[1], -1.0, 1.0)  # step_y = body-Y = FORWARD stride
            a[3] = np.clip(a[3] + g[2] * action[2], -1.0, 1.0)  # step_height
            a[4] = np.clip(a[4] + g[3] * action[3], -1.0, 1.0)  # stride-gain interpolator
            a[5] = np.clip(a[5] + g[4] * action[4], -1.0, 1.0)  # phase_rate
            self._zm_delta = float(action[5]) * BODY_ZM_MM
            self._duty_delta = float(action[6]) * SCHED_DUTY_RANGE
            self._lag_delta = (float(action[7]) * SCHED_LAG_RANGE,
                               float(action[8]) * SCHED_LAG_RANGE)
            self._apply_gait_params(a)
            self._add_foot_residual(action[9:27])
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
        if self.control_mode == "residual_gait":
            return action[6:24]  # foot residuals only; gait deltas self-regulate via the power penalty
        if self.control_mode == "residual_sched":
            return action[9:27]
        return np.zeros(0, dtype=np.float32)

    def _add_foot_residual(self, res18):
        """spot_mini_mini-style per-leg foot XYZ residuals added on top of the gait."""
        res = np.asarray(res18).reshape(6, 3)
        self.body.feet[:, 0] += res[:, 0] * FOOT_RESIDUAL * 1000.0
        self.body.feet[:, 1] += res[:, 1] * FOOT_RESIDUAL * 1000.0
        self.body.feet[:, 2] += res[:, 2] * FOOT_RESIDUAL * 1000.0

    def _base_gait_action(self):
        """Command -> the 6 normalized gait params the policy sits on top of."""
        if self.gait_schedule is not None:
            return self.gait_schedule.gait_action(self.cmd)
        return analytic_gait_action(self.cmd)

    def _slew_offsets(self, base_offset):
        """Move the live leg phase offsets toward the commanded pattern at a bounded rate.

        Zero lag delta leaves them pinned to the base pattern, so a zero action is exactly the
        base gait. A non-zero delta re-times the legs metachronally (right side by `dlag` per leg
        position, left side by `dlag` plus a left/right shift `dcontra`), and the slew keeps the
        foot trajectories continuous while it happens.
        """
        dlag, dcontra = self._lag_delta
        if dlag == 0.0 and dcontra == 0.0 and self._offsets is None:
            return base_offset
        step = np.array([0.0, 1.0, 2.0])
        target = (base_offset + np.concatenate([dlag * step, dcontra + dlag * step])) % 1.0
        if self._offsets is None:
            self._offsets = np.array(base_offset, dtype=float)
        err = (target - self._offsets + 0.5) % 1.0 - 0.5   # shortest way round the cycle
        limit = SCHED_LAG_RATE * self.dt
        self._offsets = (self._offsets + np.clip(err, -limit, limit)) % 1.0
        return self._offsets

    def _apply_gait_params(self, a):
        gait = GaitState()
        gait.step_x = a[0] * PG_STEP_XY
        gait.step_y = a[1] * PG_STEP_XY
        gait.step_angle = a[2] * PG_STEP_ANGLE
        gait.step_height = np.interp(a[3], [-1, 1], PG_HEIGHT)
        # a[4] = gait_blend in [0,1]: 0 -> tripod (slow/stable), 1 -> bipod (fast/dynamic)
        blend = float(np.interp(a[4], [-1, 1], [0.0, 1.0]))
        sched = self.gait_schedule
        if sched is None:
            gait.step_depth = GAIT_COEF["step_depth"]  # stance-phase downward push (traction)
            base_offset = (1.0 - blend) * TRI_OFFSET + blend * BI_OFFSET
            base_duty = (1.0 - blend) * TRI_STAND_FRAC + blend * BI_STAND_FRAC
            ride_mm = 0.0
        else:
            # the schedule owns coordination outright: blend only interpolates its stride gains
            gait.step_depth = sched.step_depth
            base_offset = sched.offsets()
            base_duty = float(sched.duty)
            ride_mm = float(sched.ride_mm)
        gait.stand_frac = float(np.clip(base_duty + self._duty_delta, *PG_STAND_FRAC))
        gait.offset = self._slew_offsets(base_offset)
        self.applied_duty = float(gait.stand_frac)
        self.body.zm = -(ride_mm + self._zm_delta)  # firmware sign: negative zm raises the body
        self.gait_blend = blend
        phase_rate = np.interp(a[5], [-1, 1], PG_PHASE_RATE)
        self.applied_step_height = float(gait.step_height)
        self.applied_cadence = float(phase_rate)
        # phase_scale is the reflexes' brake on the shared clock (1.0 when reflexes are off)
        self.gait_phase = (self.gait_phase + self.dt * phase_rate * self.gc.phase_scale) % 1.0
        self.gc.set_phase(self.gait_phase)
        if self.reflex:
            self.gc.generate_feet(gait, self.body, contacts=self._contact_sensed, dt=self.dt)
        else:
            self.gc.generate_feet(gait, self.body)

    # ------------------------------------------------------------------ obs
    def _foot_contact_sensor(self):
        """Binary per-foot contact as a real switch/FSR would report it: a force threshold, plus
        dropouts, false closes and (under DR) dead sensors -- the policy must treat it as a hint,
        not ground truth, or it will not transfer."""
        c = (self._foot_force > CONTACT_FORCE_THRESH).astype(np.float32)
        r = self._contact_rng
        c = np.where(r.random(6) < self._contact_drop, 0.0, c)
        c = np.where((r.random(6) < self._contact_false) & (c == 0.0), 1.0, c)
        c[self._contact_dead] = 0.0
        return c

    def _update_contact_sensor(self):
        """Latch this step's sensor reading, delayed by the sensor latency. Sampled ONCE per step so
        the observation and the gait reflexes act on the same signal."""
        self._contact_buffer.append(self._foot_contact_sensor())
        idx = max(0, len(self._contact_buffer) - 1 - CONTACT_LATENCY_STEPS)
        self._contact_sensed = self._contact_buffer[idx]

    def _get_obs(self):
        q = self.sim.base_quat()
        grav = _gravity_in_body(q)
        gyro = self.sim.gyro()
        rpy = _quat_to_rpy(q)
        if self.randomize:
            grav, gyro, rpy = self.dr.noisy_imu(grav, gyro, rpy, self.np_random_)
        frame = np.concatenate([grav, gyro, self._contact_sensed]) if self.obs_contact \
            else np.concatenate([grav, gyro])
        if not self._hist:
            self._hist.extend([frame] * self._hist.maxlen)  # episode start: history = the first frame
        self._hist.append(frame)
        stack = np.concatenate([self._hist[-1 - k * HIST_STRIDE] for k in range(self.obs_history)])
        phase_clock = np.array([np.sin(2 * np.pi * self.gait_phase), np.cos(2 * np.pi * self.gait_phase)])
        obs = np.concatenate([stack, rpy, self.prev_joint_cmd, phase_clock,
                              np.array([self.dt], dtype=np.float32), self.cmd, self.prev_action])
        return obs.astype(np.float32)

    # ------------------------------------------------------------------ reward
    def _reward_and_done(self):
        d = self.sim.data
        q = self.sim.base_quat()
        yaw = _quat_to_rpy(q)[2]
        wvx, wvy, wvz = d.qvel[0], d.qvel[1], d.qvel[2]
        # world -> body-heading frame (command is body-relative)
        body_x_vel = np.cos(yaw) * wvx + np.sin(yaw) * wvy   # body +X (this robot's LATERAL axis)
        body_y_vel = -np.sin(yaw) * wvx + np.cos(yaw) * wvy  # body +Y (this robot's FORWARD/long axis)
        yaw_rate = d.qvel[5]
        grav = _gravity_in_body(q)

        # robot faces +Y: cmd[0]=forward tracks body-Y, cmd[1]=lateral tracks body-X
        fwd_vel, lat_vel = body_y_vel, body_x_vel
        r_vel = np.exp(-((fwd_vel - self.cmd[0]) ** 2 + (lat_vel - self.cmd[1]) ** 2) / VEL_SIGMA)
        r_yaw = np.exp(-((yaw_rate - self.cmd[2]) ** 2) / YAW_SIGMA)
        pen_upright = grav[0] ** 2 + grav[1] ** 2
        pen_height = (self.sim.base_height() - STAND_Z) ** 2
        pen_vz = wvz ** 2
        pen_energy = np.sum(d.actuator_force ** 2)
        pen_arate = np.sum((self._cur_action - self.prev_action) ** 2)
        pen_slip = self._slip
        # shins/belly dragging on the ground, as a fraction of body weight: the cost of plowing
        # through an obstacle instead of stepping over it
        pen_knock = self._nonfoot_force / (float(np.sum(self.sim.model.body_mass)) * 9.81)
        pen_power = self.sim.actuator_power()  # cost-of-transport: drives efficient stride/freq/gait
        gyro = self.sim.gyro()  # clean sim value (reward never sees the DR-noised obs)
        pen_angvel = gyro[0] ** 2 + gyro[1] ** 2  # roll/pitch oscillation, not just static tilt
        pen_res = np.sum(self._residual_part(self._cur_action) ** 2)

        w = self.reward_weights
        if self.reward_mode == "score":
            # Per-step analogue of rollout.score. MEASURED: the shaped reward below ranks gaits at
            # rho=0.81 globally but only 0.61 near the optimum, and a policy trained on it earns
            # MORE reward than its base gait (+6.664 vs +6.382) while scoring 0.23 LOWER. It is
            # optimizing the shaping terms, which the evaluation objective does not contain.
            # So: multiply the tracking kernels instead of adding them (a yaw error must be able to
            # ruin the step, not be bought off with speed), and keep only the two penalties the
            # score actually prices. Falls are already handled by termination.
            stuck = float(abs(fwd_vel) < 0.3 * abs(self.cmd[0])) if abs(self.cmd[0]) > 1e-6 else 0.0
            track = r_vel * r_yaw
            terms = {
                "r_vel": w["vel"] * track,
                "r_yaw": 0.0,
                "p_upright": 0.0, "p_height": 0.0, "p_vz": 0.0, "p_energy": 0.0,
                "p_power": 0.0, "p_arate": -w["arate"] * pen_arate, "p_slip": 0.0,
                "p_angvel": 0.0, "p_res": 0.0,
                "p_knock": -w["vel"] * SCORE_W_KNOCK * pen_knock,
                "alive": -w["vel"] * SCORE_W_STUCK * stuck,
            }
            terms.update({
                "track_vel": float(r_vel),
                "bvx": fwd_vel, "bvy": lat_vel, "gait_blend": self.gait_blend,
                "step_h": self.applied_step_height, "cadence": self.applied_cadence,
                "duty": self.applied_duty, "body_zm": float(self.body.zm), "knock": pen_knock,
                "contacts": float(np.count_nonzero(self._foot_force > CONTACT_FORCE_THRESH)),
            })
            reward = sum(v for k, v in terms.items() if k.startswith(("r_", "p_", "alive")))
            terminated = bool((grav[2] > -0.5) or (self.sim.base_height() < 0.03))
            if terminated:
                reward -= 1.0
            return reward, terminated, terms

        terms = {
            "r_vel": w["vel"] * r_vel,
            "r_yaw": w["yaw"] * r_yaw,
            "p_upright": -w["upright"] * pen_upright,
            "p_height": -w["height"] * pen_height,
            "p_vz": -w["vz"] * pen_vz,
            "p_energy": -w["energy"] * pen_energy,
            "p_power": -w["power"] * pen_power,
            "p_arate": -w["arate"] * pen_arate,
            "p_slip": -w["slip"] * pen_slip,
            "p_angvel": -w["angvel"] * pen_angvel,
            "p_res": -w["res"] * pen_res,
            "p_knock": -w["knock"] * pen_knock,
            "alive": w["alive"],
            # Unweighted velocity-tracking kernel in [0,1], identical in every reward mode, so a
            # curriculum can gate on tracking quality without unpicking the reward's weighting.
            "track_vel": float(r_vel),
            "bvx": fwd_vel,   # report forward (body-Y) as "bvx" so eval compares it to cmd[0]
            "bvy": lat_vel,   # lateral (body-X) vs cmd[1]
            "gait_blend": self.gait_blend,
            # what the policy actually asked the gait engine for -- the adaptivity readout
            "step_h": self.applied_step_height,
            "cadence": self.applied_cadence,
            "duty": self.applied_duty,
            "body_zm": float(self.body.zm),
            "knock": pen_knock,
            "contacts": float(np.count_nonzero(self._foot_force > CONTACT_FORCE_THRESH)),
        }
        reward = sum(v for k, v in terms.items() if k.startswith(("r_", "p_", "alive")))

        terminated = bool((grav[2] > -0.5) or (self.sim.base_height() < 0.03))
        if terminated:
            reward -= 1.0
        return reward, terminated, terms


def make_env(control_mode="phase_gait", randomize=False, seed=0, resample_steps=0, terrain=0.0,
             terrain_kind="bumps", obs_contact=False, obs_history=1, gait_schedule=None,
             terrain_speed_floor=TERRAIN_SPEED_FLOOR, reward_weights=None,
             reward_mode="shaped"):
    """Factory for SubprocVecEnv (must be picklable / module-level)."""
    def _thunk():
        return HexapodMjEnv(control_mode=control_mode, randomize=randomize, seed=seed,
                            resample_steps=resample_steps, terrain=terrain, terrain_kind=terrain_kind,
                            obs_contact=obs_contact, obs_history=obs_history,
                            gait_schedule=gait_schedule,
                            terrain_speed_floor=terrain_speed_floor,
                            reward_weights=reward_weights, reward_mode=reward_mode)
    return _thunk


CONFIG_FILE = "env_config.json"
# Constructor kwargs that change the observation/action layout or the gait the policy sits on, so
# evaluation must reproduce them. `gait_schedule` is stored as a plain dict of floats.
CONFIG_KEYS = ("control_mode", "obs_contact", "obs_history", "gait_schedule")


def save_env_config(rundir, **kw):
    cfg = {k: kw[k] for k in CONFIG_KEYS if k in kw}
    sched = cfg.get("gait_schedule")
    if sched is not None and not isinstance(sched, dict):
        cfg["gait_schedule"] = sched.to_dict()
    with open(os.path.join(rundir, CONFIG_FILE), "w") as f:
        json.dump(cfg, f, indent=2)


def load_env_config(rundir):
    """Observation/action layout and base gait a run was trained with. Runs predating the file
    used the single-frame, no-contact observation on the legacy analytic gait."""
    path = os.path.join(rundir, CONFIG_FILE)
    cfg = {"obs_contact": False, "obs_history": 1, "gait_schedule": None}
    if os.path.exists(path):
        cfg.update(json.load(open(path)))
    return cfg
