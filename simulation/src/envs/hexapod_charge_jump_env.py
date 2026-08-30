"""MuJoCo Gymnasium env for a hexapod DIRECTIONAL CHARGE-JUMP (sim-only demo).

A phased "hold, then let go" leap, trained end-to-end as one committed jump:

  1. READY   (charging=True):  start from a SEMI-RANDOM joint pose and settle -- feet
     planted, body leveling out, waiting for the release.
  2. LAUNCH  (released, still grounded): one explosive push -- build upward AND
     commanded-direction velocity off the ground.
  3. FLIGHT  (airborne): go high and far; the apex and the distance travelled (from
     the release point) are what pay off.
  4. RECOVER (back on the ground, ~1 s window): get upright -- on its feet, body level, near
     stand height. Stabilizing steps/hops are ALLOWED (a projectile keeps its launch horizontal
     velocity, which these servos cannot arrest without a recovery step); only a true flip fails.

ONE committed jump is enforced STRUCTURALLY: the airborne reward is paid only during the FIRST
flight (so there's no incentive to bounce your way across in many small travel-hops), and the
episode ends one recovery window after the first touchdown. Landing is treated as a forgiving
RECOVERY, not a dead-still "stick": a big leap out-scores standing even with a scrappy landing,
which is what makes the policy commit to jumping (an earlier "stick it" reward made jump-and-
crash score worse than standing, so it refused to jump). The anti-suicide alive floor makes
first-flight gating safe (an earlier attempt without it collapsed into always crashing).

In training the release fires at a random step (`auto_release`); at deploy the GUI
holds/releases it (`auto_release=False` + `env.release()`).

Command: a horizontal jump direction (unit vector, body frame), sampled per episode.
Action: action_mode="joint" -> 18-D direct joint targets (plateaus at vertical-only jumps);
        action_mode="body"  -> 6-D body kinematics [foot_spread, height, x, y, roll, pitch] -> IK.
        The body-kinematics action is what unlocked directional-jump-and-recover (see BODY_* consts);
        yaw is intentionally excluded, reserved for a future spin-jump.

Observation is HARDWARE-AVAILABLE only (IMU gravity/gyro/rpy + prev commanded joint angles + command
dir + charging flag + time-since-release + prev action) -- the real robot has no base pose/velocity/
contact sensing. Reward may still use privileged sim state; only the OBSERVATION is restricted.
With `randomize=True`, domain randomization (mass/inertia/CoM/friction/servo torque-speed/latency/IMU
noise/pushes) plus the realistic torque-speed actuator model make this a sim-to-real candidate.
"""

from __future__ import annotations

from collections import deque
import numpy as np
import gymnasium as gym
import mujoco

from src.sim.mj_runtime import HexapodSim, CONTROL_DT
from src.sim.domain_rand import DomainRandomizer
from src.envs.hexapod_mj_env import _quat_to_rpy, _gravity_in_body
from src.envs.hexapod_jump_env import STAND_Z, ACTION_SCALE
from src.robot.firmware_gait import BodyState, DEFAULT_FEET

RELEASE_HORIZON = 60.0  # steps used to normalize "time since release" in the obs (~1.2 s)
# An actuator-strength curriculum (strong servos early -> realistic) does NOT work: the strong-servo
# jump evaporates as servos weaken and the policy gets stuck in the standing optimum. What works:
# warm-start a fresh run from a policy that already jumps (`train_jump.py --init-from`) and refine on
# realistic servos.

# body-kinematics action (action_mode="body"): 6-D [foot_spread, height(zm), x, y, roll, pitch].
# Feet stay planted; the policy poses the BODY -> IK -> joints. A fast height (zm) ramp is the jump.
# Far smaller + smoother than 18-joint control, and stays on the gait/IK manifold the robot lands on.
# YAW is deliberately excluded for now -- reserve it for a later spin-jump (jump + rotate on the spot).
BODY_ACT_DIM = 6
BODY_FOOT_SPREAD = 40.0    # mm, radial foot offset from body center (widen/narrow the stance)
BODY_ZM = (26.0, -64.0)    # zm at action -1 (deep crouch ~40 mm) .. +1 (full extend ~130 mm)
BODY_XY = 40.0             # mm, body x/y translation
BODY_RPY = 0.5             # rad, body roll/pitch (NOT yaw)

# --- start pose / charge timing (control steps; auto_release training only) ---
JOINT_JITTER = 0.10              # rad, uniform per-joint perturbation of the start pose
CHARGE_MIN, CHARGE_MAX = 15, 45  # 0.3-0.9 s ready phase before the launch fires
RECOVER_STEPS = 50               # ~1 s recovery window after the first touchdown, then end episode

# --- reward weights ---
# Hard-won structure (each guards against a distinct failure mode seen in training):
#  * No reward while charging/grounded before the jump -> a grounded income basin makes the policy
#    park there and never explore jumping. "All feet down before the jump" is a ONE-TIME takeoff
#    bonus (earnable only by jumping), not a per-step grounded reward.
#  * Airborne reward is paid only during the FIRST flight -> one committed leap, not many small
#    travel-hops (paying per airborne step rewards total airtime => bouncing).
#  * ALIVE must exceed the per-step penalty rate, else "suicide" (crash early to stop the bleeding)
#    beats surviving; a big leap must out-score standing even with a scrappy landing, or the policy
#    just stands. Modest terminals + --target-kl avoid the advantage-spike -> collapse failure.
#  * LANDING is a forgiving RECOVERY: a dense per-step reward for being upright + on-feet + near
#    stand height over a ~1 s window (stabilizing steps/hops ALLOWED). "Recover upright", not
#    "stick it dead-still" -- a projectile keeps its launch horizontal velocity, which these servos
#    cannot arrest without a recovery step. Only a true flip / body-slam terminates.
W_SLIP = 0.3        # feet must not slide while planted (low, since the random-start settle slips a lot)
W_UPRIGHT = 1.0     # keep the body level (all phases)
W_SPIN = 0.01       # discourage tumbling
W_STANCE = 2.0      # READY: one-time bonus for launching from a full stance
W_AIR = 1.0         # FLIGHT: flat bonus for zero feet touching (first flight only)
W_AIR_H = 12.0      # FLIGHT: x apex height while airborne ("jump high")
# "long" must clearly out-pay a safe vertical hop or the policy just jumps straight up (landing a
# horizontal leap is harder, so with weak direction incentives it avoids travelling).
W_AIRVEL = 30.0     # FLIGHT: x commanded-direction speed while airborne ("jump long")
# Empirically apex is CAPPED ~64 mm by the actuators
# for a recoverable jump -- pushing apex/airtime harder does NOT raise it. And airtime competes with
# distance (max airtime = a vertical takeoff), so a heavy airtime weight collapses the leap. Keep
# airtime a light nudge; distance is the main directional driver. Recovery at 1.0 -> clean landings.
W_RECOVER = 1.0     # LAND: per-step reward for being upright + on-feet + near stand height (recovery)
W_AIRTIME = 2.0     # terminal: airborne steps in the first flight (light -- high weight starves distance)
W_APEX = 80.0       # terminal: peak apex reached in flight ("jump high"; capped ~64 mm by physics)
W_DIST = 250.0      # terminal: peak displacement from release, along the jump dir ("jump long")
ALIVE = 0.20        # per-step survival bonus (anti-suicide floor)
CRASH_PENALTY = 30.0

VMAX = 2.0
RECOVER_LEVEL_K = 10.0   # recovery uprightness kernel: tilt (gravity_xy^2)
RECOVER_H_K = 200.0      # recovery height kernel: height error from stand^2
# Jump height is measured from the LOWEST point of the chassis (not the base center), so a tilted
# body high at its center isn't credited as a big jump. "flight" = feet off AND that lowest point
# risen by more than this above its stand value (leg-retraction lifts the feet but not the body, so
# it can't fake this) -- only genuine ballistic flight is rewarded.
FLIGHT_RISE_MIN = 0.015  # m, min rise of the lowest body point to count as flight (un-fakeable:
                         # leg retraction lifts the feet, not the chassis). 15 mm of chassis rise is
                         # still unambiguous flight, while the lowest point rises less than the center
                         # on a marginal hop.


class HexapodChargeJumpEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, episode_seconds: float = 3.0, seed: int | None = None,
                 auto_release: bool = True, direction: float | None = None,
                 action_mode: str = "joint", randomize: bool = False):
        super().__init__()
        assert action_mode in ("joint", "body")
        self.action_mode = action_mode
        self.randomize = randomize
        self.sim = HexapodSim()
        self.stand_pose = self.sim.stand_pose.astype(np.float32)
        self.jnt_lo, self.jnt_hi = self.sim.joint_limits
        self.max_steps = int(episode_seconds / CONTROL_DT)
        self.auto_release = auto_release
        self.fixed_direction = direction  # radians; None -> sample per episode
        self.np_random_, _ = gym.utils.seeding.np_random(seed)
        self._body = BodyState()  # scratch BodyState for action_mode="body" -> IK
        # lowest chassis point at the nominal stand -> baseline for jump height "rise"
        self.stand_low = STAND_Z - float(self.sim.chassis_half[2])
        self.dr = DomainRandomizer(self.sim.model) if randomize else None
        self._cmd_buffer: deque = deque(maxlen=8)  # action-latency buffer (DR)
        self.release_at_step = None  # control step at which the charge was released

        act_dim = 18 if action_mode == "joint" else BODY_ACT_DIM
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(act_dim,), dtype=np.float32)
        # HARDWARE-AVAILABLE obs only (the real robot has no base pose/velocity/contact sensing):
        # gravity(3)+gyro(3)+rpy(3) from the IMU, prev commanded joint angles(18), command dir(2),
        # charging flag(1), normalized time-since-release(1), prev action(act_dim).
        obs_dim = 3 + 3 + 3 + 18 + 2 + 1 + 1 + act_dim
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)

        self.curriculum = 1.0  # 0 = vertical jump+land from stand; 1 = directional leap + random start
        self.prev_action = np.zeros(act_dim, dtype=np.float32)
        self.prev_joint_cmd = self.stand_pose.copy()
        self._cur_action = np.zeros(act_dim, dtype=np.float32)
        self.dir = np.array([1.0, 0.0], dtype=np.float32)  # body-frame jump direction
        self.charging = True
        self.release_step = CHARGE_MAX
        self.was_airborne = False   # left the ground at least once since release
        self.landed = False         # first touchdown after the flight -> recovery window begins
        self.recover_counter = 0    # steps since first touchdown
        self.prev_n_contact = 6     # feet planted on the previous step (for the takeoff-stance bonus)
        self.took_off = False       # first takeoff has happened (for the one-time stance bonus)
        self.peak_apex = 0.0        # highest apex reached in flight
        self.peak_dist = 0.0        # best displacement from release, along the (world) jump dir
        self.air_steps = 0          # airborne steps during the first flight (airtime)
        self.release_wpos = None
        self.world_dir = None       # jump dir frozen to world frame at the moment of release
        self.current_step = 0

    # ------------------------------------------------------------------ reset
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.np_random_, _ = gym.utils.seeding.np_random(seed)
        if self.randomize:
            self.dr.reset_episode(self.sim.model, self.np_random_)  # mass/inertia/CoM/friction/IMU/latency
            self.sim.set_servo_scale(kp=self.np_random_.uniform(0.9, 1.1),
                                     stall=self.np_random_.uniform(0.8, 1.15),
                                     noload=self.np_random_.uniform(0.8, 1.2))  # torque-speed curve
        self._cmd_buffer.clear()
        self.sim.reset_to_stand()
        # semi-random start pose: perturb the joints (jitter ramps in with the curriculum)
        jit = JOINT_JITTER * self.curriculum
        start = (self.stand_pose + self.np_random_.uniform(-jit, jit, 18)).astype(np.float64)
        self.sim.data.qpos[self.sim.qpos_adr] = start
        self.sim.joint_target = start.copy()  # servo holds the jittered pose (motor ctrl set by loop)
        mujoco.mj_forward(self.sim.model, self.sim.data)

        theta = (self.fixed_direction if self.fixed_direction is not None
                 else self.np_random_.uniform(0.0, 2.0 * np.pi))
        self.dir = np.array([np.cos(theta), np.sin(theta)], dtype=np.float32)
        self.charging = True
        self.release_step = (int(self.np_random_.integers(CHARGE_MIN, CHARGE_MAX + 1))
                             if self.auto_release else 10 ** 9)
        self.prev_action[:] = 0.0
        self.prev_joint_cmd = start.astype(np.float32)
        self._cur_action[:] = 0.0
        self.was_airborne = False
        self.landed = False
        self.recover_counter = 0
        self.prev_n_contact = 6
        self.took_off = False
        self.peak_apex = 0.0
        self.peak_dist = 0.0
        self.air_steps = 0
        self.release_wpos = None
        self.world_dir = None
        self.release_at_step = None
        self.current_step = 0
        return self._get_obs(), {}

    def set_curriculum(self, level):
        """Training hook: 0 = vertical jump+land from the stand pose (the base skill), 1 = full
        directional leap from a random start. Scales the start jitter and the horizontal-direction
        rewards, so the policy learns to land a jump first, then to aim it."""
        self.curriculum = float(np.clip(level, 0.0, 1.0))

    def set_direction(self, theta: float):
        """GUI hook: aim the jump (radians, body frame)."""
        self.dir = np.array([np.cos(theta), np.sin(theta)], dtype=np.float32)

    def release(self):
        """GUI hook: let go of the charge -> launch."""
        self.charging = False

    # ------------------------------------------------------------------ helpers
    def _action_to_joints(self, action):
        """Map the raw action to 18 joint targets (rad), per action_mode."""
        if self.action_mode == "joint":
            return np.clip(self.stand_pose + action * ACTION_SCALE, self.jnt_lo, self.jnt_hi)
        # body-kinematics: pose the body (+ radial foot spread) -> IK. Feet stay planted.
        b = self._body
        feet = DEFAULT_FEET.copy()
        r = np.hypot(feet[:, 0], feet[:, 1])
        spread = float(action[0]) * BODY_FOOT_SPREAD
        feet[:, 0] += spread * feet[:, 0] / r
        feet[:, 1] += spread * feet[:, 1] / r
        b.feet = feet
        b.zm = float(np.interp(action[1], [-1.0, 1.0], BODY_ZM))  # height: +1 = extend (push up)
        b.xm = float(action[2]) * BODY_XY
        b.ym = float(action[3]) * BODY_XY
        b.omega = float(action[4]) * BODY_RPY  # roll
        b.phi = float(action[5]) * BODY_RPY    # pitch
        b.psi = 0.0                            # yaw held at 0 (reserved for a future spin-jump)
        return np.clip(self.sim.kin.inverse_kinematics(b, degrees=False), self.jnt_lo, self.jnt_hi)

    # ------------------------------------------------------------------ step
    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        self._cur_action = action
        joint_cmd = self._action_to_joints(action)
        # action latency (DR): the servos act on a delayed command
        self._cmd_buffer.append(joint_cmd)
        if self.randomize and self.dr.action_latency_steps > 0:
            idx = max(0, len(self._cmd_buffer) - 1 - self.dr.action_latency_steps)
            self.sim.set_joint_targets(self._cmd_buffer[idx])
        else:
            self.sim.set_joint_targets(joint_cmd)
        if self.randomize:
            self.dr.maybe_push(self.sim.model, self.sim.data, self.np_random_, self.current_step)
        self.sim.step_physics()
        self.current_step += 1
        if self.auto_release and self.current_step >= self.release_step:
            self.charging = False
        if not self.charging and self.release_wpos is None:  # freeze the launch reference frame
            yaw = _quat_to_rpy(self.sim.base_quat())[2]
            self.release_at_step = self.current_step
            self.release_wpos = self.sim.data.qpos[0:2].copy()
            self.world_dir = np.array([np.cos(yaw) * self.dir[0] - np.sin(yaw) * self.dir[1],
                                       np.sin(yaw) * self.dir[0] + np.cos(yaw) * self.dir[1]])

        obs = self._get_obs()
        reward, terminated, terms = self._reward_and_done()
        if self.landed:
            self.recover_counter += 1
        # end the episode after the ~1 s recovery window that starts at the first touchdown
        truncated = (self.current_step >= self.max_steps
                     or (self.landed and self.recover_counter >= RECOVER_STEPS))
        if terminated or truncated:  # bigger jump = higher apex + longer airtime + farther
            terms["r_apex"] = W_APEX * self.peak_apex
            terms["r_dist"] = W_DIST * self.curriculum * self.peak_dist  # "long" ramps in with curriculum
            terms["r_airtime"] = W_AIRTIME * self.air_steps
            reward += terms["r_apex"] + terms["r_dist"] + terms["r_airtime"]

        self.prev_action = action
        self.prev_joint_cmd = joint_cmd.astype(np.float32)
        return obs, float(reward), bool(terminated), bool(truncated), terms

    # ------------------------------------------------------------------ obs (hardware-available only)
    def _get_obs(self):
        q = self.sim.base_quat()
        grav = _gravity_in_body(q)
        gyro = self.sim.gyro()
        rpy = _quat_to_rpy(q)
        if self.randomize:  # IMU bias + noise (as on the real robot)
            grav, gyro, rpy = self.dr.noisy_imu(grav, gyro, rpy, self.np_random_)
        # time-since-release: the firmware can track this (it knows when the button was released);
        # 0 while charging, ramps to 1 over ~1.2 s after launch -> a landing/recovery clock.
        t_rel = 0.0 if self.release_at_step is None else min(
            1.0, (self.current_step - self.release_at_step) / RELEASE_HORIZON)
        obs = np.concatenate([
            grav, gyro, rpy,
            self.prev_joint_cmd,
            self.dir,
            [1.0 if self.charging else 0.0],
            [t_rel],
            self.prev_action,
        ])
        return obs.astype(np.float32)

    # ------------------------------------------------------------------ reward
    def _reward_and_done(self):
        q = self.sim.base_quat()
        grav = _gravity_in_body(q)
        gyro = self.sim.gyro()
        h = self.sim.base_height()
        jh = self.sim.base_lowest_z() - self.stand_low  # jump height: rise of the LOWEST body point
        feet = self.sim.feet_in_contact()
        n_contact = int(feet.sum())
        airborne = n_contact == 0
        flying = airborne and jh > FLIGHT_RISE_MIN  # genuine flight (body risen), not leg-retraction

        pen_slip = self.sim.foot_slip_sq()
        terms = {
            "p_slip": -W_SLIP * pen_slip,
            "p_spin": -W_SPIN * float(np.sum(gyro ** 2)),
            "p_upright": -W_UPRIGHT * (grav[0] ** 2 + grav[1] ** 2),
            "alive": ALIVE,
        }

        # no reward while charging/grounded (a grounded income basin suppresses jumping).
        if not self.charging:
            if airborne and not self.took_off:  # READY: one-time bonus for launching from a full stance
                self.took_off = True
                terms["r_stance"] = W_STANCE * (self.prev_n_contact / 6.0)
            if not self.landed:
                if flying:
                    # FLIGHT (first only): body genuinely airborne (not leg-retraction).
                    # Reward apex height + commanded-direction speed; count airtime.
                    self.was_airborne = True
                    dir_vel = float(np.dot(self.sim.data.qvel[0:2], self.world_dir))
                    terms["r_air"] = (W_AIR + W_AIR_H * max(0.0, jh)
                                      + W_AIRVEL * self.curriculum * float(np.clip(dir_vel, 0.0, VMAX)))
                    wdisp = self.sim.data.qpos[0:2] - self.release_wpos
                    self.peak_apex = max(self.peak_apex, jh)
                    self.peak_dist = max(self.peak_dist, float(np.dot(wdisp, self.world_dir)))
                    self.air_steps += 1  # airtime (length of the first flight)
                elif self.was_airborne and not airborne:
                    self.landed = True  # feet back on the ground after a real flight -> recovery
            else:
                # RECOVERY (~1 s, stabilizing steps/hops ALLOWED): dense reward for being upright,
                # on its feet, near stand height. Sooner + more of the window recovered -> more reward.
                terms["r_recover"] = (W_RECOVER * (n_contact / 6.0)
                                      * np.exp(-RECOVER_LEVEL_K * (grav[0] ** 2 + grav[1] ** 2))
                                      * np.exp(-RECOVER_H_K * (h - STAND_Z) ** 2))
        self.prev_n_contact = n_contact

        reward = sum(v for k, v in terms.items() if k.startswith(("r_", "p_", "alive")))
        terminated = bool(grav[2] > 0.0 or h < 0.012)  # only a true flip / body-slam fails
        if terminated:
            reward -= CRASH_PENALTY
        terms["dist"] = self.peak_dist
        terms["apex"] = self.peak_apex
        terms["charging"] = float(self.charging)
        return reward, terminated, terms


def make_charge_jump_env(episode_seconds=3.0, seed=0, action_mode="joint", randomize=False):
    """Factory for SubprocVecEnv (must be picklable / module-level)."""
    def _thunk():
        return HexapodChargeJumpEnv(episode_seconds=episode_seconds, seed=seed,
                                    action_mode=action_mode, randomize=randomize)
    return _thunk


def make_body_jump_env(episode_seconds=3.0, seed=0, randomize=False):
    """Charge-jump env with the 6-D body-kinematics action (module-level for picklability)."""
    def _thunk():
        return HexapodChargeJumpEnv(episode_seconds=episode_seconds, seed=seed,
                                    action_mode="body", randomize=randomize)
    return _thunk
