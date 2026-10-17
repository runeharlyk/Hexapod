"""Shared MuJoCo runtime for the hexapod, reused by replay + the RL env.

Verified facts:
  - Actuator order is [coxa_i, femur_i, tibia_i for i in 0..5] which is exactly the
    flatten order of Kinematics.inverse_kinematics(...).reshape(6,3).
  - The MJCF joint convention matches the firmware IK 1:1 (sign mapping (1,1,1),
    0.0 mm foot error), so ctrl[joint] = ik_angle_in_radians directly.
  - Physics timestep 0.002 s; policy/gait control at 0.02 s -> frame_skip = 10.
"""

import collections
import os
import numpy as np
import mujoco

from src.robot.firmware_gait import Kinematics, BodyState

MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "resources", "model.xml")
TERRAIN_MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "resources", "model_terrain.xml")
CONTROL_DT = 0.02
JOINT_NAMES = [f"{j}_{i}" for i in range(6) for j in ("coxa", "femur", "tibia")]

# --- servo model (MG92B) ---
# Motor (torque) actuators driven HERE by a cascaded position->velocity->torque loop, mimicking a hobby
# servo's internal controller: position error sets a target velocity (capped at no-load speed), and a
# stiff velocity loop applies up to stall torque to reach it -- full power on any error, a natural
# top-speed cap, and the velocity loop damps position-PD shaking.
# Holding is asymmetric: driving, the joint gets at most its stall torque, but a load that would back-drive
# it has to turn the gear train backwards, so a powered servo resists up to a larger holding torque
# SERVO_HOLD = SERVO_K_HOLD * SERVO_STALL. Modelled as gear friction: the motor delivers up to the mean of
# stall and hold, and a joint friction of half their difference opposes motion. MuJoCo's constraint solver
# applies that friction implicitly, so it sticks and slips without the chatter of a velocity-switched cap;
# being a soft constraint it still lets a held load creep by thousandths of a degree per second.
# Driving nets motor - friction = stall; holding still, being back-driven or braking, motor + friction = hold.
SERVO_KP = 250.0      # position->velocity gain (1/s): commands full no-load speed at ~3 deg error
SERVO_KV = 1.5        # velocity->torque gain (N.m per rad/s): saturates to stall on small vel error
SERVO_J_EFF = 0.004   # kg.m^2, reflected joint inertia; sets the torque-limited STOPPING profile so the
                      # joint brakes to the target with no overshoot/ringing
SERVO_STALL = 0.34    # N.m driving torque envelope, flat (available at any speed); MG92B rated stall @6V
SERVO_K_HOLD = 2.5    # PLACEHOLDER, unverified: holding/stall ratio, estimated 2-3; a bench measurement replaces it
SERVO_HOLD = SERVO_K_HOLD * SERVO_STALL  # N.m the powered servo resists before a load back-drives it
SERVO_NOLOAD = 16.0   # rad/s no-load speed cap (MG92B @6V); the joint's top slew speed


class HexapodSim:
    """Thin wrapper: loads the model, maps IK angles (rad) to actuators, steps physics."""

    def __init__(self, model_path: str = MODEL_PATH):
        self.model = mujoco.MjModel.from_xml_path(os.path.abspath(model_path))
        self.data = mujoco.MjData(self.model)
        self.kin = Kinematics()
        self.frame_skip = int(round(CONTROL_DT / self.model.opt.timestep))

        self.act_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINT_NAMES]
        )
        self.qpos_adr = np.array(
            [
                self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                for n in JOINT_NAMES
            ]
        )
        self.qvel_adr = np.array(
            [
                self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                for n in JOINT_NAMES
            ]
        )
        # servo params (mutable so domain randomization can scale them per episode)
        self.kp, self.kv = SERVO_KP, SERVO_KV
        self.stall, self.noload = SERVO_STALL, SERVO_NOLOAD
        self.k_hold = SERVO_K_HOLD
        self.base_frictionloss = self.model.dof_frictionloss[self.qvel_adr].copy()
        # MuJoCo's frictionloss is a soft constraint that eases the joint toward zero velocity over its solref time
        # constant; at the default 0.02 s it pushes WITH a motion the motor brakes faster than that (a 0.03 deg
        # overshoot on a 20 deg step), so use 2 timesteps, MuJoCo's stiffest stable setting. At the default
        # impedance (solimp 0.9-0.95) a load near either limit slips 1-3 deg in 2 s; 0.999-0.9999 cuts that to
        # under 0.03 deg (stall < load < hold must neither lift nor back-drive the joint).
        self.model.dof_solref[self.qvel_adr, 0] = 2.0 * self.model.opt.timestep
        self.model.dof_solimp[self.qvel_adr, :2] = (0.999, 0.9999)
        self.j_eff = SERVO_J_EFF
        self.deadband = 0.0   # rad: gear-lash / PWM deadband -- no corrective torque within this of target
        self.powered = True
        self.joint_target = self.stand_pose.copy()
        self.imu_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu")
        self._gyro = self._sensor_slice("imu_gyro")
        self._rates = collections.deque(maxlen=64)  # (sim time at its end, gyro rate) of the latest physics steps
        self.foot_site_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"foot_{i}") for i in range(6)]
        )
        self.foot_geom_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"foot_{i}") for i in range(6)]
        )
        self.ground_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        # chassis box: for measuring jump height from the LOWEST point of the body (not the center)
        self.chassis_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "chassis")
        self.chassis_half = self.model.geom_size[self.chassis_geom].copy()
        self._box_corners = self.chassis_half * np.array(
            [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)

    @property
    def stand_pose(self) -> np.ndarray:
        """18 joint angles (rad) for the resting/standby pose."""
        return self.kin.inverse_kinematics(BodyState(), degrees=False)

    @property
    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """(lo, hi) hinge limits (rad) for the 18 joints, in JOINT_NAMES order."""
        ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
        r = self.model.jnt_range[ids]
        return r[:, 0].copy(), r[:, 1].copy()

    def set_servo_scale(self, kp=1.0, stall=1.0, noload=1.0, hold=1.0):
        """Domain-randomization hook: scale servo params for the episode.
        `hold` scales the holding/stall ratio, so the holding torque also follows `stall` (stall=0 is limp)."""
        self.kp = SERVO_KP * kp
        self.stall = SERVO_STALL * stall
        self.noload = SERVO_NOLOAD * noload
        self.k_hold = SERVO_K_HOLD * hold

    def reset_to_stand(self):
        mujoco.mj_resetData(self.model, self.data)
        stand = self.stand_pose
        self.data.qpos[self.qpos_adr] = stand
        self.joint_target = stand.copy()
        mujoco.mj_forward(self.model, self.data)
        self.data.ctrl[self.act_ids] = 0.0  # motor actuators: ctrl is torque, set by the servo loop
        self._rates.clear()

    def set_powered(self, powered: bool) -> None:
        """A deactivated robot's PCA9685s sleep: its servos give no torque, only their gears' own friction, and the
        servo params stay as they are for when it is powered again."""
        self.powered = powered

    def set_joint_targets(self, angles_rad: np.ndarray):
        self.joint_target = np.asarray(angles_rad, dtype=np.float64).copy()

    def _apply_servo(self):
        """PD position loop with a FLAT torque envelope + a speed cap (per physics step).
        Up to +/-stall torque is available at any speed (unlike a raw DC motor's torque-speed droop);
        the servo just can't accelerate past its no-load speed. The motor cap and the gear friction split
        stall and hold as described at SERVO_K_HOLD; both scale with stall, so stall=0 is limp."""
        if not self.powered:
            self.model.dof_frictionloss[self.qvel_adr] = self.base_frictionloss
            self.data.ctrl[self.act_ids] = 0.0
            return
        q = self.data.qpos[self.qpos_adr]
        qd = self.data.qvel[self.qvel_adr]
        err = self.joint_target - q
        if self.deadband > 0.0:  # gear lash / PWM deadband: shrink error toward 0 -> free play near target
            err = np.sign(err) * np.maximum(np.abs(err) - self.deadband, 0.0)
        # cascaded servo: position error -> target velocity -> stiff velocity loop -> torque (<=stall).
        # The target velocity is capped by (a) no-load speed (back-EMF) and (b) the max speed from which
        # the joint can still brake to a stop at the target with stall torque -> NO overshoot/ringing.
        # target velocity capped by no-load speed AND by the max speed from which stall torque can still
        # brake to a stop at the target -> no overshoot/ringing
        ae = np.abs(err)
        v_stop = np.sqrt(2.0 * (self.stall / self.j_eff) * ae)  # torque-limited stopping profile
        vel_des = np.sign(err) * np.minimum(self.kp * ae, np.minimum(v_stop, self.noload))
        self.model.dof_frictionloss[self.qvel_adr] = self.base_frictionloss + 0.5 * (self.k_hold - 1.0) * self.stall
        motor = self.motor_torque_cap
        self.data.ctrl[self.act_ids] = np.clip(self.kv * (vel_des - qd), -motor, motor)

    @property
    def motor_torque_cap(self) -> float:
        """The motor's own torque limit, the mean of stall and hold (see SERVO_K_HOLD)."""
        return 0.5 * (1.0 + self.k_hold) * self.stall

    def step_physics(self, n: int | None = None):
        for _ in range(n if n is not None else self.frame_skip):
            self._apply_servo()  # recompute torque each substep (PD runs at the physics rate)
            mujoco.mj_step(self.model, self.data)
            # The step's sensors were computed at its start, so its rate holds over the step.
            self._rates.append((self.data.time, self.data.sensordata[self._gyro].copy()))

    def body_targets_from_feet(self, body: BodyState) -> np.ndarray:
        return self.kin.inverse_kinematics(body, degrees=False)

    # --- hardware-available state (for the RL observation) ---
    def base_quat(self) -> np.ndarray:
        return self.data.qpos[3:7].copy()  # w, x, y, z

    def base_height(self) -> float:
        return float(self.data.qpos[2])

    def base_yaw(self) -> float:
        """Rotation about world Z (rad, counter-clockwise); 0 is the reset pose, facing world +Y."""
        w, x, y, z = self.data.qpos[3:7]
        return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    def body_tilt_deg(self) -> float:
        """Angle between the body's up axis and world up (deg) -- how far off level the robot is."""
        up_z = self.data.xmat[self.model.site_bodyid[self.imu_site]].reshape(3, 3)[2, 2]
        return float(np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0))))

    def base_vz(self) -> float:
        """World-frame vertical velocity of the base (m/s)."""
        return float(self.data.qvel[2])

    def base_lowest_z(self) -> float:
        """World z of the lowest chassis-box corner. Jump height measured from here (not the base
        center) stays honest under body tilt."""
        pos = self.data.geom_xpos[self.chassis_geom]
        matz = self.data.geom_xmat[self.chassis_geom].reshape(3, 3)[2]  # world z of the box axes
        return float(pos[2] + (self._box_corners @ matz).min())

    def contact_state(self) -> tuple[np.ndarray, float, float]:
        """One pass over the contact list -> (per-foot normal force N, total non-foot ground-contact
        force N, squared horizontal slip speed of planted feet).

        The non-foot term is what a shin or the belly dragging on an obstacle costs; it only became
        observable once the leg links and chassis were given collision masks.
        """
        d = self.data
        foot_force = np.zeros(6)
        nonfoot_force = 0.0
        f6 = np.zeros(6)
        for c in range(d.ncon):
            g1, g2 = int(d.contact[c].geom1), int(d.contact[c].geom2)
            if g1 == self.ground_geom_id:
                other = g2
            elif g2 == self.ground_geom_id:
                other = g1
            else:
                continue
            mujoco.mj_contactForce(self.model, d, c, f6)
            fn = abs(float(f6[0]))
            hit = np.nonzero(self.foot_geom_ids == other)[0]
            if hit.size:
                foot_force[hit[0]] += fn
            else:
                nonfoot_force += fn

        slip = 0.0
        res = np.zeros(6)
        for i in np.nonzero(foot_force)[0]:
            mujoco.mj_objectVelocity(
                self.model, d, mujoco.mjtObj.mjOBJ_SITE, int(self.foot_site_ids[i]), res, 0
            )
            slip += res[3] ** 2 + res[4] ** 2  # world-frame x,y linear velocity
        return foot_force, float(nonfoot_force), float(slip)

    def feet_in_contact(self) -> np.ndarray:
        """Boolean per-foot mask of feet touching the ground (jump env uses it to detect flight)."""
        return self.contact_state()[0] > 0.0

    def gyro(self) -> np.ndarray:
        return self.data.sensordata[self._gyro].copy()

    def mean_gyro(self, start: float, end: float) -> np.ndarray:
        """The gyro's rate averaged over the sim time from `start` to `end`, as the IMU's low-pass reports it, from the
        latest physics steps; the rate now before any step. One step's rate, read once per control period, aliases the
        body's vibration: integrated so, a 90 deg turn on WAVE read 91.2 deg for a true 83.0 (measured)."""
        total, covered = np.zeros(3), 0.0
        for step_end, rate in self._rates:
            overlap = min(step_end, end) - max(step_end - self.model.opt.timestep, start)
            if overlap > 0:
                total += rate * overlap
                covered += overlap
        return total / covered if covered > 0 else self.gyro()

    def _sensor_slice(self, name):
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        adr = self.model.sensor_adr[sid]
        dim = self.model.sensor_dim[sid]
        return slice(adr, adr + dim)

    def actuator_power(self) -> float:
        """Mechanical power |torque * joint_velocity| summed over actuators (W); the cost-of-transport
        signal for trading step length against cadence."""
        return float(np.sum(np.abs(self.data.actuator_force * self.data.actuator_velocity)))

    def foot_slip_sq(self) -> float:
        """Sum of squared horizontal foot speed over planted feet; penalizing it discourages the
        sim-only slip exploit and improves transfer."""
        return self.contact_state()[2]
