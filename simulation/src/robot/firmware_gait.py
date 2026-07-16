"""NumPy port of the FIRMWARE gait + inverse kinematics.

This is a faithful port of `firmware/include/kinematics.h` and `firmware/include/gait.h`
(NOT the older `simulation/src/robot/{kinematics,gait}.py`, which diverge from what runs
on the robot). The firmware is the sim-to-real deployment target, so the learned policy's
residual is added on top of *these* joint angles.

Units match the firmware: positions in millimetres, angles returned in DEGREES from the
IK (the firmware writes degrees to the servos); helpers below also expose radians for MuJoCo.

Authoritative constants are duplicated from:
  - firmware/include/kinematics.h  (HexapodConfig hexapodConfig, get_transformation_matrix, IK)
  - firmware/include/gait.h        (default_offset, BEZIER_*, GaitController::step)
  - firmware/include/motion.h      (CommandMsg -> gait_state mapping)
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import numpy as np

# --- config (firmware/include/kinematics.h :: hexapodConfig == simulation/config.json) ---
MOUNT_X = np.array([44.82, 61.03, 44.82, -44.82, -61.03, -44.82])
MOUNT_Y = np.array([74.82, 0.0, -74.82, 74.82, 0.0, -74.82])
MOUNT_ANGLE_DEG = np.array([45.0, 0.0, -45.0, -225.0, -180.0, -135.0])
ROOT_TO_J1 = 0.0
J1_TO_J2 = 38.0
J2_TO_J3 = 54.06
J3_TO_TIP = 97.0

# Resting foot positions (firmware/include/gait.h :: defaultPosition, also motion.h base_feet_pos)
DEFAULT_FEET = np.array(
    [
        [122.0, 152.0, -66.0, 1.0],
        [171.0, 0.0, -66.0, 1.0],
        [122.0, -152.0, -66.0, 1.0],
        [-122.0, 152.0, -66.0, 1.0],
        [-171.0, 0.0, -66.0, 1.0],
        [-122.0, -152.0, -66.0, 1.0],
    ]
)

STAND_HEIGHT_MM = 66.0  # |z| of resting feet; expected body clearance

# --- gait constants (firmware/include/gait.h) ---
DEFAULT_OFFSET = np.array([0.0, 0.52, 0.08, 0.58, 0.16, 0.66])
DEFAULT_STAND_FRAC = 3.1 / 6.0

# Coordination patterns for continuous gait blending: tripod (stable/slow, 50% duty) <-> bipod
# (dynamic/fast, ~35% duty). A gait_blend in [0,1] lerps between them so the policy can pick a
# more tripod-like gait when slow and a more bipod-like gait when fast.
TRI_OFFSET = DEFAULT_OFFSET
TRI_STAND_FRAC = DEFAULT_STAND_FRAC
BI_OFFSET = np.array([0.0, 1.0 / 3, 2.0 / 3, 2.0 / 3, 1.0 / 3, 0.0])
BI_STAND_FRAC = 2.1 / 6.0

# C(11, k) for k = 0..11
COMBINATORIAL_VALUES = np.array(
    [1, 11, 55, 165, 330, 462, 462, 330, 165, 55, 11, 1], dtype=float
)
BEZIER_STEPS = np.array(
    [-1.0, -1.4, -1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 1.5, 1.5, 1.4, 1.0]
)
BEZIER_HEIGHTS = np.array(
    [0.0, 0.0, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 1.1, 0.0, 0.0]
)

# GaitType enum (firmware/include/message_types.h)
TRI_GATE, BI_GATE, WAVE, RIPPLE = 0, 1, 2, 3


@dataclass
class BodyState:
    omega: float = 0.0  # roll
    phi: float = 0.0  # pitch
    psi: float = 0.0  # yaw
    xm: float = 0.0
    ym: float = 0.0
    zm: float = 0.0
    feet: np.ndarray = field(default_factory=lambda: DEFAULT_FEET.copy())


@dataclass
class GaitState:
    step_height: float = 15.0
    step_x: float = 0.0
    step_z: float = 0.0
    step_angle: float = 0.0
    step_speed: float = 1.0
    step_depth: float = 0.002
    stand_frac: float = DEFAULT_STAND_FRAC
    gait_type: int = TRI_GATE
    offset: np.ndarray = field(default_factory=lambda: DEFAULT_OFFSET.copy())


def set_gait(gait: GaitState) -> None:
    """Port of GaitController::setGait — fills offset/stand_frac for the gait type."""
    if gait.gait_type == TRI_GATE:
        gait.offset = np.array([0.0, 0.52, 0.08, 0.58, 0.16, 0.66])
        gait.stand_frac = 3.1 / 6.0
    elif gait.gait_type == BI_GATE:
        gait.offset = np.array([0.0, 1 / 3, 2 / 3, 2 / 3, 1 / 3, 0.0])
        gait.stand_frac = 2.1 / 6.0
    elif gait.gait_type == WAVE:
        gait.offset = np.array([0.0, 1 / 6, 2 / 6, 5 / 6, 4 / 6, 3 / 6])
        gait.stand_frac = 5.0 / 6.0
    elif gait.gait_type == RIPPLE:
        gait.offset = np.array([0.0, 4 / 6, 2 / 6, 1 / 6, 5 / 6, 3 / 6])
        gait.stand_frac = 5.0 / 6.0


def command_to_walk_gait(lx, ly, rx, s, s1, gait: GaitState) -> None:
    """Port of MotionService::handleCommand WALK branch (firmware/include/motion.h)."""
    gait.step_x = -lx * 100.0
    gait.step_z = ly * 100.0
    gait.step_angle = rx * 0.8
    gait.step_speed = s + 1.0
    gait.step_height = (s1 + 1.0) * 20.0
    gait.step_depth = 0.002


class Kinematics:
    """Port of firmware/include/kinematics.h :: Kinematics."""

    def __init__(self):
        self.mount_x = MOUNT_X
        self.mount_y = MOUNT_Y
        self.root_j1 = ROOT_TO_J1
        self.j1_j2 = J1_TO_J2
        self.j2_j3 = J2_TO_J3
        self.j3_tip = J3_TO_TIP
        a = np.deg2rad(MOUNT_ANGLE_DEG)
        self.ca = np.cos(a)
        self.sa = np.sin(a)
        self.mount_pos = np.column_stack([self.mount_x, self.mount_y, np.zeros(6)])

    @staticmethod
    def transformation_matrix(b: BodyState) -> np.ndarray:
        """Port of get_transformation_matrix (firmware). w = T @ [x,y,z,1]."""
        co, so = math.cos(b.omega), math.sin(b.omega)
        cp, sp = math.cos(b.phi), math.sin(b.phi)
        cs, ss = math.cos(b.psi), math.sin(b.psi)
        T = np.array(
            [
                [cp * cs, -cp * ss, sp, b.xm],
                [so * sp * cs + ss * co, -so * sp * ss + co * cs, -so * cp, b.ym],
                [so * ss - sp * co * cs, so * cs + sp * ss * co, co * cp, b.zm],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        return T

    def inverse_kinematics(self, b: BodyState, degrees: bool = True) -> np.ndarray:
        """Returns 18 joint angles (6 legs x [coxa, femur, tibia]).

        Faithful port of Kinematics::inverseKinematics; firmware returns degrees.
        """
        T = self.transformation_matrix(b)
        ang = np.zeros((6, 3))
        for i in range(6):
            w = T @ b.feet[i]
            wx = w[0] - self.mount_pos[i][0]
            wy = w[1] - self.mount_pos[i][1]
            wz = w[2] - self.mount_pos[i][2]

            lx = wx * self.ca[i] + wy * self.sa[i]
            ly = wx * self.sa[i] - wy * self.ca[i]
            lz = wz

            dx = lx - self.root_j1
            dy = ly
            a0 = -math.atan2(dy, dx)

            radial = math.hypot(dx, dy) - self.j1_j2
            vertical = lz
            base = math.atan2(vertical, radial)
            lr2 = radial * radial + vertical * vertical
            lr = math.sqrt(lr2)

            c1 = (lr2 + self.j2_j3**2 - self.j3_tip**2) / (2 * self.j2_j3 * lr)
            c2 = (lr2 - self.j2_j3**2 + self.j3_tip**2) / (2 * self.j3_tip * lr)
            c1 = max(-1.0, min(1.0, c1))
            c2 = max(-1.0, min(1.0, c2))
            a1 = math.acos(c1)
            a2 = math.acos(c2)

            ang[i, 0] = a0
            ang[i, 1] = base + a1
            ang[i, 2] = -(a1 + a2)

        ang = ang.flatten()
        return np.rad2deg(ang) if degrees else ang

    def forward_kinematics_local(self, leg: int, q0: float, q1: float, q2: float) -> np.ndarray:
        """FK for one leg in WORLD frame (no body transform), for validating IK round-trip.

        q* are the IK output angles in radians: q0=coxa yaw, q1=femur abs angle,
        q2=tibia relative angle. Inverse of the equations in inverse_kinematics.
        """
        # planar 2-link arm in (radial, vertical); femur abs = q1, tibia abs = q1 + q2
        arm_radial = self.j2_j3 * math.cos(q1) + self.j3_tip * math.cos(q1 + q2)
        arm_vertical = self.j2_j3 * math.sin(q1) + self.j3_tip * math.sin(q1 + q2)
        total_radial = self.j1_j2 + self.root_j1 + arm_radial
        lz = arm_vertical
        # azimuth of leg plane: a0 = -atan2(ly, lx)  =>  atan2(ly,lx) = -a0
        lx = total_radial * math.cos(-q0)
        ly = total_radial * math.sin(-q0)
        # back to world (un-rotate by mount angle), add mount pos
        wx = lx * self.ca[leg] + ly * self.sa[leg]
        wy = lx * self.sa[leg] - ly * self.ca[leg]
        wz = lz
        return np.array([wx + self.mount_pos[leg][0], wy + self.mount_pos[leg][1], wz])


class GaitController:
    """Port of firmware/include/gait.h :: GaitController."""

    def __init__(self):
        self.phase = 0.0
        self.default_position = DEFAULT_FEET.copy()
        self.target_default_position = DEFAULT_FEET.copy()
        self.swing_start_position = DEFAULT_FEET.copy()
        self.foot_was_swinging = [False] * 6

    @staticmethod
    def _stance_curve(length, angle, depth, phase, point):
        step = length * (1.0 - 2.0 * phase)
        point[0] += step * math.cos(angle)
        point[1] += step * math.sin(angle)
        if length != 0.0:
            point[2] = depth * math.cos((math.pi * (point[0] + point[1])) / (2.0 * length))

    @staticmethod
    def _bezier_curve(length, angle, height, phase, point):
        x_polar = math.cos(angle)
        z_polar = math.sin(angle)
        phase_power = 1.0
        inv_phase_power = (1.0 - phase) ** 11
        one_minus_phase = 1.0 - phase
        for i in range(12):
            b = COMBINATORIAL_VALUES[i] * phase_power * inv_phase_power
            point[0] += b * BEZIER_STEPS[i] * length * x_polar
            point[1] += b * BEZIER_STEPS[i] * length * z_polar
            point[2] += b * BEZIER_HEIGHTS[i] * height
            phase_power *= phase
            if one_minus_phase != 0.0:
                inv_phase_power /= one_minus_phase

    @staticmethod
    def _yaw_arc(default_foot_pos, current_pos):
        foot_mag = math.hypot(default_foot_pos[0], default_foot_pos[1])
        foot_dir = math.atan2(default_foot_pos[1], default_foot_pos[0])
        offsets = [
            current_pos[0] - default_foot_pos[0],
            current_pos[2] - default_foot_pos[2],
            current_pos[1] - default_foot_pos[1],
        ]
        offset_mag = math.hypot(offsets[0], offsets[1])
        offset_mod = math.atan2(offset_mag, foot_mag)
        return math.pi / 2 + foot_dir + offset_mod

    def _phase_params(self, phase, stand_frac, depth, height):
        if phase < stand_frac:
            return phase / stand_frac, self._stance_curve, -depth
        return (phase - stand_frac) / (1 - stand_frac), self._bezier_curve, height

    def _kinematic_params(self, gait: GaitState):
        length = math.hypot(gait.step_x, gait.step_z) * (-1 if gait.step_x < 0 else 1)
        turn_amplitude = math.atan2(gait.step_z, length) * 2 if length != 0 else 0.0
        return length, turn_amplitude

    def generate_feet(self, gait: GaitState, body: BodyState) -> None:
        """Generate feet at the CURRENT self.phase (does NOT advance phase).

        Used by `phase_gait` control mode where the policy owns the phase.
        """
        length, turn_amplitude = self._kinematic_params(gait)
        angle = gait.step_angle
        new_feet = self.default_position.copy()
        for i in range(6):
            current_foot = body.feet[i]
            phase = math.fmod(self.phase + gait.offset[i], 1.0)
            is_swinging = phase >= gait.stand_frac

            if is_swinging and not self.foot_was_swinging[i]:
                self.swing_start_position[i] = self.default_position[i].copy()
            self.foot_was_swinging[i] = is_swinging

            ph_norm, curve_fn, amp = self._phase_params(
                phase, gait.stand_frac, gait.step_depth, gait.step_height
            )

            delta_pos = [0.0, 0.0, 0.0]
            curve_fn(length / 2, turn_amplitude, amp, ph_norm, delta_pos)

            delta_rot = [0.0, 0.0, 0.0]
            curve_fn(
                (angle * 180) / math.pi,
                self._yaw_arc(self.default_position[i], current_foot),
                amp,
                ph_norm,
                delta_rot,
            )

            for j in range(3):
                new_feet[i][j] = self.default_position[i][j] + delta_pos[j] + delta_rot[j]
            new_feet[i][3] = 1.0

        body.feet = new_feet

    def advance_phase(self, gait: GaitState, dt: float) -> None:
        """Firmware phase advance (speed scales with step length / turn)."""
        length, _ = self._kinematic_params(gait)
        speed_factor = max(abs(length) / 25.0, abs(gait.step_angle) * 1.5)
        speed = gait.step_speed * min(max(speed_factor, 0.75), 1.5)
        self.phase = math.fmod(self.phase + dt * speed, 1.0)

    def set_phase(self, phase: float) -> None:
        self.phase = math.fmod(phase, 1.0)

    def step(self, gait: GaitState, body: BodyState, dt: float) -> None:
        """Faithful firmware step: ease to default when idle, else advance + generate."""
        is_moving = abs(gait.step_x) >= 2 or abs(gait.step_z) >= 2 or gait.step_angle != 0.0
        if not is_moving:
            for i in range(6):
                for j in range(4):
                    body.feet[i][j] += (self.default_position[i][j] - body.feet[i][j]) * dt * 10.0
            self.phase = 0.0
            return
        self.advance_phase(gait, dt)
        self.generate_feet(gait, body)


if __name__ == "__main__":
    # Self-test: IK <-> FK round trip on the resting pose, and a short gait rollout.
    kin = Kinematics()
    body = BodyState()
    ang = kin.inverse_kinematics(body, degrees=False).reshape(6, 3)
    max_err = 0.0
    for i in range(6):
        foot = kin.forward_kinematics_local(i, ang[i, 0], ang[i, 1], ang[i, 2])
        err = np.linalg.norm(foot - DEFAULT_FEET[i, :3])
        max_err = max(max_err, err)
    print(f"IK/FK round-trip max foot error: {max_err:.4f} mm  (expect ~0)")
    print("Resting joint angles (deg):")
    print(np.round(kin.inverse_kinematics(body), 2).reshape(6, 3))

    gait = GaitState()
    set_gait(gait)
    command_to_walk_gait(lx=0.0, ly=0.5, rx=0.0, s=0.0, s1=0.0, gait=gait)  # walk forward
    body = BodyState()
    gait_obj = GaitController()
    for k in range(400):
        gait_obj.step(gait, body, dt=0.02)
    a = kin.inverse_kinematics(body)
    print(f"\nAfter walk rollout, phase={gait_obj.phase:.3f}, angles finite: {np.all(np.isfinite(a))}")
    print("Sample foot 0 after rollout (mm):", np.round(body.feet[0, :3], 1))
