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
    step_y: float = 0.0
    step_angle: float = 0.0
    step_speed: float = 1.0
    step_depth: float = 0.002
    stand_frac: float = DEFAULT_STAND_FRAC
    gait_type: int = TRI_GATE
    offset: np.ndarray = field(default_factory=lambda: DEFAULT_OFFSET.copy())


@dataclass
class ReflexConfig:
    """Contact-driven reflexes for the analytic gait (Cruse's Walknet / Espenschied's insect
    reflexes). These are OFF unless a config is attached to the GaitController, so the firmware
    mirror in gait.h stays in parity until they prove themselves.

    They address what a blind gait cannot: touchdown TIMING. The engine assumes ground at a fixed
    body-frame height, so on uneven terrain a leg either lands early (body lurches) or swings
    through air and lands late (support gap). A foot switch knows before the body does.
    """

    # MEASURED (2026-07-26, bench_terrain.py, 6 seeds), on the classical gait with no policy:
    #   reach + ground_follow  raises the curb ceiling 30 -> 40 mm (0/6 -> 6/6) and roughly doubles
    #                          forward reach at a 40 mm step (0.67 -> 1.40 m). A 25 mm taller stance
    #                          alone does NOT do this, so it is genuine reflex behaviour. Cost: flat
    #                          progress -0.12 (p<0.001); the rough-FIELD gain is not significant.
    #   elevator               inert with foot-bottom sensors: a switch under the foot senses ground
    #                          beneath it, not an obstacle in front of the shin. It needs a shin
    #                          bumper or motor-current sensing to fire at all. Left on: it is free.
    #   slow_when_unsupported  never showed a benefit; off.
    search: bool = True            # keep reaching down until the foot actually takes load
    ground_follow: bool = True     # carry that reach through stance, so the leg accommodates
    elevator: bool = True          # contact while the foot should be clear -> lift higher
    slow_when_unsupported: bool = False  # shared-clock brake while too few feet are loaded
    # Holding a leg's phase until touchdown is the textbook version, but it fires on EVERY swing --
    # the planned trajectory returns the foot to exactly nominal height, where it is not yet loaded --
    # so it desynchronizes a gait that was already correct and costs progress on flat ground.
    # Reaching further down needs no phase change, so the hold is off by default.
    hold_phase: bool = False

    # Asymmetric debounce: a touchdown is a real force and is believed immediately, while "no ground
    # here" must survive `debounce` consecutive samples. Debouncing both directions just adds delay
    # to the one event that needs to stop the reach promptly.
    debounce: int = 2              # consecutive OPEN samples before concluding the ground is missing

    # Reach/release form a bang-bang regulator on contact rather than a one-shot probe. Reaching
    # alone overshoots by the sensing + actuation lag (~20 mm at 250 mm/s) and stance then holds that
    # over-extension, which is what made reflexes cost progress on flat ground; backing off while
    # loaded cancels it without having to know the lag.
    search_rate: float = 250.0     # mm/s reach down while the foot should be loaded but is not
    release_rate: float = 90.0     # mm/s back off while it IS loaded
    search_max: float = 30.0       # mm
    hold_max_s: float = 0.08       # s before entering stance anyway -- a dead sensor must never
                                   # deadlock the gait (only used when hold_phase is on)
    elev_step: float = 12.0        # mm of extra step height per bump
    elev_max: float = 35.0
    elev_decay: float = 0.6        # retained fraction after a clean swing
    slow_factor: float = 0.45      # phase-rate multiplier when support is missing

    # The elevator triggers on the PLANNED foot height, not on swing phase: at the start of swing the
    # foot is legitimately still touching the ground it just left, so a phase window fires constantly
    # on flat ground. Height is the physical criterion -- "should this foot be clear right now?".
    elev_trigger: float = 12.0     # mm of planned lift above which contact means an obstacle
    # The reach must fire only when a foot is genuinely LATE, never merely "not loaded yet". Contact
    # force needs a step or two to build even on flat ground, so reaching on instantaneous absence
    # digs every foot into the floor on every stride. Waiting `search_grace` steps past the planned
    # touchdown costs nothing on flat and still leaves most of stance to find a hole.
    search_grace: int = 3          # control steps past planned touchdown before reaching down
    search_window: float = 0.5     # give up once this fraction of stance has elapsed


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
    gait.step_y = ly * 100.0
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

    def __init__(self, reflex: "ReflexConfig | None" = None, arc_stance: bool = False):
        # arc_stance: during a turn, sweep the planted foot along the ARC about the instantaneous
        # centre of rotation instead of the straight chord between the stroke endpoints.
        # `stroke = v + omega x r` (commit c8d2b52) gives each foot the right instantaneous
        # velocity, but _stance_curve then moves it linearly, so mid-stance the foot is off the
        # body's true path by the arc's sagitta: 15 mm for an in-place turn at step_angle 0.8, and
        # 20 mm for forward-plus-turn. A planted foot that cannot follow the body can only slip.
        self.arc_stance = bool(arc_stance)
        self.phase = 0.0
        self.default_position = DEFAULT_FEET.copy()
        self.target_default_position = DEFAULT_FEET.copy()
        self.swing_start_position = DEFAULT_FEET.copy()
        self.foot_was_swinging = [False] * 6

        # contact-reflex state (unused while reflex is None)
        self.reflex = reflex
        self.phase_adj = np.zeros(6)   # per-leg phase hold, so a leg can wait for touchdown
        self.depth = np.zeros(6)       # mm this leg is reaching below nominal to find/keep ground
        self.elev_z = np.zeros(6)      # mm of extra step height from the elevator reflex
        self.hold_t = np.zeros(6)      # s each leg has been holding in late swing
        self.phase_scale = 1.0         # caller multiplies its phase advance by this
        self.open_run = np.zeros(6, dtype=int)    # consecutive samples reading open
        self.closed_run = np.zeros(6, dtype=int)  # consecutive samples reading closed
        self.stance_steps = np.zeros(6, dtype=int)  # control steps since this leg entered stance
        self._last_phase = 0.0

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

    def _phase_params(self, phase, stand_frac, depth, height):
        if phase < stand_frac:
            return phase / stand_frac, self._stance_curve, -depth
        return (phase - stand_frac) / (1 - stand_frac), self._bezier_curve, height

    def _kinematic_params(self, gait: GaitState):
        length = math.hypot(gait.step_x, gait.step_y) * (-1 if gait.step_x < 0 else 1)
        turn_amplitude = math.atan2(gait.step_y, length) * 2 if length != 0 else 0.0
        return length, turn_amplitude

    def generate_feet(self, gait: GaitState, body: BodyState,
                      contacts=None, dt: float = 0.0) -> None:
        """Generate feet at the CURRENT self.phase (does NOT advance phase).

        Used by `phase_gait` control mode where the policy owns the phase. When a ReflexConfig is
        attached and `contacts` (6 per-foot booleans) is supplied, per-leg contact reflexes adjust
        phase and step height; otherwise this is the plain open-loop gait.
        """
        reflexive = self.reflex is not None and contacts is not None
        dphase = math.fmod(self.phase - self._last_phase + 1.0, 1.0) if reflexive else 0.0
        self._last_phase = self.phase

        new_feet = self.default_position.copy()
        n_stance = n_loaded = 0
        for i in range(6):
            # `%` not fmod: phase_adj accumulates negative while a leg's phase is held, and fmod
            # keeps the sign, which would push the phase out of [0,1) and evaluate the stance curve
            # at a negative parameter. The firmware has no phase_adj, so the two agree without it.
            phase = (self.phase + gait.offset[i] + self.phase_adj[i]) % 1.0
            is_swinging = phase >= gait.stand_frac
            if is_swinging and not self.foot_was_swinging[i]:
                self.swing_start_position[i] = self.default_position[i].copy()
            if reflexive and self.foot_was_swinging[i] and not is_swinging:
                self._on_touchdown(i)
            self.foot_was_swinging[i] = is_swinging

            rx, ry = self.default_position[i][0], self.default_position[i][1]
            stroke_x = gait.step_x + gait.step_angle * (-ry)   # translation + (omega x r): radius-scaled turn
            stroke_y = gait.step_y + gait.step_angle * (rx)
            stroke = math.hypot(stroke_x, stroke_y)
            direction = math.atan2(stroke_y, stroke_x)

            height = gait.step_height + (self.elev_z[i] if reflexive else 0.0)
            ph_norm, curve_fn, amp = self._phase_params(
                phase, gait.stand_frac, gait.step_depth, height
            )
            delta = [0.0, 0.0, 0.0]
            curve_fn(stroke / 2.0, direction, amp, ph_norm, delta)
            if self.arc_stance and not is_swinging and abs(gait.step_angle) > 1e-6:
                # _arc_stance_xy returns the OFFSET from the chord, so it adds to the stance
                # displacement rather than replacing it. delta[2] (the depth curve) is unchanged.
                ax, ay = self._arc_stance_xy(gait, rx, ry, ph_norm)
                delta[0] += ax
                delta[1] += ay
            if reflexive:
                delta[2] += self._reflex_z(i, is_swinging, ph_norm, delta[2], bool(contacts[i]),
                                           dphase, dt)
                n_stance += 0 if is_swinging else 1
                n_loaded += 1 if contacts[i] else 0
            for j in range(3):
                new_feet[i][j] = self.default_position[i][j] + delta[j]
            new_feet[i][3] = 1.0

        if reflexive and self.reflex.slow_when_unsupported:
            # Too few feet carrying load for this part of the cycle means the last touchdown did not
            # land where it was planned; slow the shared clock instead of striding on regardless.
            self.phase_scale = self.reflex.slow_factor if n_loaded < n_stance - 1 else 1.0
        body.feet = new_feet

    @staticmethod
    def _arc_stance_xy(gait: GaitState, rx: float, ry: float, ph_norm: float):
        """Foot XY displacement for a stance that follows the body's actual rotation.

        The instantaneous centre of rotation is where the body-frame velocity field vanishes:
        `v + omega x c = 0`, i.e. `c = (-step_y, step_x) / step_angle`. Over one stance the body
        turns by `step_angle`, so relative to the body the foot rotates about `c` by the same angle
        the other way.

        Anchoring matters: the arc is re-centred so its two ENDPOINTS coincide with the linear
        stroke's, and the curvature appears as an outward bulge at mid-stance. Anchoring at
        mid-stance instead would move the endpoints by the full 15 mm and break the swing, which
        still plans a straight chord between them. The residual is the 2.7 % difference between
        arc length `ang*R` and chord `2*R*sin(ang/2)`, which is not worth correcting.

        Computed as a perpendicular offset from the chord rather than by rotating about `c`.
        Rotating about the centre means forming `c = (-step_y, step_x)/ang`, which diverges as the
        turn rate goes to zero -- and `step_angle` is never exactly zero, because `yaw_comp` adds a
        velocity-proportional term even on a straight command. The subtraction of two huge nearly
        equal vectors then loses all precision, which showed up as a spurious effect on a
        forward-only control command. Here the whole correction carries a factor that vanishes
        linearly with `ang`, so straight-line walking is untouched by construction.
        """
        ang = gait.step_angle
        stroke_x = gait.step_x + ang * (-ry)
        stroke_y = gait.step_y + ang * rx
        # (p0 - c) = (stroke_y, -stroke_x) / ang, so the offset from the chord is
        #   [cos(ang*(p-1/2)) - cos(ang/2)] * (p0 - c),
        # whose scalar factor is O(ang) and stays well conditioned.
        k = (math.cos(ang * (ph_norm - 0.5)) - math.cos(0.5 * ang)) / ang
        return k * stroke_y, -k * stroke_x

    def _on_touchdown(self, i):
        """Per-cycle bookkeeping at the swing->stance transition."""
        r = self.reflex
        self.elev_z[i] *= r.elev_decay
        self.phase_adj[i] = 0.0
        self.hold_t[i] = 0.0
        self.open_run[i] = 0
        self.closed_run[i] = 0

    def _reflex_z(self, i, is_swinging, ph_norm, planned_z, contact, dphase, dt):
        """Vertical foot correction (mm) from the contact reflexes, and the phase hold."""
        r = self.reflex
        if contact:
            self.closed_run[i] += 1
            self.open_run[i] = 0
        else:
            self.open_run[i] += 1
            self.closed_run[i] = 0
        loaded = self.closed_run[i] >= 1
        unloaded = self.open_run[i] >= r.debounce

        if is_swinging:
            self.stance_steps[i] = 0
            if r.elevator and loaded and planned_z >= r.elev_trigger:
                self.elev_z[i] = min(r.elev_max, self.elev_z[i] + r.elev_step)
            return 0.0   # mid-swing clearance is never traded away
        self.stance_steps[i] += 1

        late = r.search_grace <= self.stance_steps[i] and ph_norm < r.search_window
        if r.search and late and unloaded:
            self.depth[i] = min(r.search_max, self.depth[i] + r.search_rate * dt)
            if r.hold_phase and self.hold_t[i] < r.hold_max_s:
                self.hold_t[i] += dt
                self.phase_adj[i] -= dphase          # hold this leg; the others keep going
        elif loaded:
            self.depth[i] = max(0.0, self.depth[i] - r.release_rate * dt)
            self.hold_t[i] = 0.0
        return -self.depth[i] if r.ground_follow else 0.0

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
        is_moving = abs(gait.step_x) >= 2 or abs(gait.step_y) >= 2 or gait.step_angle != 0.0
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
