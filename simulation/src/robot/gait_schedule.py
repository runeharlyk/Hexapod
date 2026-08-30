"""Parameterized open-loop gait: command -> gait engine settings, as a searchable vector.

`analytic_gait_action` in the env is the firmware's hand-written command->gait map with a fixed
tripod<->bipod blend. That map is what the residual policies sit on top of, but it cannot express
things the policy CAN do -- raise the body, change the duty factor, or re-phase the legs into a
wave/ripple gait. Benchmarking a policy against it therefore compares unequal authority and
flatters the policy.

A GaitSchedule is the same map with those degrees of freedom exposed, so a per-terrain search can
produce the *best open-loop gait for that terrain* and the policy has to beat that instead.

Leg phase pattern
-----------------
Legs are ordered [RF, RM, RR, LF, LM, LR] (robot faces +Y, +X is right). Rather than six free
phases -- which mostly generate gaits with no support polygon -- the pattern is the standard
metachronal family:

    right: phi = [0, lag_r, 2*lag_r]          left: phi = contra + [0, lag_l, 2*lag_l]

which contains every gait the firmware ships (all phases mod 1):

    tripod  lag_r=1/2, lag_l=1/2,  contra=1/2  -> [0, 1/2, 0, 1/2, 0, 1/2]
    bipod   lag_r=1/3, lag_l=2/3,  contra=2/3  -> [0, 1/3, 2/3, 2/3, 1/3, 0]
    wave    lag_r=1/6, lag_l=-1/6, contra=5/6  -> [0, 1/6, 1/3, 5/6, 2/3, 1/2]
    ripple  lag_r=2/3, lag_l=2/3,  contra=1/6  -> [0, 2/3, 1/3, 1/6, 5/6, 1/2]

Duty factor (`duty` = fraction of the cycle in stance) is independent of the pattern here, whereas
the firmware ties it to the gait type. A hexapod needs duty >= 1/2 for a statically stable tripod
and >= 5/6 for a one-leg-at-a-time wave, but "statically stable" is not automatically "fastest over
rocks", so the search is allowed to pick.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import numpy as np

# name -> (low, high) search bounds. Order defines the vector layout.
BOUNDS = {
    # Stride gains: commanded m/s -> normalized stride, at blend 0 and 1. NOTE the inherited naming
    # inversion from gait_coef.json -- `gx` gains the FORWARD axis (a[1] = step_y = body-Y) and `gy`
    # the LATERAL axis (a[0] = step_x = body-X), because the robot faces +Y. Kept as-is so existing
    # gait_coef.json files stay loadable.
    "gx0": (0.15, 0.70),
    "gx1": (0.15, 0.80),
    "gy0": (0.15, 0.70),
    "gy1": (0.15, 0.80),
    "gyaw0": (0.8, 3.5),
    "gyaw1": (0.8, 4.0),
    "blend_speed": (0.10, 0.60),
    # cadence (normalized -1..1 -> PG_PHASE_RATE)
    "pr_base": (-1.0, 1.0),
    "pr_slope": (0.0, 5.0),
    "pr_yaw": (0.0, 3.0),
    # posture / foot trajectory
    "step_height": (-1.0, 1.0),   # normalized -> PG_HEIGHT (10..80 mm)
    "step_depth": (0.0, 10.0),    # mm of stance-phase downward push
    "yaw_comp": (-1.5, 1.5),
    "ride_mm": (-25.0, 25.0),     # + = body raised (more ground clearance, more lift authority)
    # coordination
    "duty": (0.35, 0.90),         # stance fraction of the cycle
    "lag_r": (0.0, 1.0),
    "lag_l": (0.0, 1.0),
    "contra": (0.0, 1.0),
}
NAMES = tuple(BOUNDS)
DIM = len(NAMES)

# Free-phase variant. The metachronal family is a 3-parameter prior on a 5-dimensional space (leg 0
# pins the clock), and sensitivity analysis showed phasing is the most influential thing in the whole
# vector -- `lag_r` alone spans 0.85 of score, more than any other parameter. That is exactly when an
# unmeasured structural prior is worth testing, so this variant searches all five offsets directly.
FREE_OFFSET_NAMES = ("o1", "o2", "o3", "o4", "o5")
BOUNDS_FREE = {**BOUNDS, **{n: (0.0, 1.0) for n in FREE_OFFSET_NAMES}}
NAMES_FREE = tuple(BOUNDS_FREE)
DIM_FREE = len(NAMES_FREE)

# The firmware's shipped tripod is SKEWED -- offsets [0, .52, .08, .58, .16, .66] rather than the
# ideal [0, .5, 0, .5, 0, .5] -- and is not exactly representable in the metachronal family (the
# closest member is lag_r=.52, lag_l=.58, contra=.58, off by up to .08 of a cycle on one leg).
# It is therefore NOT the origin of this search space. The legacy gait remains available exactly,
# as the env's `gait_schedule=None` path, and is always benchmarked as its own arm; the search
# reports whether anything in this family beats it.
TRIPOD = {"duty": 3.1 / 6.0, "lag_r": 0.5, "lag_l": 0.5, "contra": 0.5, "ride_mm": 0.0}

NAMED_PATTERNS = {
    "tripod": (0.5, 0.5, 0.5),
    "bipod": (1 / 3, 2 / 3, 2 / 3),
    "wave": (1 / 6, -1 / 6 % 1.0, 5 / 6),
    "ripple": (2 / 3, 2 / 3, 1 / 6),
}


@dataclass
class GaitSchedule:
    gx0: float = 0.259
    gx1: float = 0.346
    gy0: float = 0.294
    gy1: float = 0.360
    gyaw0: float = 1.602
    gyaw1: float = 2.153
    blend_speed: float = 0.30
    pr_base: float = 0.2
    pr_slope: float = 2.0
    pr_yaw: float = 1.5
    step_height: float = -0.5
    step_depth: float = 0.002
    yaw_comp: float = 0.0
    ride_mm: float = 0.0
    duty: float = 3.1 / 6.0
    lag_r: float = 0.5
    lag_l: float = 0.5
    contra: float = 0.5
    # Explicit per-leg offsets for legs 1..5 (leg 0 defines the clock). NaN = use the metachronal
    # family above. Set only by the --free-offsets search.
    o1: float = float("nan")
    o2: float = float("nan")
    o3: float = float("nan")
    o4: float = float("nan")
    o5: float = float("nan")

    # ------------------------------------------------------------------ vector interface
    @staticmethod
    def from_vector(x) -> "GaitSchedule":
        return GaitSchedule(**{k: float(v) for k, v in zip(NAMES, x)})

    def to_vector(self) -> np.ndarray:
        return np.array([getattr(self, k) for k in NAMES], dtype=float)

    def to_dict(self) -> dict:
        return {f.name: float(getattr(self, f.name)) for f in fields(self)}

    @staticmethod
    def from_dict(d: dict) -> "GaitSchedule":
        s = GaitSchedule()
        for k, v in d.items():
            if hasattr(s, k):
                setattr(s, k, float(v))
        return s

    # ------------------------------------------------------------------ gait engine interface
    def offsets(self) -> np.ndarray:
        """Per-leg phase offsets in [0,1) for legs [RF, RM, RR, LF, LM, LR]."""
        free = np.array([self.o1, self.o2, self.o3, self.o4, self.o5])
        if np.all(np.isfinite(free)):
            return np.concatenate([[0.0], free]) % 1.0
        right = np.array([0.0, self.lag_r, 2.0 * self.lag_r])
        left = self.contra + np.array([0.0, self.lag_l, 2.0 * self.lag_l])
        return np.concatenate([right, left]) % 1.0

    def gait_action(self, cmd) -> np.ndarray:
        """Command -> the same 6 normalized gait actions `analytic_gait_action` produces.

        a = [step_x, step_y, step_angle, step_height, gait_blend, phase_rate]. `gait_blend` is
        retained only as the stride-gain interpolator; the schedule supplies offsets/duty
        directly, so it no longer selects the coordination pattern.
        """
        vx, vy, yaw = float(cmd[0]), float(cmd[1]), float(cmd[2])
        speed = float(np.hypot(vx, vy))
        b = float(np.clip(speed / max(self.blend_speed, 1e-6), 0.0, 1.0))
        gx = self.gx0 + (self.gx1 - self.gx0) * b
        gy = self.gy0 + (self.gy1 - self.gy0) * b
        gyaw = self.gyaw0 + (self.gyaw1 - self.gyaw0) * b
        step_angle = np.clip(yaw / gyaw + self.yaw_comp * vx, -1, 1)
        phase_rate = np.clip(self.pr_base + self.pr_slope * speed + self.pr_yaw * abs(yaw), -1, 1)
        return np.array([np.clip(vy / gy, -1, 1),   # step_x  (body-X) <- lateral vy
                         np.clip(vx / gx, -1, 1),   # step_y  (body-Y) <- forward vx
                         step_angle, self.step_height, b * 2 - 1, phase_rate], dtype=np.float32)


def seed_schedule(coef: dict | None = None) -> GaitSchedule:
    """Search seed: the tuned stride/cadence coefficients from gait_coef.json on an ideal tripod
    with no ride-height offset. The nearest in-family point to what the robot walks with today."""
    s = GaitSchedule(**TRIPOD)
    if coef:
        for k, v in coef.items():
            if hasattr(s, k):
                setattr(s, k, float(v))
    return s


def clip_to_bounds(x) -> np.ndarray:
    lo = np.array([BOUNDS[k][0] for k in NAMES])
    hi = np.array([BOUNDS[k][1] for k in NAMES])
    return np.clip(np.asarray(x, dtype=float), lo, hi)
