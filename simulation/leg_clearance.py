"""Can neighbouring legs hit each other anywhere in the command envelope?

Six legs at full strafe plus full yaw bring neighbouring legs closest, and the MuJoCo model cannot
answer this: `build_model.py` gives every leg link contype 4 / conaffinity 1, so link-link contacts
are filtered out ((4 & 1) == 0) and the sim would let two legs pass through each other silently.

The check is geometric instead. Over a stick grid (lx, ly, rx) and the whole gait cycle, the firmware
gait port (`src/robot/firmware_gait.py`) produces the feet, its IK the joint angles, and those angles
the leg chain mount -> hip -> knee -> tip. Neighbouring legs' coxa, femur and tibia segments are then
compared by exact segment-segment distance, with the foot sphere as a fourth, zero-length segment.
"Centre" clearance is between link centre lines; "surface" clearance subtracts the capsule and foot
radii `build_model.py` gives the links, and is negative on contact.

The servos stop at their travel (`ServoController::setAngles` clips to `JOINT_LIMIT_DEG`), so the
default check runs on the clamped angles the robot reaches; `commanded` rows use the raw IK angles,
which for the searched gait swing a coxa well past its 31.5 deg travel (the classic gaits' speed
envelope keeps their strokes inside it).

The gait is sampled at the commanded stride with the command already reached, one phase per sample,
so the command ramp and the stop/start easing are not part of the sweep.

  python leg_clearance.py                     # every gait, 9-point stick grid, 40 phases
  python leg_clearance.py --grid 5 --phases 20 --s1 -1 0 1 --h -1 0 1
"""

from __future__ import annotations

import argparse
import itertools
from dataclasses import dataclass

import numpy as np

from src.robot.animation import JOINT_LIMIT_DEG
from src.robot.firmware_gait import (
    BI_GATE,
    RIPPLE,
    TRI_GATE,
    TUNED,
    WAVE,
    BodyState,
    GaitController,
    GaitState,
    Kinematics,
    command_to_walk_gait,
    set_gait,
)

GAITS = {"tripod": TRI_GATE, "bipod": BI_GATE, "wave": WAVE, "ripple": RIPPLE, "tuned": TUNED}
SEGMENTS = ("coxa", "femur", "tibia", "foot")
# Radii of the capsules link_coxa_i / link_femur_i / link_tibia_i and the sphere foot_i in build_model.py,
# in mm. The foot is the zero-length segment at the tip.
LINK_RADIUS_MM = {"coxa": 8.0, "femur": 7.0, "tibia": 5.0, "foot": 6.0}
_RADIUS_SUM = np.array([[LINK_RADIUS_MM[a] + LINK_RADIUS_MM[b] for b in SEGMENTS] for a in SEGMENTS])
# Ring order 0-1-2-5-4-3: same-side neighbours, plus the front and rear pairs across the body.
NEIGHBOUR_PAIRS = ((0, 1), (1, 2), (3, 4), (4, 5), (0, 3), (2, 5))


def segment_distance(p0, p1, q0, q1):
    """Exact distance between segments p0-p1 and q0-q1, broadcast over leading axes (Ericson 5.1.9)."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a = np.einsum("...i,...i", d1, d1)
    e = np.einsum("...i,...i", d2, d2)
    b = np.einsum("...i,...i", d1, d2)
    c = np.einsum("...i,...i", d1, r)
    f = np.einsum("...i,...i", d2, r)
    eps = 1e-12
    p_point, q_point = a <= eps, e <= eps
    safe_a = np.where(p_point, 1.0, a)
    safe_e = np.where(q_point, 1.0, e)
    denom = a * e - b * b
    skew = denom > eps * a * e
    s = np.where(skew, np.clip((b * f - c * e) / np.where(skew, denom, 1.0), 0.0, 1.0), 0.0)
    s = np.where(p_point, 0.0, s)
    t = np.where(q_point, 0.0, (b * s + f) / safe_e)
    s_lo = np.where(p_point, 0.0, np.clip(-c / safe_a, 0.0, 1.0))
    s_hi = np.where(p_point, 0.0, np.clip((b - c) / safe_a, 0.0, 1.0))
    s = np.where(q_point | (t < 0.0), s_lo, np.where(t > 1.0, s_hi, s))
    t = np.clip(t, 0.0, 1.0)
    closest_p = p0 + s[..., None] * d1
    closest_q = q0 + t[..., None] * d2
    return np.linalg.norm(closest_p - closest_q, axis=-1)


def leg_chains(kin: Kinematics, angles: np.ndarray) -> np.ndarray:
    """Joint positions (..., 6, 4, 3) - mount, hip, knee, tip - from IK angles (..., 6, 3) in radians.

    Same frame and conventions as `Kinematics.forward_kinematics_local`: the coxa azimuth is -q0 in
    the leg's mount frame, the femur's absolute pitch q1, the tibia's q1 + q2.
    """
    q0, q1, q2 = angles[..., 0], angles[..., 1], angles[..., 2]
    local_x, local_y = np.cos(q0), -np.sin(q0)
    ux = local_x * kin.ca + local_y * kin.sa
    uy = local_x * kin.sa - local_y * kin.ca
    radial = np.stack([ux, uy, np.zeros_like(ux)], axis=-1)
    up = np.array([0.0, 0.0, 1.0])
    mount = np.broadcast_to(kin.mount_pos, radial.shape)
    hip = mount + (kin.root_j1 + kin.j1_j2) * radial
    knee = hip + kin.j2_j3 * (np.cos(q1)[..., None] * radial + np.sin(q1)[..., None] * up)
    tip = knee + kin.j3_tip * (np.cos(q1 + q2)[..., None] * radial + np.sin(q1 + q2)[..., None] * up)
    return np.stack([mount, hip, knee, tip], axis=-2)


def _segment_pair_distances(chain_a: np.ndarray, chain_b: np.ndarray) -> np.ndarray:
    """Centre-line distances (..., 4, 4) between every segment of chain a and every segment of chain b."""
    a = np.concatenate([chain_a, chain_a[..., 3:, :]], axis=-2)   # the tip again closes the foot segment
    b = np.concatenate([chain_b, chain_b[..., 3:, :]], axis=-2)
    pa0, pa1 = a[..., :4, None, :], a[..., 1:, None, :]
    pb0, pb1 = b[..., None, :4, :], b[..., None, 1:, :]
    return segment_distance(pa0, pa1, pb0, pb1)


def pair_clearance(chain_a: np.ndarray, chain_b: np.ndarray) -> tuple[float, float, tuple[str, str]]:
    """(centre clearance, surface clearance, the segment pair with the least surface clearance)."""
    centre = _segment_pair_distances(chain_a, chain_b)
    surface = centre - _RADIUS_SUM
    i, j = np.unravel_index(np.argmin(surface), surface.shape)
    return float(centre.min()), float(surface[i, j]), (SEGMENTS[i], SEGMENTS[j])


@dataclass
class SweepResult:
    samples: int
    beyond_travel_fraction: float
    min_centre_mm: float
    min_surface_mm: float
    worst_command: tuple[float, float, float]
    worst_phase: float
    worst_pair: tuple[int, int]
    worst_segments: tuple[str, str]
    colliding_commands: list[tuple[float, float, float]]


def stick_grid(values) -> list[tuple[float, float, float]]:
    """Every (lx, ly, rx) stick command with each axis drawn from `values`."""
    return [tuple(float(v) for v in cmd) for cmd in itertools.product(values, repeat=3)]


def _sample_angles(gait_type, commands, phases, s1, h):
    """IK joint angles (N, 6, 3) and their (lx, ly, rx, phase) labels over the commands x cycle."""
    kin = Kinematics()
    phase_grid = np.linspace(0.0, 1.0, phases, endpoint=False)
    angles, labels = [], []
    for lx, ly, rx in commands:
        gait = GaitState(gait_type=gait_type)
        set_gait(gait)
        zm = command_to_walk_gait(lx, ly, rx, 0.0, s1, gait, h)
        ctrl = GaitController()
        for phase in phase_grid:
            body = BodyState(zm=zm)
            ctrl.set_phase(phase)
            ctrl.generate_feet(gait, body)
            angles.append(kin.inverse_kinematics(body, degrees=False).reshape(6, 3))
            labels.append((lx, ly, rx, float(phase)))
    return kin, np.array(angles), labels


def clamp_to_travel(angles: np.ndarray) -> np.ndarray:
    """The joint angles the servos execute: ServoController::setAngles clips each to JOINT_LIMIT_DEG."""
    limit = np.deg2rad(JOINT_LIMIT_DEG)
    return np.clip(angles, -limit, limit)


def sweep(gait_type: int, commands, phases: int, s1: float = 0.0, h: float = 0.0,
          clamped: bool = True) -> SweepResult:
    """Least neighbouring-leg clearance over the (lx, ly, rx) stick commands x gait cycle.

    `clamped` checks the joint angles the servos reach after the firmware's travel clamp; without it,
    the angles the IK commands, which can sweep a coxa well past its travel.
    """
    kin, commanded, labels = _sample_angles(gait_type, commands, phases, s1, h)
    reached = clamp_to_travel(commanded)
    beyond_travel = np.any(reached != commanded, axis=(1, 2))
    chains = leg_chains(kin, reached if clamped else commanded)
    pairs = np.array(NEIGHBOUR_PAIRS)
    centre = _segment_pair_distances(chains[:, pairs[:, 0]], chains[:, pairs[:, 1]])  # (N, pairs, 4, 4)
    surface = centre - _RADIUS_SUM
    per_sample = surface.reshape(len(labels), -1).min(axis=1)
    k = int(np.argmin(per_sample))
    p, i, j = np.unravel_index(np.argmin(surface[k]), surface[k].shape)
    lx, ly, rx, phase = labels[k]
    colliding = sorted({labels[n][:3] for n in np.flatnonzero(per_sample < 0.0)})
    return SweepResult(
        samples=len(labels),
        beyond_travel_fraction=float(beyond_travel.mean()),
        min_centre_mm=float(centre.min()),
        min_surface_mm=float(per_sample[k]),
        worst_command=(lx, ly, rx),
        worst_phase=phase,
        worst_pair=(int(pairs[p][0]), int(pairs[p][1])),
        worst_segments=(SEGMENTS[i], SEGMENTS[j]),
        colliding_commands=colliding,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--gaits", nargs="*", choices=list(GAITS), default=list(GAITS))
    ap.add_argument("--grid", type=int, default=9, help="stick values per axis, spanning -1..1")
    ap.add_argument("--phases", type=int, default=40, help="samples per gait cycle")
    ap.add_argument("--s1", type=float, nargs="*", default=[0.0], help="step-height sticks to sweep")
    ap.add_argument("--h", type=float, nargs="*", default=[0.0], help="body-height sticks (classical gaits)")
    ap.add_argument("--list-hits", action="store_true", help="print every stick command that collides")
    args = ap.parse_args()

    commands = stick_grid(np.linspace(-1.0, 1.0, args.grid))
    radii = "/".join(f"{LINK_RADIUS_MM[s]:.0f}" for s in SEGMENTS)
    print(f"stick grid {args.grid}^3 over [-1, 1], {args.phases} phases per cycle, "
          f"radii {'/'.join(SEGMENTS)} {radii} mm")
    print(f"\n{'gait':8s} {'angles':9s} {'s1':>5s} {'h':>5s} {'travel':>7s} {'centre':>7s} {'surface':>8s} "
          f"{'hits':>5s}  worst (lx, ly, rx) @ phase, legs, segments")
    for name in args.gaits:
        gait_type = GAITS[name]
        heights = [0.0] if gait_type == TUNED else args.h   # TUNED rides at its searched height
        for clamped, s1, h in itertools.product((True, False), args.s1, heights):
            r = sweep(gait_type, commands, args.phases, s1, h, clamped)
            lx, ly, rx = r.worst_command
            print(f"{name:8s} {'clamped' if clamped else 'commanded':9s} {s1:5.2f} {h:5.2f} "
                  f"{100 * r.beyond_travel_fraction:6.1f}% {r.min_centre_mm:7.1f} {r.min_surface_mm:8.1f} "
                  f"{len(r.colliding_commands):5d}  ({lx:+.2f}, {ly:+.2f}, {rx:+.2f}) @ {r.worst_phase:.3f}, "
                  f"legs {r.worst_pair[0]}-{r.worst_pair[1]}, {r.worst_segments[0]}-{r.worst_segments[1]}")
            if args.list_hits:
                for cmd in r.colliding_commands:
                    print(f"    collides at (lx, ly, rx) = ({cmd[0]:+.2f}, {cmd[1]:+.2f}, {cmd[2]:+.2f})")
    print("\nclamped = the angles the servos reach after the firmware's JOINT_LIMIT_DEG clamp; commanded =")
    print("the raw IK angles. travel = samples with any joint commanded beyond its travel. centre = least")
    print("distance between neighbouring link centre lines; surface = centre minus the two link radii,")
    print("negative on contact; hits = stick commands that touch. MuJoCo filters out leg-leg contacts.")


if __name__ == "__main__":
    main()
