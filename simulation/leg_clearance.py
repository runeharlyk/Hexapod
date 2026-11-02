"""Can neighbouring legs hit each other anywhere in the command envelope?

Six legs at full strafe plus full yaw bring neighbouring legs closest, and the MuJoCo model cannot
answer this: `build_model.py` gives every leg link contype 4 / conaffinity 1, so link-link contacts
are filtered out ((4 & 1) == 0) and the sim would let two legs pass through each other silently.

The check is geometric instead. Over a stick grid (lx, ly, rx) and the whole gait cycle, the firmware
gait port (`src/robot/firmware_gait.py`) produces the feet, its IK the joint angles, and those angles
the leg chain mount -> hip -> knee -> tip. Neighbouring legs' coxa, femur and tibia segments are then
compared by exact segment-segment distance, with the foot sphere as a fourth, zero-length segment.
"Centre" clearance is between link centre lines; "surface" clearance subtracts the capsule and foot
radii of `src/robot/clearance.py`, the check the firmware ports, and is negative on contact.

The servos stop at their travel (`ServoController::setAngles` clips to `JOINT_LIMIT_DEG`, and the searched
gait's coxa to `TRAINED_COXA_LIMIT_DEG`), so the default check runs on the clamped angles the robot reaches;
`commanded` rows use the raw IK angles, which for the searched gait swing a coxa well past its 31.5 deg travel
(the classic gaits' speed envelope keeps their strokes inside theirs).

The gait is sampled at the commanded stride with the command already reached, one phase per sample,
so the command ramp and the stop/start easing are not part of the sweep. A classic gait walks the posture
clamped to its stance's walkable region, on that stance's resting feet; the searched gait walks the
default stance at its own height.

  python leg_clearance.py                     # every gait, 9-point stick grid, 40 phases
  python leg_clearance.py --grid 5 --phases 20 --s1 -1 0 1 --h -1 0 1
  python leg_clearance.py --gaits tripod wave --stance crab --stride 1 --fd -1 0 1
"""

from __future__ import annotations

import argparse
import itertools
from dataclasses import dataclass

import numpy as np

from src.robot.animation import JOINT_LIMIT_DEG, TRAINED_COXA_LIMIT_DEG
from src.robot.clearance import (
    LINK_RADIUS_MM,
    NEIGHBOUR_PAIRS,
    SEGMENTS,
    leg_chains,
    segment_pair_distances,
    surface_clearance,
)
from src.robot.firmware_gait import (
    BI_GATE,
    DEFAULT_FEET,
    RIPPLE,
    STANCE_DEFAULT,
    STANCE_NAMES,
    TRI_GATE,
    TUNED,
    WAVE,
    BodyState,
    GaitController,
    GaitState,
    Kinematics,
    clamp_posture,
    command_to_walk_gait,
    set_gait,
    stance_feet,
)
from src.robot.gait_envelope import posture

GAITS = {"tripod": TRI_GATE, "bipod": BI_GATE, "wave": WAVE, "ripple": RIPPLE, "tuned": TUNED}


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


def _resting_feet(gait_type, s1, h, fd, stance):
    """The feet the gait walks about: the stance's at the posture its walk clamps to, or the searched gait's own."""
    if gait_type == TUNED:
        return DEFAULT_FEET.copy()
    return stance_feet(stance, posture(*clamp_posture(s1, h, fd, stance))[2])


def _sample_angles(gait_type, commands, phases, s1, h, fd, stride, stance):
    """IK joint angles (N, 6, 3) and their (lx, ly, rx, phase) labels over the commands x cycle."""
    kin = Kinematics()
    phase_grid = np.linspace(0.0, 1.0, phases, endpoint=False)
    feet = _resting_feet(gait_type, s1, h, fd, stance)
    angles, labels = [], []
    for lx, ly, rx in commands:
        gait = GaitState(gait_type=gait_type)
        set_gait(gait)
        zm = command_to_walk_gait(lx, ly, rx, 0.0, s1, gait, h, fd, stride, stance)
        ctrl = GaitController()
        ctrl.default_position = feet.copy()
        ctrl.target_default_position = feet.copy()
        for phase in phase_grid:
            body = BodyState(zm=zm)
            ctrl.set_phase(phase)
            ctrl.generate_feet(gait, body)
            angles.append(kin.inverse_kinematics(body, degrees=False).reshape(6, 3))
            labels.append((lx, ly, rx, float(phase)))
    return kin, np.array(angles), labels


def clamp_to_travel(angles: np.ndarray, gait_type: int) -> np.ndarray:
    """The joint angles the servos execute: ServoController::setAngles clips each to JOINT_LIMIT_DEG, and the searched
    gait's coxa to TRAINED_COXA_LIMIT_DEG, the travel it was searched in."""
    limit = np.array(JOINT_LIMIT_DEG)
    if gait_type == TUNED:
        limit[:, 0] = TRAINED_COXA_LIMIT_DEG
    limit = np.deg2rad(limit)
    return np.clip(angles, -limit, limit)


def sweep(gait_type: int, commands, phases: int, s1: float = 0.0, h: float = 0.0, clamped: bool = True,
          fd: float = 0.0, stride: float = 0.0, stance: int = STANCE_DEFAULT) -> SweepResult:
    """Least neighbouring-leg clearance over the (lx, ly, rx) stick commands x gait cycle, at the posture, stride and
    stance.

    `clamped` checks the joint angles the servos reach after the firmware's travel clamp; without it,
    the angles the IK commands, which can sweep a coxa well past its travel.
    """
    kin, commanded, labels = _sample_angles(gait_type, commands, phases, s1, h, fd, stride, stance)
    reached = clamp_to_travel(commanded, gait_type)
    beyond_travel = np.any(reached != commanded, axis=(1, 2))
    chains = leg_chains(kin, reached if clamped else commanded)
    pairs = np.array(NEIGHBOUR_PAIRS)
    centre = segment_pair_distances(chains[:, pairs[:, 0]], chains[:, pairs[:, 1]])  # (N, pairs, 4, 4)
    surface = surface_clearance(centre)
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
    ap.add_argument("--fd", type=float, nargs="*", default=[0.0], help="feet-distance sticks (classical gaits)")
    ap.add_argument("--stride", type=float, default=0.0, help="stride slider: -1 shortest, 0 fastest, +1 longest")
    ap.add_argument("--stance", choices=STANCE_NAMES, default=STANCE_NAMES[STANCE_DEFAULT],
                    help="the classical gaits' feet")
    ap.add_argument("--list-hits", action="store_true", help="print every stick command that collides")
    args = ap.parse_args()

    commands = stick_grid(np.linspace(-1.0, 1.0, args.grid))
    radii = "/".join(f"{LINK_RADIUS_MM[s]:g}" for s in SEGMENTS)
    stance = STANCE_NAMES.index(args.stance)
    print(f"stick grid {args.grid}^3 over [-1, 1], {args.phases} phases per cycle, stride {args.stride:+.2f}, "
          f"{args.stance} stance, radii {'/'.join(SEGMENTS)} {radii} mm")
    print(f"\n{'gait':8s} {'angles':9s} {'s1':>5s} {'h':>5s} {'fd':>5s} {'travel':>7s} {'centre':>7s} "
          f"{'surface':>8s} {'hits':>5s}  worst (lx, ly, rx) @ phase, legs, segments")
    for name in args.gaits:
        gait_type = GAITS[name]
        # TUNED rides at its searched height on the default stance
        heights, distances = ([0.0], [0.0]) if gait_type == TUNED else (args.h, args.fd)
        for clamped, s1, h, fd in itertools.product((True, False), args.s1, heights, distances):
            r = sweep(gait_type, commands, args.phases, s1, h, clamped, fd, args.stride,
                      STANCE_DEFAULT if gait_type == TUNED else stance)
            lx, ly, rx = r.worst_command
            print(f"{name:8s} {'clamped' if clamped else 'commanded':9s} {s1:5.2f} {h:5.2f} {fd:5.2f} "
                  f"{100 * r.beyond_travel_fraction:6.1f}% {r.min_centre_mm:7.1f} {r.min_surface_mm:8.1f} "
                  f"{len(r.colliding_commands):5d}  ({lx:+.2f}, {ly:+.2f}, {rx:+.2f}) @ {r.worst_phase:.3f}, "
                  f"legs {r.worst_pair[0]}-{r.worst_pair[1]}, {r.worst_segments[0]}-{r.worst_segments[1]}")
            if args.list_hits:
                for cmd in r.colliding_commands:
                    print(f"    collides at (lx, ly, rx) = ({cmd[0]:+.2f}, {cmd[1]:+.2f}, {cmd[2]:+.2f})")
    print("\nclamped = the angles the servos reach after the firmware's travel clamp; commanded =")
    print("the raw IK angles. travel = samples with any joint commanded beyond its travel. centre = least")
    print("distance between neighbouring link centre lines; surface = centre minus the two link radii,")
    print("negative on contact; hits = stick commands that touch. MuJoCo filters out leg-leg contacts.")


if __name__ == "__main__":
    main()
