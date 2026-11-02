"""Writes platform_shared/leg_clearance_fixtures.txt: poses with the leg-to-leg clearance `src/robot/clearance.py` gives
them, which firmware/include/leg_clearance.h must give too (firmware/test/test_leg_clearance) and test_leg_clearance.py
holds to this generator. After a change to the check or its radii, regenerate the file in the same commit:

    uv run python gen_leg_clearance_fixtures.py

The poses: the default stance, the crab (every leg straight out to its side), and for every leg pair that can meet the
two coxas turned toward each other until that pair touches, is 0.1 mm inside the margin, and is 0.1 mm clear of it,
poses whose closest segments are a tibia and the other leg's foot, a foot and the other's tibia, and two feet, plus a
few poses drawn within the joints' travel. The segment cases pin the segment distance on its own, every branch of
Ericson's algorithm: a point against a segment either way and against a point, crossing, skew, parallel and collinear
segments, segments 1e-2 to 1e-7 rad from parallel at a leg's scale, crossing over each other or closing in, and
segments drawn at random.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.robot.animation import JOINT_LIMIT_DEG
from src.robot.clearance import (
    KIN,
    LEG_CLEARANCE_MM,
    LINK_RADIUS_MM,
    NEIGHBOUR_PAIRS,
    closest_legs,
    leg_chains,
    legs_clear,
    pair_clearance,
    segment_distance,
)
from src.robot.firmware_gait import BodyState

FIXTURES = Path(__file__).resolve().parents[1] / "platform_shared" / "leg_clearance_fixtures.txt"
STANCE = KIN.inverse_kinematics(BodyState(), degrees=True)
CRAB_COXA = (-45.0, 0.0, 45.0, 45.0, 0.0, -45.0)
# The middle legs' pairs are reached with the legs stretched out low, so a femur, tibia or foot can be what meets.
STRETCHED = {(1, 2): (10.0, -30.0), (4, 5): (10.0, -30.0)}
RANDOM_POSES = 6
RANDOM_SEED = 30
FOOT_CELLS = (("tibia", "foot"), ("foot", "tibia"), ("foot", "foot"))
FOOT_CELL_SEED = 7
SEGMENT_SEED = 31


@dataclass
class Case:
    name: str
    pair: tuple[int, int]
    clear: bool
    clearance: float
    angles: list[float]


def _case(name: str, angles) -> Case:
    rounded = [round(float(a), 6) + 0.0 for a in angles]
    pair, clearance = closest_legs(rounded)
    return Case(name, pair, legs_clear(rounded), clearance, rounded)


def _pair_pose(pair: tuple[int, int], turn: float) -> np.ndarray:
    """The stance with the pair's coxas turned `turn` degrees toward each other."""
    start = STANCE.copy()
    for leg in pair:
        if pair in STRETCHED:
            start[leg * 3 + 1:leg * 3 + 3] = STRETCHED[pair]
    angles = start.copy()
    for leg, other in (pair, pair[::-1]):
        angles[leg * 3] += _toward(leg, other, start) * turn
    return angles


def _toward(leg: int, other: int, angles) -> float:
    """The sign of a coxa turn that brings `leg`'s hip toward `other`'s."""
    def hip_distance(turn: float) -> float:
        turned = np.array(angles, dtype=float)
        turned[leg * 3] += turn
        chains = leg_chains(KIN, np.radians(turned).reshape(6, 3))
        return float(np.linalg.norm(chains[leg, 1] - chains[other, 1]))
    return 1.0 if hip_distance(10.0) < hip_distance(-10.0) else -1.0


def _gap(pair: tuple[int, int], angles) -> float:
    chains = leg_chains(KIN, np.radians(angles).reshape(6, 3))
    return pair_clearance(chains[pair[0]], chains[pair[1]])[1]


def _turn_to(pair: tuple[int, int], target: float) -> float:
    """The turn at which the pair's clearance first falls to `target`, by bisection within the first bracket."""
    lo = 0.0
    while _gap(pair, _pair_pose(pair, lo + 1.0)) > target:
        lo += 1.0
    hi = lo + 1.0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _gap(pair, _pair_pose(pair, mid)) > target else (lo, mid)
    return (lo + hi) / 2


def closest_cell(angles) -> tuple[str, str]:
    """The segments of the closest pair that are closest, the first leg's first."""
    (a, b), _ = closest_legs(angles)
    chains = leg_chains(KIN, np.radians(np.asarray(angles, dtype=float)).reshape(6, 3))
    return pair_clearance(chains[a], chains[b])[2]


def _foot_cells() -> list[Case]:
    """Legs 1 and 2 turned toward each other with their femurs and tibias drawn at random until the closest segments of
    each of FOOT_CELLS have turned up, the closest pair being 1-2."""
    rng = np.random.default_rng(FOOT_CELL_SEED)
    found: dict[tuple[str, str], Case] = {}
    while len(found) < len(FOOT_CELLS):
        angles = STANCE.copy()
        for leg, sign in ((1, -1.0), (2, 1.0)):
            angles[leg * 3:leg * 3 + 3] = (sign * rng.uniform(15.0, JOINT_LIMIT_DEG[leg, 0]),
                                           rng.uniform(-40.0, 60.0), rng.uniform(-JOINT_LIMIT_DEG[leg, 2], 0.0))
        case = _case("", angles)
        cell = closest_cell(case.angles)
        if case.pair == (1, 2) and cell in FOOT_CELLS and cell not in found and -10.0 < case.clearance < 15.0:
            case.name = f"cell_{cell[0]}_{cell[1]}"
            found[cell] = case
    return [found[cell] for cell in FOOT_CELLS]


def _f32(values) -> np.ndarray:
    return np.asarray(values, dtype=np.float32).astype(float)


def segments() -> list[tuple[str, np.ndarray]]:
    """(name, p0 p1 q0 q1 as 12 float32 coordinates in mm)."""
    out = [
        ("point_segment", [5, 4, 1, 5, 4, 1, 0, 0, 0, 10, 0, 0]),
        ("segment_point", [0, 0, 0, 10, 0, 0, 13, 4, 0, 13, 4, 0]),
        ("point_point", [0, 0, 0, 0, 0, 0, 3, 4, 0, 3, 4, 0]),
        ("crossing", [0, 0, 0, 10, 0, 0, 5, -5, 0, 5, 5, 0]),
        ("skew", [0, 0, 0, 10, 0, 0, 5, -5, 2, 5, 5, 2]),
        ("skew_end", [0, 0, 0, 10, 0, 0, 14, -5, 3, 14, 5, 3]),
        ("parallel_overlapping", [0, 0, 0, 97, 0, 0, 20, 30, 0, 120, 30, 0]),
        ("parallel_disjoint", [0, 0, 0, 38, 0, 0, 50, 30, 0, 90, 30, 0]),
        ("antiparallel", [0, 0, 0, 54, 0, 0, 80, 12, 5, 30, 12, 5]),
        ("collinear_overlapping", [0, 0, 0, 97, 0, 0, 40, 0, 0, 150, 0, 0]),
        ("collinear_disjoint", [0, 0, 0, 38, 0, 0, 60, 0, 0, 120, 0, 0]),
    ]
    out = [(name, _f32(v)) for name, v in out]
    # A tibia and one 25 mm above it, both 97 mm long, the upper turned `angle` from it about their middles, so the
    # closest points lie inside both: a near-parallel pair whose distance is the skew branch's to get right.
    along, side, up = np.array([0.6, 0.8, 0.0]), np.array([-0.8, 0.6, 0.0]), np.array([0.0, 0.0, 1.0])
    p0 = np.array([30.0, 40.0, -20.0])
    middle = p0 + 48.5 * along
    for exponent in range(2, 8):
        angle = 10.0 ** -exponent
        turned = np.cos(angle) * along + np.sin(angle) * side
        q0, q1 = middle + 25.0 * up - 48.5 * turned, middle + 25.0 * up + 48.5 * turned
        out.append((f"near_parallel_1e-{exponent}", _f32(np.concatenate([p0, p0 + 97.0 * along, q0, q1]))))
    # A tibia beside another 25 mm away, both 97 mm long, closing in by `angle` along their length in a plane tilted out
    # of the body's: the closest points are the far ends, which a parallel shortcut misses by up to 97 mm x angle.
    beside = np.cos(0.4) * side + np.sin(0.4) * up
    for exponent in range(2, 8):
        angle = 10.0 ** -exponent
        q0 = p0 + 25.0 * beside
        q1 = q0 + 97.0 * (np.cos(angle) * along - np.sin(angle) * beside)
        out.append((f"converging_1e-{exponent}", _f32(np.concatenate([p0, p0 + 97.0 * along, q0, q1]))))
    rng = np.random.default_rng(SEGMENT_SEED)
    for k in range(6):
        out.append((f"random_{k}", _f32(rng.uniform(-200.0, 200.0, 12))))
    return out


def segment_distances() -> list[float]:
    rows = np.array([v for _, v in segments()]).reshape(-1, 4, 3)
    return [float(d) for d in segment_distance(rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3])]


def cases() -> list[Case]:
    out = [_case("stance", STANCE)]
    crab = STANCE.copy()
    crab[0::3] = CRAB_COXA
    out.append(_case("crab", crab))
    for pair in NEIGHBOUR_PAIRS:
        a, b = pair
        for kind, target in (("touching", 0.0), ("inside", LEG_CLEARANCE_MM - 0.1), ("clear", LEG_CLEARANCE_MM + 0.1)):
            out.append(_case(f"{kind}_{a}_{b}", _pair_pose(pair, _turn_to(pair, target))))
        out.append(_case(f"through_{a}_{b}", _pair_pose(pair, _turn_to(pair, 0.0) + 5.0)))
    out += _foot_cells()
    rng = np.random.default_rng(RANDOM_SEED)
    limit = JOINT_LIMIT_DEG.reshape(18)
    for k in range(RANDOM_POSES):
        out.append(_case(f"random_{k}", rng.uniform(-limit, limit)))
    return out


def fixture_text() -> str:
    lines = [
        "# Leg-to-leg clearance fixtures, written by simulation/gen_leg_clearance_fixtures.py from src/robot/clearance.py",
        "# and read by simulation/test_leg_clearance.py and firmware/test/test_leg_clearance. Regenerate, never edit.",
        "# radii: coxa femur tibia foot capsule radii, mm. margin: LEG_CLEARANCE_MM.",
        "# case: name, closest pair (two legs), clear (1 or 0), clearance mm, 18 joint angles in degrees (coxa, femur,",
        "# tibia per leg, in leg order).",
        "# segment: name, p0 p1 q0 q1 (12 coordinates, mm, exact as float32), their distance mm.",
        "radii " + " ".join(f"{LINK_RADIUS_MM[s]:g}" for s in ("coxa", "femur", "tibia", "foot")),
        f"margin {LEG_CLEARANCE_MM:g}",
    ]
    for c in cases():
        angles = " ".join(f"{a:.6f}" for a in c.angles)
        lines.append(f"case {c.name} {c.pair[0]} {c.pair[1]} {int(c.clear)} {c.clearance:.6f} {angles}")
    for (name, coords), distance in zip(segments(), segment_distances()):
        lines.append(f"segment {name} " + " ".join(f"{v:.9g}" for v in coords) + f" {distance:.6f}")
    return "\n".join(lines) + "\n"


def read_fixtures(path: Path = FIXTURES) -> tuple[list[float], float, list[Case]]:
    radii, margin, out = [], 0.0, []
    for line in path.read_text().splitlines():
        words = line.split()
        if not words or words[0] == "#":
            continue
        if words[0] == "radii":
            radii = [float(w) for w in words[1:]]
        elif words[0] == "margin":
            margin = float(words[1])
        elif words[0] == "case":
            out.append(Case(words[1], (int(words[2]), int(words[3])), words[4] == "1", float(words[5]),
                            [float(w) for w in words[6:]]))
    return radii, margin, out


def read_segments(path: Path = FIXTURES) -> list[tuple[str, np.ndarray, float]]:
    """(name, p0 p1 q0 q1 as a 4 x 3 array, distance) of every segment line."""
    out = []
    for line in path.read_text().splitlines():
        words = line.split()
        if words and words[0] == "segment":
            out.append((words[1], np.array([float(w) for w in words[2:14]]).reshape(4, 3), float(words[14])))
    return out


if __name__ == "__main__":
    FIXTURES.write_text(fixture_text(), newline="\n")
    print(f"wrote {FIXTURES}")
