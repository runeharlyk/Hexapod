"""Leg-to-leg clearance of a pose: each leg as a chain of capsules, the least surface gap over the leg pairs that can
meet, and whether that gap keeps the margin. firmware/include/leg_clearance.h ports it; both are checked against the
shared fixtures in platform_shared/leg_clearance_fixtures.txt (gen_leg_clearance_fixtures.py), so they cannot drift.

Each leg is the chain mount -> hip -> knee -> tip from its joint angles, the coxa, femur and tibia segments, with the
foot a zero-length segment at the tip. Two legs' clearance is the least exact segment-segment distance between their
segments less the two radii, negative when they overlap.
"""

from __future__ import annotations

import numpy as np

from src.robot.firmware_gait import Kinematics

SEGMENTS = ("coxa", "femur", "tibia", "foot")
# Capsule radii in mm: each the larger of the MuJoCo model's (build_model.py: coxa 8, femur 7, tibia 5, the foot sphere
# 6) and half the part's thickness across the leg's plane, the way a neighbouring leg comes at it, measured from the
# meshes the app draws (app/static/hex) and the MG92B's size (TowerPro: 22.8 x 12 x 31 mm,
# https://www.towerpro.com.tw/product/mg92b/):
# - coxa 20.8: half the 41.6 x 41.6 mm cross-section of the coxa bracket that carries the femur servo (joint.stl);
# - femur 15.5: half the MG92B's 31 mm along its output shaft, the femur's pitch axis, which the femur bracket
#   (middle_joint.stl) matches at 30.5 mm across; its end cap holds the tibia's 16 mm horn boss at the knee;
# - tibia 5: the model's, over half the tibia blade's 5 to 7 mm (foot.stl); the blade's up to 40 mm depth within the
#   leg's plane is not modelled;
# - foot 6: the model's foot sphere, which holds foot.stl's 9 x 4 mm tip.
# None of these is measured on the robot.
LINK_RADIUS_MM = {"coxa": 20.8, "femur": 15.5, "tibia": 5.0, "foot": 6.0}
_RADIUS_SUM = np.array([[LINK_RADIUS_MM[a] + LINK_RADIUS_MM[b] for b in SEGMENTS] for a in SEGMENTS])
# The least surface clearance a pose keeps: an allowance, chosen and not measured, for the round capsules against the
# real parts and for the servos' tracking error.
LEG_CLEARANCE_MM = 5.0
# Ring order 0-1-2-5-4-3: same-side neighbours, plus the front and rear pairs across the body.
NEIGHBOUR_PAIRS = ((0, 1), (1, 2), (3, 4), (4, 5), (0, 3), (2, 5))
KIN = Kinematics()


def segment_distance(p0, p1, q0, q1):
    """Exact distance between segments p0-p1 and q0-q1, broadcast over leading axes (Ericson 5.1.9).

    As the firmware's leg_clearance::segmentDistance: the skew parameter (b f - c e) / (a e - b^2) is solved as
    -(n . (r x d2)) / (n . n) with n = d1 x d2, a segment under 1e-3 mm is a point, and segments within 1e-6 rad of
    parallel are parallel.
    """
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a = np.einsum("...i,...i", d1, d1)
    e = np.einsum("...i,...i", d2, d2)
    b = np.einsum("...i,...i", d1, d2)
    c = np.einsum("...i,...i", d1, r)
    f = np.einsum("...i,...i", d2, r)
    near_zero, near_parallel = 1e-6, 1e-12
    p_point, q_point = a <= near_zero, e <= near_zero
    safe_a = np.where(p_point, 1.0, a)
    safe_e = np.where(q_point, 1.0, e)
    n = np.cross(d1, d2)
    denom = np.einsum("...i,...i", n, n)
    skew = denom > near_parallel * a * e
    numerator = -np.einsum("...i,...i", n, np.cross(r, d2))
    s = np.where(skew, np.clip(numerator / np.where(skew, denom, 1.0), 0.0, 1.0), 0.0)
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


def segment_pair_distances(chain_a: np.ndarray, chain_b: np.ndarray) -> np.ndarray:
    """Centre-line distances (..., 4, 4) between every segment of chain a and every segment of chain b."""
    a = np.concatenate([chain_a, chain_a[..., 3:, :]], axis=-2)   # the tip again closes the foot segment
    b = np.concatenate([chain_b, chain_b[..., 3:, :]], axis=-2)
    pa0, pa1 = a[..., :4, None, :], a[..., 1:, None, :]
    pb0, pb1 = b[..., None, :4, :], b[..., None, 1:, :]
    return segment_distance(pa0, pa1, pb0, pb1)


def surface_clearance(centre: np.ndarray) -> np.ndarray:
    """Surface clearances (..., 4, 4) from segment_pair_distances' centre-line distances."""
    return centre - _RADIUS_SUM


def pair_clearance(chain_a: np.ndarray, chain_b: np.ndarray) -> tuple[float, float, tuple[str, str]]:
    """(centre clearance, surface clearance, the segment pair with the least surface clearance)."""
    centre = segment_pair_distances(chain_a, chain_b)
    surface = surface_clearance(centre)
    i, j = np.unravel_index(np.argmin(surface), surface.shape)
    return float(centre.min()), float(surface[i, j]), (SEGMENTS[i], SEGMENTS[j])


def closest_legs(angles_deg, kin: Kinematics = KIN) -> tuple[tuple[int, int], float]:
    """The pair of NEIGHBOUR_PAIRS with the least surface clearance of 18 joint angles in degrees, coxa, femur and tibia
    per leg in leg order, and that clearance in mm, as the firmware's leg_clearance::closest."""
    chains = leg_chains(kin, np.radians(np.asarray(angles_deg, dtype=float)).reshape(6, 3))
    pairs = np.array(NEIGHBOUR_PAIRS)
    per_pair = surface_clearance(segment_pair_distances(chains[pairs[:, 0]], chains[pairs[:, 1]])).min(axis=(1, 2))
    k = int(np.argmin(per_pair))
    return NEIGHBOUR_PAIRS[k], float(per_pair[k])


def leg_clearance(angles_deg, kin: Kinematics = KIN) -> float:
    """The least surface clearance in mm over NEIGHBOUR_PAIRS, as the firmware's legClearance."""
    return closest_legs(angles_deg, kin)[1]


def legs_clear(angles_deg, kin: Kinematics = KIN) -> bool:
    """Whether every leg pair keeps LEG_CLEARANCE_MM, as the firmware's legsClear."""
    return leg_clearance(angles_deg, kin) >= LEG_CLEARANCE_MM
