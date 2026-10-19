"""Geometric adjacent-leg clearance: segment distance, the leg chain, and the envelope sweep."""

import numpy as np
import pytest

from leg_clearance import (
    LINK_RADIUS_MM,
    NEIGHBOUR_PAIRS,
    leg_chains,
    pair_clearance,
    segment_distance,
    stick_grid,
    sweep,
)
from src.robot.firmware_gait import (
    J1_TO_J2,
    J2_TO_J3,
    J3_TO_TIP,
    TRI_GATE,
    TUNED,
    BodyState,
    GaitController,
    GaitState,
    Kinematics,
    command_to_walk_gait,
    set_gait,
)


def _brute_force_distance(p0, p1, q0, q1, n=801):
    s = np.linspace(0.0, 1.0, n)[:, None]
    a = p0 + s * (p1 - p0)
    b = q0 + s * (q1 - q0)
    return np.min(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=-1))


@pytest.mark.parametrize(
    "p0, p1, q0, q1, expected",
    [
        ((0, 0, 0), (10, 0, 0), (0, 3, 0), (10, 3, 0), 3.0),      # parallel, overlapping
        ((0, 0, 0), (10, 0, 0), (14, 3, 0), (20, 3, 0), 5.0),     # parallel, disjoint: end to end
        ((0, 0, 0), (10, 0, 0), (5, -5, 0), (5, 5, 0), 0.0),      # crossing
        ((0, 0, 0), (10, 0, 0), (5, -5, 2), (5, 5, 2), 2.0),      # skew, interior to interior
        ((0, 0, 0), (10, 0, 0), (13, 4, 0), (13, 4, 0), 5.0),     # degenerate point segment
        ((0, 0, 0), (0, 0, 0), (3, 4, 0), (3, 4, 0), 5.0),        # two points
    ],
)
def test_segment_distance_known_cases(p0, p1, q0, q1, expected):
    d = segment_distance(*(np.array(v, dtype=float) for v in (p0, p1, q0, q1)))
    assert d == pytest.approx(expected, abs=1e-9)


def test_segment_distance_matches_brute_force_and_broadcasts():
    rng = np.random.default_rng(3)
    pts = rng.uniform(-50, 50, size=(200, 4, 3))
    d = segment_distance(pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3])
    assert d.shape == (200,)
    for k in range(200):
        brute = _brute_force_distance(*pts[k])
        assert d[k] <= brute + 1e-9
        assert d[k] == pytest.approx(brute, abs=0.2)


def _straight_leg(mount, direction):
    """A synthetic leg chain laid flat along `direction` from `mount`: mount, hip, knee, tip."""
    u = np.asarray(direction, dtype=float) / np.linalg.norm(direction)
    m = np.asarray(mount, dtype=float)
    return np.array([m, m + J1_TO_J2 * u, m + (J1_TO_J2 + J2_TO_J3) * u,
                     m + (J1_TO_J2 + J2_TO_J3 + J3_TO_TIP) * u])


def test_synthetic_parallel_legs_clearance_is_their_spacing_minus_radii():
    a = _straight_leg((0, 0, 0), (1, 0, 0))
    b = _straight_leg((0, 30, 0), (1, 0, 0))
    centre, surface, (seg_a, seg_b) = pair_clearance(a, b)
    assert centre == pytest.approx(30.0)
    # Every segment pair is 30 mm apart; the thickest pair (coxa-coxa) has the least surface gap.
    assert (seg_a, seg_b) == ("coxa", "coxa")
    assert surface == pytest.approx(30.0 - 2 * LINK_RADIUS_MM["coxa"])


def test_synthetic_crossing_tibiae_collide():
    a = _straight_leg((0, 0, 0), (1, 0, 0))
    # Leg b's tibia runs down through leg a's tibia midpoint.
    knee_b = a[2] + 0.5 * (a[3] - a[2]) + np.array([0.0, 0.0, 40.0])
    tip_b = knee_b - np.array([0.0, 0.0, 80.0])
    b = np.array([knee_b + [0.0, 0.0, 50.0], knee_b + [0.0, 0.0, 45.0], knee_b, tip_b])
    centre, surface, (seg_a, seg_b) = pair_clearance(a, b)
    assert centre == pytest.approx(0.0, abs=1e-9)
    assert surface == pytest.approx(-2 * LINK_RADIUS_MM["tibia"])
    assert (seg_a, seg_b) == ("tibia", "tibia")


def test_synthetic_foot_spheres_touch_before_the_tibiae():
    a = _straight_leg((0, 0, 0), (1, 0, 0))
    reach = J1_TO_J2 + J2_TO_J3 + J3_TO_TIP
    b = _straight_leg((2 * reach + 10.0, 0, 0), (-1, 0, 0))   # tips 10 mm apart, end to end
    centre, surface, segments = pair_clearance(a, b)
    assert centre == pytest.approx(10.0)
    assert segments == ("foot", "foot")
    assert surface == pytest.approx(10.0 - 2 * LINK_RADIUS_MM["foot"])


def _gait_bodies(lx, ly, rx, phases, gait_type=TRI_GATE):
    gait = GaitState(gait_type=gait_type)
    set_gait(gait)
    command_to_walk_gait(lx, ly, rx, 0.0, 0.0, gait)
    ctrl = GaitController()
    for p in phases:
        body = BodyState()
        ctrl.set_phase(p)
        ctrl.generate_feet(gait, body)
        yield body


def _chain(kin, body):
    angles = kin.inverse_kinematics(body, degrees=False).reshape(1, 6, 3)
    return leg_chains(kin, angles)[0], angles[0]


def test_leg_chain_inverts_the_ik_on_reachable_feet():
    kin = Kinematics()
    bodies = [BodyState()] + list(_gait_bodies(0.3, -0.3, 0.2, np.linspace(0, 1, 7, endpoint=False)))
    for body in bodies:
        chain, angles = _chain(kin, body)
        np.testing.assert_allclose(chain[:, 3], body.feet[:, :3], atol=1e-6)
        np.testing.assert_allclose(chain[:, 0, :2], np.column_stack([kin.mount_x, kin.mount_y]))
        lengths = np.linalg.norm(np.diff(chain, axis=1), axis=-1)
        np.testing.assert_allclose(lengths, np.tile([J1_TO_J2, J2_TO_J3, J3_TO_TIP], (6, 1)), atol=1e-9)
        np.testing.assert_allclose(chain[:, 1, 2], 0.0, atol=1e-12)   # the coxa swings in the body plane
        for leg in range(6):
            np.testing.assert_allclose(chain[leg, 3], kin.forward_kinematics_local(leg, *angles[leg]), atol=1e-9)


def test_leg_chain_follows_the_clamped_ik_when_a_foot_is_out_of_reach():
    # Combined full forward, lateral and yaw strokes ask leg 0 of the searched gait, which the speed envelope does not
    # bound, for a foot beyond femur + tibia reach mid-cycle; the IK clamps its acos arguments, so the servos (and the
    # chain the sweep checks) stop short of it.
    kin = Kinematics()
    body = next(_gait_bodies(-1.0, -1.0, 1.0, [0.575], TUNED))
    chain, angles = _chain(kin, body)
    assert np.linalg.norm(chain[0, 3] - body.feet[0, :3]) > 1.0
    np.testing.assert_allclose(chain[0, 3], kin.forward_kinematics_local(0, *angles[0]), atol=1e-9)
    np.testing.assert_allclose(np.linalg.norm(chain[0, 3] - chain[0, 1]), J2_TO_J3 + J3_TO_TIP, atol=1e-6)


def test_neighbour_pairs_follow_the_leg_ring():
    assert set(NEIGHBOUR_PAIRS) == {(0, 1), (1, 2), (3, 4), (4, 5), (0, 3), (2, 5)}


def test_standstill_sweep_is_the_resting_clearance_at_every_phase():
    kin = Kinematics()
    rest = leg_chains(kin, kin.inverse_kinematics(BodyState(), degrees=False).reshape(1, 6, 3))[0]
    expected = min(pair_clearance(rest[a], rest[b])[1] for a, b in NEIGHBOUR_PAIRS)
    result = sweep(TRI_GATE, [(0.0, 0.0, 0.0)], phases=8)
    assert result.min_surface_mm == pytest.approx(expected, abs=1e-9)
    assert result.samples == 8
    assert result.beyond_travel_fraction == 0.0
    assert result.colliding_commands == []


def test_travel_clamp_is_what_keeps_full_reverse_turn_clear():
    # Full reverse plus full yaw asks the searched gait's legs 0 and 1 to swing past their travel until they
    # cross; the servos stop at travel, which leaves the legs apart. The classic gaits' speed envelope keeps
    # their strokes inside travel over this stick grid, so only the searched gait can still ask for this.
    command = [(0.25, -1.0, 1.0)]
    commanded = sweep(TUNED, command, phases=40, clamped=False)
    reached = sweep(TUNED, command, phases=40, clamped=True)
    assert commanded.beyond_travel_fraction > 0.0
    assert commanded.min_surface_mm < 0.0
    assert commanded.worst_pair == (0, 1)
    assert commanded.colliding_commands == command
    assert reached.min_surface_mm > 0.0
    assert reached.colliding_commands == []


def test_tripod_sweep_reports_its_worst_case_on_the_grid():
    result = sweep(TRI_GATE, stick_grid([-1.0, 0.0, 1.0]), phases=12)
    assert result.samples == 27 * 12
    lx, ly, rx = result.worst_command
    assert {lx, ly, rx} <= {-1.0, 0.0, 1.0}
    assert result.worst_pair in NEIGHBOUR_PAIRS
    # The full sweep cannot be clearer than standing still.
    assert result.min_surface_mm <= sweep(TRI_GATE, [(0.0, 0.0, 0.0)], phases=1).min_surface_mm


@pytest.mark.parametrize("gait_type", [TRI_GATE, TUNED])
def test_servos_keep_neighbouring_legs_apart_over_the_stick_envelope(gait_type):
    result = sweep(gait_type, stick_grid([-1.0, -0.5, 0.0, 0.5, 1.0]), phases=20)
    assert result.colliding_commands == []
    assert result.min_surface_mm > 5.0
