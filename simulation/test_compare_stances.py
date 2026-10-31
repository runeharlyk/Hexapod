"""The static stability margin compare_stances.py reports: the distance from the centre of mass, projected on the floor,
to the nearest edge of the support polygon of the feet on it, on the firmware's own walk (spec section 6: the crab keeps
at least 25 mm at its walkable extremes)."""
import numpy as np
import pytest

import compare_stances
from compare_stances import (AXIS_INDEX, CRAB_MIN_MARGIN_MM, POSTURES, STATICALLY_STABLE, StaticMargin, feet_on,
                             static_margins, support_margin)
from src.robot.firmware_gait import STANCE_CRAB, STANCE_DEFAULT, TRI_GATE


def _square(z=-66.0):
    return np.array([[100.0, 100.0, z], [100.0, -100.0, z], [-100.0, -100.0, z], [-100.0, 100.0, z]])


def test_the_margin_is_the_distance_to_the_nearest_edge_of_the_feet_on_the_floor():
    assert support_margin(np.array([0.0, 0.0]), _square()) == pytest.approx(100.0)
    assert support_margin(np.array([30.0, -20.0]), _square()) == pytest.approx(70.0)


def test_a_centre_of_mass_outside_the_support_has_a_negative_margin():
    assert support_margin(np.array([130.0, 0.0]), _square()) == pytest.approx(-30.0)


def test_a_lifted_foot_does_not_support():
    feet = _square()
    feet[0, 2] += 1.0  # the front right foot 1 mm up: the triangle left has its hypotenuse through the origin
    assert support_margin(np.array([0.0, 0.0]), feet) == pytest.approx(0.0, abs=1e-9)
    assert support_margin(np.array([-20.0, -20.0]), feet) == pytest.approx(20.0 * np.sqrt(2))


def test_two_feet_on_the_floor_give_no_static_margin():
    feet = _square()
    feet[[0, 2], 2] += 10.0
    assert support_margin(np.array([0.0, 0.0]), feet) is None


@pytest.fixture(scope="module")
def crab() -> list[StaticMargin]:
    return static_margins(STANCE_CRAB)


def test_every_crab_walk_measured_travels_at_least_half_its_plan_along_its_axis(crab):
    assert len(crab) == len(STATICALLY_STABLE) * len(POSTURES) * 4
    for m in crab:
        axis = AXIS_INDEX[m.axis]
        assert m.planned[axis] != 0.0, m
        assert m.travelled[axis] / m.planned[axis] >= 0.5, m


def test_the_stances_are_told_apart_by_their_coxas():
    crab = [-45.0, 30.0, -90.0, 0.0, 30.0, -90.0, 45.0, 30.0, -90.0, 45.0, 30.0, -90.0, 0.0, 30.0, -90.0, -45.0,
            30.0, -90.0]
    halfway = [c / 2 if i % 3 == 0 else c for i, c in enumerate(crab)]
    assert feet_on(STANCE_CRAB, crab, 0.5) and not feet_on(STANCE_DEFAULT, crab, 0.5)
    assert not feet_on(STANCE_CRAB, halfway, 0.5) and not feet_on(STANCE_DEFAULT, halfway, 0.5)
    assert feet_on(STANCE_DEFAULT, [0.0] * 18, 0.5) and not feet_on(STANCE_CRAB, [0.0] * 18, 0.5)


def test_a_margin_measured_before_the_feet_reach_the_crab_is_refused(monkeypatch):
    monkeypatch.setattr(compare_stances, "POSTURE_S", 0.3)
    with pytest.raises(RuntimeError, match="do not stand on the crab stance"):
        compare_stances._static_margin((STANCE_CRAB, TRI_GATE, POSTURES[0], "sideways right"))


def test_the_crab_keeps_its_static_margin_at_its_walkable_extremes(crab):
    worst = min(crab, key=lambda m: m.margin_mm)
    assert worst.margin_mm >= CRAB_MIN_MARGIN_MM, worst
