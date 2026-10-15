"""The measured stick -> velocity model: it must still describe the simulation, and invert it correctly."""
import numpy as np
import pytest

import measure_speed_model as msm
from src.hexapod import UnmodelledGait
from src.hexapod import speed_model as sm
from src.hexapod.backends.base import ZERO, Gait, Mode, Velocity
from src.platform_shared import message_pb2 as pb

MODELLED = [Gait.TRI_GATE, Gait.BI_GATE, Gait.WAVE, Gait.RIPPLE, Gait.TUNED]


def test_the_enums_are_the_wire_values():
    assert {m.name: m.value for m in Mode} == dict(pb.ModesEnum.items())
    assert {g.name: g.value for g in Gait} == dict(pb.GaitEnum.items())


# The table is generated, so it goes stale when the gait, the model or the loop changes; regenerate it then.
@pytest.mark.parametrize("gait, axis, stick", [
    (Gait.TRI_GATE, "vx", 0.5), (Gait.TRI_GATE, "vy", -0.3), (Gait.TRI_GATE, "yaw_rate", 0.7),
    (Gait.TUNED, "vx", 0.4), (Gait.RIPPLE, "vy", 0.9),
])
def test_the_table_still_describes_the_simulation(gait, axis, stick):
    table = sm._axes(gait)[axis]
    assert msm.measure(int(gait), axis, stick) == pytest.approx(table.value(stick), abs=2e-3)


# What the table says about the axes, so a regenerated table that flips one is noticed rather than obeyed.
@pytest.mark.parametrize("gait", MODELLED)
def test_left_and_counter_clockwise_come_from_positive_lx_and_rx(gait):
    sticks = sm.sticks_for(gait, Velocity(0.02, 0.02, 0.2))
    assert sticks.lx > 0 and sticks.ly > 0 and sticks.rx > 0
    back = sm.sticks_for(gait, Velocity(-0.02, -0.02, -0.2))
    assert back.lx < 0 and back.ly < 0 and back.rx < 0


@pytest.mark.parametrize("gait", MODELLED)
def test_a_velocity_within_the_top_comes_back_from_its_sticks(gait):
    top = sm.max_velocity(gait)
    for fraction in np.linspace(-1.0, 1.0, 201):
        wanted = Velocity(fraction * top.vx, fraction * top.vy, fraction * top.yaw_rate)
        got = sm.velocity_of(gait, sm.sticks_for(gait, wanted))
        assert (got.vx, got.vy, got.yaw_rate) == pytest.approx((wanted.vx, wanted.vy, wanted.yaw_rate), abs=1e-4)


# TRI_GATE's leftward speed peaks at lx = 0.9 and falls at full deflection (measured): asking for more than the peak
# must give the peak, not the larger, slower stick.
def test_a_velocity_beyond_the_top_gives_the_peak_not_a_slower_full_stick():
    table = sm._axes(Gait.TRI_GATE)["vy"]
    assert abs(table.value(-1.0)) < abs(table.value(-0.9))
    sticks = sm.sticks_for(Gait.TRI_GATE, Velocity(0.0, -5.0, 0.0))
    assert sticks.lx == pytest.approx(-0.9)
    assert sm.velocity_of(Gait.TRI_GATE, sticks).vy == pytest.approx(table.value(-0.9))


def test_the_top_speed_is_reachable_both_ways():
    top = sm.max_velocity(Gait.TRI_GATE)
    table = sm._axes(Gait.TRI_GATE)["vx"]
    assert top.vx == pytest.approx(min(table.value(1.0), -table.value(-1.0)))


def test_a_gait_without_a_model_is_refused_but_can_still_stop():
    with pytest.raises(UnmodelledGait):
        sm.sticks_for(Gait.AUTO, Velocity(0.1, 0, 0))
    with pytest.raises(UnmodelledGait):
        sm.max_velocity(None)
    assert sm.sticks_for(None, ZERO) == sm.CENTRED
    assert sm.velocity_of(Gait.AUTO, sm.CENTRED) == ZERO
