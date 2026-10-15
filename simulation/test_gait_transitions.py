"""Starting and leaving a walk move no joint with a jolt, in the port of gait.h.

Mirrors the entry tests in firmware/test/test_gait/test_gait.cpp: a walk entered from a settled
stand, and one entered after the gait was swapped at a standstill, must not move any joint by more
than ENTRY_STEP_BOUND times the largest per-tick step of steady walking at the same command.
Without the swing replay of GaitController.step, the legs a fresh cycle finds mid-swing left the
floor in one tick at up to 3,100 deg/s.

Mirrors the leaving tests too: once the stop ramp is inside the deadband the feet are eased home
along a minimum-jerk path, where the exponential pull it replaced moved a joint 0.82 deg in a tick.
"""

import copy
import itertools

import numpy as np
import pytest

from src.robot import firmware_gait as fg

DT = 0.005  # the firmware control-task period
ENTRY_TICKS = 200   # 1 s: the ramp's rise and the first cycle at least
SETTLE_TICKS = 600  # 3 s: the ramp is within 0.1 mm of the command
STEADY_TICKS = 400  # 2 s: at least one full cycle at every classical cadence
ENTRY_STEP_BOUND = 1.5
CLASSICAL_GAITS = (fg.TRI_GATE, fg.BI_GATE, fg.WAVE, fg.RIPPLE)


def _stand_command(gait_type):
    """MotionService's STAND command."""
    gait = fg.GaitState(gait_type=gait_type)
    fg.set_gait(gait)
    return gait


def _full_forward(stand):
    """MotionService's classical WALK command for full forward stick at s = s1 = 0."""
    walk = copy.deepcopy(stand)
    fg.command_to_walk_gait(lx=0.0, ly=1.0, rx=0.0, s=0.0, s1=0.0, gait=walk)
    return walk


def _swap_gait(live, target, gait_type):
    """MotionService::updateAutoGait's swap: only while not stepping, onto the live and target state."""
    live.gait_type = gait_type
    fg.set_gait(live)
    target.gait_type = gait_type
    target.stand_frac = live.stand_frac
    target.offset = live.offset.copy()


class _JointTracker:
    def __init__(self, body):
        self.kin = fg.Kinematics()
        self.previous = self.kin.inverse_kinematics(body)

    def run(self, controller, live, target, body, ticks):
        """Largest per-tick joint change in degrees over `ticks` control ticks."""
        worst = 0.0
        for _ in range(ticks):
            fg.approach_gait_command(live, target, DT)
            controller.step(live, body, DT)
            angles = self.kin.inverse_kinematics(body)
            worst = max(worst, float(np.max(np.abs(angles - self.previous))))
            self.previous = angles
        return worst


def _entry_and_steady(controller, live, target, body, joints):
    entry = joints.run(controller, live, target, body, ENTRY_TICKS)
    joints.run(controller, live, target, body, SETTLE_TICKS)
    steady = joints.run(controller, live, target, body, STEADY_TICKS)
    return entry, steady


@pytest.mark.parametrize("gait_type", CLASSICAL_GAITS)
def test_walk_entered_from_stand_moves_no_joint_faster_than_walking(gait_type):
    controller, body = fg.GaitController(), fg.BodyState()
    live = _stand_command(gait_type)
    target = copy.deepcopy(live)
    joints = _JointTracker(body)

    joints.run(controller, live, target, body, SETTLE_TICKS)
    target = _full_forward(target)
    entry, steady = _entry_and_steady(controller, live, target, body, joints)

    assert entry <= ENTRY_STEP_BOUND * steady, (
        f"entry {entry:.2f} deg/tick ({entry / DT:.0f} deg/s), steady {steady:.2f} deg/tick")


@pytest.mark.parametrize("from_type,to_type",
                         list(itertools.permutations(CLASSICAL_GAITS, 2)))
def test_gait_swapped_at_a_standstill_moves_no_joint_faster_than_walking(from_type, to_type):
    controller, body = fg.GaitController(), fg.BodyState()
    live = _stand_command(from_type)
    stand = copy.deepcopy(live)
    target = _full_forward(stand)
    joints = _JointTracker(body)

    joints.run(controller, live, target, body, SETTLE_TICKS)
    target = copy.deepcopy(stand)
    joints.run(controller, live, target, body, SETTLE_TICKS)
    _swap_gait(live, target, to_type)
    target = _full_forward(target)
    entry, steady = _entry_and_steady(controller, live, target, body, joints)

    assert entry <= ENTRY_STEP_BOUND * steady, (
        f"entry {entry:.2f} deg/tick ({entry / DT:.0f} deg/s), steady {steady:.2f} deg/tick")


def _walk_until_the_command_is_released(joints, controller, live, target, body):
    """Walks on until the ramp is one tick from the deadband, so the next tick is the first at rest."""
    while True:
        nxt = copy.deepcopy(live)
        fg.approach_gait_command(nxt, target, DT)
        if not fg.gait_is_commanded(nxt):
            return
        joints.run(controller, live, target, body, 1)


@pytest.mark.parametrize("gait_type", (fg.TRI_GATE, fg.BI_GATE))
def test_feet_come_home_from_a_stopped_walk_without_a_jolt(gait_type):
    worst, lowest = 0.0, 0.0
    for stop_at in range(8):
        controller, body = fg.GaitController(), fg.BodyState()
        live = _stand_command(gait_type)
        stand = copy.deepcopy(live)
        target = _full_forward(stand)
        joints = _JointTracker(body)

        joints.run(controller, live, target, body, SETTLE_TICKS + 17 * stop_at)
        target = copy.deepcopy(stand)
        _walk_until_the_command_is_released(joints, controller, live, target, body)
        for _ in range(ENTRY_TICKS):
            worst = max(worst, joints.run(controller, live, target, body, 1))
            lowest = min(lowest, float(np.min(body.feet[:, 2] - fg.DEFAULT_FEET[:, 2])))
        np.testing.assert_allclose(body.feet[:, :3], fg.DEFAULT_FEET[:, :3], atol=1e-3)
    assert worst <= 0.4, f"largest joint step at rest {worst:.3f} deg/tick"
    assert lowest >= -0.01, f"a foot went {-lowest:.4f} mm below the floor"


def test_feet_moved_while_standing_are_eased_back():
    gait = _stand_command(fg.TRI_GATE)
    controller, body = fg.GaitController(), fg.BodyState()
    for _ in range(200):
        controller.step(gait, body, DT)

    body.feet[:, 2] += 30.0
    controller.step(gait, body, DT)
    assert np.all(body.feet[:, 2] > fg.DEFAULT_FEET[:, 2] + 29.0), "the feet were snapped home"

    for _ in range(99):
        controller.step(gait, body, DT)
    np.testing.assert_allclose(body.feet[:, 2], fg.DEFAULT_FEET[:, 2], atol=1e-9)


@pytest.mark.parametrize("gait_type", (fg.TRI_GATE, fg.BI_GATE))
@pytest.mark.parametrize("rest_ticks", (1, 40))  # the first tick at rest, and 0.2 s into the ease
def test_walk_resumed_while_the_feet_come_home_moves_no_joint_faster_than_walking(gait_type, rest_ticks):
    controller, body = fg.GaitController(), fg.BodyState()
    stand = _stand_command(gait_type)
    live = copy.deepcopy(stand)
    target = _full_forward(stand)
    joints = _JointTracker(body)

    joints.run(controller, live, target, body, SETTLE_TICKS)
    target = copy.deepcopy(stand)
    _walk_until_the_command_is_released(joints, controller, live, target, body)
    joints.run(controller, live, target, body, rest_ticks)
    target = _full_forward(stand)
    entry, steady = _entry_and_steady(controller, live, target, body, joints)

    assert entry <= ENTRY_STEP_BOUND * steady, (
        f"entry {entry:.2f} deg/tick ({entry / DT:.0f} deg/s), steady {steady:.2f} deg/tick")
