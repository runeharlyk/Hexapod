"""Replays the firmware's gait trace through firmware_gait.py, every recorded foot coordinate.

The fixture `fixtures/gait_trace.json` is written by test_gait_trace_ends_at_rest_on_the_stance in
firmware/test/test_gait/test_gait.cpp: stand, walk full forward, a stop cut short so the next walk
starts while the feet are still easing home, walk full left, stop, for the tripod and the bipod,
through the command ramp and GaitController::step at the 5 ms tick. Every tick is run; every
`record_every`-th is recorded, as the feet's offsets from the stance.
Regenerate it from the repository root with
    GAIT_TRACE_OUT=simulation/fixtures/gait_trace.json pio test -e native -f test_gait
The constant-level checks in test_firmware_gait_parity.py cannot see a changed formula; this one
fails on any law the port does not reproduce.
"""

import copy
import json
from pathlib import Path

import numpy as np
import pytest

from src.robot import firmware_gait as fg

FIXTURE = Path(__file__).parent / "fixtures" / "gait_trace.json"
# The firmware runs in float32 and the port in float64; over this fixture's 695 ticks the two drift
# apart by at most 4.5e-4 mm (measured), plus 5e-5 mm of rounding in the fixture. The bound assumes
# that accumulation: a longer session can drift further. A wrong law shows at once: a 0.45 s ease in
# place of 0.5 s is 2.2 mm off, and dropping the carry into a resumed walk 9.2 mm.
TOLERANCE_MM = 1e-3


@pytest.fixture(scope="module")
def trace():
    with open(FIXTURE) as f:
        return json.load(f)


def _replay(trace, gait_type):
    controller, body = fg.GaitController(), fg.BodyState()
    live = fg.GaitState(gait_type=gait_type)
    fg.set_gait(live)
    target = copy.deepcopy(live)
    recorded = []
    tick = 0
    for segment in trace["segments"]:
        target.step_x, target.step_y = segment["step_x"], segment["step_y"]
        target.step_height = segment["step_height"]
        for _ in range(segment["ticks"]):
            fg.approach_gait_command(live, target, trace["dt"], trace["command_tau_s"])
            controller.step(live, body, trace["dt"])
            tick += 1
            if tick % trace["record_every"] == 0:
                recorded.append((body.feet[:, :3] - fg.DEFAULT_FEET[:, :3]).ravel())
    return np.array(recorded)


@pytest.mark.parametrize("gait_name", ["TRI_GATE", "BI_GATE"])
def test_port_reproduces_the_firmware_gait_trace(trace, gait_name):
    expected = np.array(trace["feet_from_stance_mm"][gait_name])
    actual = _replay(trace, getattr(fg, gait_name))
    assert actual.shape == expected.shape
    deviation = np.abs(actual - expected)
    row, coordinate = np.unravel_index(np.argmax(deviation), deviation.shape)
    assert deviation.max() <= TOLERANCE_MM, (
        f"{deviation.max():.2e} mm off at tick {(row + 1) * trace['record_every']}, "
        f"foot {coordinate // 3} axis {'xyz'[coordinate % 3]}")
