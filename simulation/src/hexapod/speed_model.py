"""The firmware gait's REP-103 body velocity per stick, as measured in the simulation by measure_speed_model.py.

The firmware takes sticks, not velocities (motion.h handleCommand): lx strafes, ly walks forward, rx turns, each in
[-1, 1]. Every axis was measured with the other two centred, so the model is exact (to the simulation) for a
single-axis command, which is what move_forward, move_sideways and rotate send. A combined command is inverted axis
by axis and is only approximate: the classical gait derives one cadence from the stride and the turn together.

Measured, not assumed: on every modelled gait +lx walks RIGHT (negative vy) and +rx turns CLOCKWISE (negative yaw
rate), so a stick pushed right on the app's joystick strafes and turns the robot to its right. The table carries the
signs; nothing here hard-codes them.
"""
import json
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

from .backends.base import ZERO, Gait, RobotError, Velocity

MODEL_PATH = Path(__file__).resolve().parents[1] / "resources" / "speed_model.json"
AXES = {"vx": "ly", "vy": "lx", "yaw_rate": "rx"}  # velocity component -> the stick that drives it


class UnmodelledGait(RobotError):
    pass


@dataclass(frozen=True)
class Sticks:
    lx: float
    ly: float
    rx: float


CENTRED = Sticks(0.0, 0.0, 0.0)
# The stride slider's setting the model was measured at: 0, each classic gait's speed-optimal stroke.
STRIDE = 0.0


class _Axis:
    """One stick against the velocity it produced. Near full deflection a larger stick can walk slower (measured on
    several gaits), so a velocity is inverted to the smallest stick that reaches it, and the peak is the top speed."""

    def __init__(self, sticks: np.ndarray, values: np.ndarray):
        self.sticks, self.values = sticks, values
        self._sides = {}  # direction of travel -> (stick sign, |stick| outward from 0, speed along that direction)
        for direction in (1.0, -1.0):
            sides = []
            for side in (1.0, -1.0):
                out = np.flatnonzero(sticks * side >= 0)
                out = out[np.argsort(np.abs(sticks[out]))]
                sides.append((side, np.abs(sticks[out]), values[out] * direction))
            self._sides[direction] = max(sides, key=lambda candidate: candidate[2].max())

    def value(self, stick: float) -> float:
        return float(np.interp(stick, self.sticks, self.values))

    def top(self) -> float:
        """The fastest speed reachable in both directions."""
        return min(float(speed.max()) for _, _, speed in self._sides.values())

    def stick(self, value: float) -> float:
        if value == 0:
            return 0.0
        side, magnitude, speed = self._sides[float(np.sign(value))]
        wanted = abs(value)
        if wanted >= speed.max():
            return side * float(magnitude[np.argmax(speed)])
        i = int(np.argmax(speed[1:] >= wanted))  # the first segment that reaches it
        fraction = (wanted - speed[i]) / (speed[i + 1] - speed[i])
        return side * float(magnitude[i] + fraction * (magnitude[i + 1] - magnitude[i]))


@lru_cache(maxsize=None)
def _axes(gait: Gait | None) -> dict[str, _Axis]:
    if gait is None:
        raise UnmodelledGait("the robot has not reported its gait")
    model = json.loads(MODEL_PATH.read_text())
    if gait.name not in model["gaits"]:
        raise UnmodelledGait(f"no measured velocity model for {gait.name}; measure_speed_model.py covers "
                             f"{', '.join(model['gaits'])}")
    sticks = np.array(model["sticks"])
    return {axis: _Axis(sticks, np.array(values)) for axis, values in model["gaits"][gait.name].items()}


def velocity_of(gait: Gait | None, sticks: Sticks) -> Velocity:
    if sticks == CENTRED:
        return ZERO
    axes = _axes(gait)
    return Velocity(axes["vx"].value(sticks.ly), axes["vy"].value(sticks.lx), axes["yaw_rate"].value(sticks.rx))


def sticks_for(gait: Gait | None, velocity: Velocity) -> Sticks:
    """The sticks for `velocity`, each component clamped to the gait's top speed in that direction. Centred sticks
    need no model, so any gait can be stopped."""
    if velocity == ZERO:
        return CENTRED
    axes = _axes(gait)
    return Sticks(axes["vy"].stick(velocity.vy), axes["vx"].stick(velocity.vx), axes["yaw_rate"].stick(velocity.yaw_rate))


def max_velocity(gait: Gait | None) -> Velocity:
    axes = _axes(gait)
    return Velocity(axes["vx"].top(), axes["vy"].top(), axes["yaw_rate"].top())
