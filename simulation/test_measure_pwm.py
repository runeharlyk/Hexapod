"""measure_pwm_per_degree.py on the MuJoCo model with a known error in the servos' counts per degree, and the servo
model it fits through against the firmware's."""
import dataclasses
import re
from pathlib import Path

import numpy as np
import pytest

import measure_pwm_per_degree as measurement
from src.hexapod import Mode, Robot
from src.hexapod.backends.sim import SimBackend
from src.hexapod.servos import FACTORY

SERVO_FACTORY_H = Path(__file__).resolve().parents[1] / "firmware" / "include" / "settings" / "servo_factory.h"


class ReportedRoll:
    """A simulated backend whose state reports the roll `roll(n)` gives on its n-th call instead of the model's."""

    def __init__(self, backend: SimBackend, roll):
        self._backend, self._roll, self._calls = backend, roll, 0

    def __getattr__(self, name):
        return getattr(self._backend, name)

    def state(self):
        self._calls += 1
        return dataclasses.replace(self._backend.state(), roll=self._roll(self._calls))


def standing(servo_scale: float | None) -> Robot:
    robot = Robot(backend=SimBackend(servo_scale=servo_scale))
    robot.connect()
    robot.stand()
    robot.sleep(1.0)
    return robot


def test_the_factory_calibration_is_the_firmwares():
    header = SERVO_FACTORY_H.read_text()
    assert int(re.search(r"FACTORY_SERVO_COUNT = (\d+);", header)[1]) == 18
    directions = re.search(r"FACTORY_DIRECTION\[FACTORY_SERVO_COUNT\] = \{([^}]*)\}", header)[1]
    assert FACTORY.direction == tuple(float(v) for v in directions.split(","))
    assert FACTORY.center_pwm == (float(re.search(r"FACTORY_CENTER_PWM = ([\d.]+);", header)[1]),) * 18
    assert FACTORY.conversion == (float(re.search(r"FACTORY_CONVERSION = ([\d.]+);", header)[1]),) * 18
    assert FACTORY.center_angle == (float(re.search(r"FACTORY_CENTER_ANGLE = ([\d.-]+);", header)[1]),) * 18


def test_pwm_is_servo_controllers_to_pwm_and_angles_inverts_it():
    angles = np.zeros(18)
    angles[0], angles[1], angles[2] = 10.0, -20.0, -100.25  # leg 0, direction -1, -1, +1
    angles[11] = -100.25  # leg 3's tibia, direction -1
    pwm = FACTORY.pwm(angles)
    # (angle + 90 on a tibia + centre angle) * direction * conversion + centre PWM, truncated to a count.
    assert pwm[:3].tolist() == [286.0, 346.0, 285.0]
    assert pwm[11] == 326.0
    assert pwm[3] == 306.0
    assert FACTORY.angles(pwm) == pytest.approx(angles, abs=0.5)
    # A servo of 2.5 counts per degree turns the same counts 2 / 2.5 as far about its centre PWM.
    assert FACTORY.angles(pwm, 2.5)[1] == pytest.approx(-20.0 * 2 / 2.5)
    assert FACTORY.angles(pwm, 2.5)[2] + 90 == pytest.approx((285 - 306) / 2.5)


@pytest.mark.parametrize("true_scale", [2.28, 1.8])
def test_the_measurement_recovers_the_servos_true_scale(true_scale):
    robot = standing(true_scale)
    result = measurement.measure(robot)
    assert result.fit.scale == pytest.approx(true_scale, rel=0.02)
    assert result.fit.residual_rms < measurement.MAX_RESIDUAL_DEG
    # Raising the left femurs' IK angle and lowering the right ones' rolls the body one way, by more for more.
    assert len(set(np.sign(result.rolls * result.offsets))) == 1
    assert np.all(np.diff(np.abs(result.rolls[::2])) > 0)


def test_servos_that_match_the_firmware_measure_its_conversion():
    result = measurement.measure(standing(None))
    assert result.fit.scale == pytest.approx(FACTORY.conversion[0], rel=0.02)


def test_the_measurement_hands_the_robot_back_standing_under_the_motion():
    robot = standing(2.28)
    backend = robot._backend
    before = backend.sim.joint_target.copy()
    measurement.measure(robot)
    robot.sleep(1.0)
    state = robot.state()
    assert state.mode is Mode.STAND
    assert abs(state.roll) < 0.5
    assert np.degrees(np.abs(backend.sim.joint_target - before)).max() < 0.5


def test_the_measurement_refuses_a_robot_that_does_not_stand():
    robot = Robot(backend=SimBackend())
    robot.connect()
    with pytest.raises(measurement.Refused, match="DEACTIVATED"):
        measurement.measure(robot)


def test_the_measurement_refuses_a_robot_that_reports_no_roll():
    backend = SimBackend()
    robot = Robot(backend=ReportedRoll(backend, lambda n: None))
    robot.connect()
    robot.stand()
    with pytest.raises(measurement.Refused, match="no roll"):
        measurement.measure(robot)
    assert backend.state().mode is Mode.STAND  # refused before it touched a servo


def test_the_measurement_refuses_a_robot_that_does_not_stand_still():
    backend = SimBackend()
    robot = Robot(backend=ReportedRoll(backend, lambda n: (-1.0) ** n))  # a degree either way, sample to sample
    robot.connect()
    robot.stand()
    with pytest.raises(measurement.Refused, match="not still"):
        measurement.measure(robot)
    assert backend.state().mode is Mode.STAND


def test_a_backend_without_raw_servo_access_is_refused_by_name():
    class NoServos:
        def servo_calibration(self):
            raise AssertionError("not called")

    with pytest.raises(measurement.Refused, match="NoServos has no set_servo_pwm,"):
        measurement.require_servo_access(NoServos())
    measurement.require_servo_access(SimBackend())
