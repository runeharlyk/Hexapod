"""Measures the servos' PWM counts per degree with the IMU alone: uv run python measure_pwm_per_degree.py 192.168.4.1

On the robot this needs the real backend's roll, servo_calibration and set_servo_pwm, which are being added to
src/hexapod/backends/real.py; until then it refuses, naming what is missing. `simulation` runs it today.

The firmware turns each joint's IK angle into PCA9685 counts through its servo calibration, assuming `conversion`
counts per degree (factory 2; src/hexapod/servos.py has the mapping, firmware/include/peripherals/servo_controller.h
the original). A servo that sweeps 180 degrees over 500-2500 us takes 2.28 counts per degree at the PCA9685's 50 Hz
(4.88 us per count), and every commanded angle would then come out 12 % short of what the firmware meant. This script
finds the true figure, taken to be the same for every servo.

Procedure. The operator stands the robot (STAND) on a flat, level floor first, and the script refuses a robot that is
not standing still. It sets the nominal height, switches to IDLE, which leaves the servos powered and the motion silent,
and writes every servo the PWM the firmware gives the stand. Then it steps the six femurs together through raw PWMs, the
left femurs' IK angles up and the right femurs' down (or the reverse), one count every RAMP_S_PER_COUNT, in steps of
+-STEPS counts with a return to the stand's PWM between them, each held DWELL_S while the IMU's roll is averaged. The
femurs lift or lower the feet, so one side's feet rise as the other's sink and the body rolls on the floor; the coxae,
which turn about the vertical, would only swing the feet fore and aft. The roll each step adds, against the roll the
firmware's leg geometry predicts for it at a given counts per degree, fits the counts per degree. Every joint's angle,
the stand's too, is taken from the PWM written to it at the fitted counts per degree, so the stand's own error at a
wrong scale is in the prediction. At the end the femurs ramp back to the stand's PWM and STAND applies afresh, which
hands the servos back to the motion. Nothing here walks; the largest step turns the femurs about 6 degrees and rolls the
body about 3. Keep a hand near it all the same. It exits 0 with a fit, 1 when the fit does not explain the measurement,
and 2 when it refuses or loses the robot.

The roll is the IMU's, left side up positive. On the robot that is the DMP's angle about the body's forward axis, whose
sign against the body frame was measured on the bench (firmware/include/imu_frame.h, docs/conventions.md, IMU angles);
a roll of the opposite sign to every prediction is reported as such.
The fit takes the firmware's leg geometry for the robot's, so a femur or tibia off by x % scales the predicted roll,
and the fit, by about x %; friction under the feet, which slide sideways as the femurs turn, a loose horn and the
servos' deadband add to it, and a residual well above the IMU's noise says the body did not roll as the geometry
expects (a foot off the floor, or slipping). No floor-slope check is made: a sloping floor the stand has levelled the
body on passes.

`simulation` as the address runs it on the MuJoCo model, where --sim-scale sets the servos' true counts per degree.
"""
import argparse
import contextlib
import sys
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from src.hexapod import Mode, Robot, RobotError
from src.hexapod.servos import ServoCalibration
from src.robot.firmware_gait import BodyState, Kinematics

SAMPLE_S = 0.1
STAND_SETTLE_S = 1.0
STILL_S = 2.0
DWELL_S = 2.0
SETTLE_S = 0.5
RAMP_S_PER_COUNT = 0.04  # 25 counts a second: about 12 degrees a second at 2 counts per degree
STEPS = (4, 8, 12)  # counts, each taken up and down
MAX_STILL_SPREAD_DEG = 0.5  # a robot standing still wavers less than this in roll (standard deviation)
MAX_ROLL_DEG = 8.0  # a step that rolls the body further than this stops the measurement
MAX_RESIDUAL_DEG = 0.25  # a fit that leaves more than this has not explained the roll
FEMURS = [3 * leg + 1 for leg in range(6)]
SIDE = np.array([-1.0, -1.0, -1.0, 1.0, 1.0, 1.0])  # legs 0-2 are the right side, 3-5 the left (docs/conventions.md)
KINEMATICS = Kinematics()


class Refused(Exception):
    pass


SERVO_ACCESS = ("servo_calibration", "set_servo_pwm")


def require_servo_access(backend) -> None:
    """Refuses a backend that cannot read the servo calibration or write a raw PWM."""
    missing = [name for name in SERVO_ACCESS if not hasattr(backend, name)]
    if missing:
        raise Refused(f"the {type(backend).__name__} has no {' or '.join(missing)}, which this measurement needs")


def floor_roll(angles_deg) -> float:
    """The roll (degrees, left side up) of a body standing on a level floor on its six feet at these IK angles: the
    least-squares plane through the feet, placed by the firmware's leg geometry."""
    q = np.radians(np.asarray(angles_deg, dtype=float).reshape(6, 3))
    feet = np.array([KINEMATICS.forward_kinematics_local(leg, *q[leg]) for leg in range(6)])
    a = np.column_stack([np.ones(6), feet[:, 0], feet[:, 1]])
    _, slope_x, slope_y = np.linalg.lstsq(a, feet[:, 2], rcond=None)[0]
    # Feet higher on the right (+x) put the floor nearer the body's right side: the left side is up.
    return float(np.degrees(np.arcsin(slope_x / np.sqrt(1.0 + slope_x**2 + slope_y**2))))


def step_signs(calibration: ServoCalibration) -> np.ndarray:
    """The femurs' PWM signs that raise the left femurs' IK angles and lower the right ones'."""
    return np.asarray(calibration.direction)[FEMURS] * SIDE


def stepped(calibration: ServoCalibration, stand_pwm: np.ndarray, offset: float) -> np.ndarray:
    pwm = np.asarray(stand_pwm, dtype=float).copy()
    pwm[FEMURS] += step_signs(calibration) * offset
    return pwm


def predicted_roll(calibration: ServoCalibration, stand_pwm, offset: float, scale: float) -> float:
    """The roll a step of `offset` counts on the femurs adds to a body standing at these PWMs, for servos that turn one
    degree per `scale` counts."""
    return (floor_roll(calibration.angles(stepped(calibration, stand_pwm, offset), scale))
            - floor_roll(calibration.angles(stand_pwm, scale)))


@dataclass(frozen=True)
class ScaleFit:
    scale: float  # PWM counts per degree
    sigma: float  # its standard error, from the residuals
    residual_rms: float  # degrees of roll the fit leaves


def fit_scale(calibration: ServoCalibration, stand_pwm, offsets, rolls, start: float = 2.0) -> ScaleFit:
    """The counts per degree that best predict the measured roll each offset added."""
    offsets, rolls = np.asarray(offsets, dtype=float), np.asarray(rolls, dtype=float)

    def residuals(params):
        return np.array([predicted_roll(calibration, stand_pwm, offset, params[0]) for offset in offsets]) - rolls

    result = least_squares(residuals, [start], bounds=([0.5], [8.0]))
    variance = float(result.fun @ result.fun) / max(1, result.fun.size - 1)
    jacobian = result.jac[:, 0]
    sigma = float(np.sqrt(variance / max(float(jacobian @ jacobian), 1e-12)))
    return ScaleFit(float(result.x[0]), sigma, float(np.sqrt(np.mean(result.fun**2))))


@dataclass(frozen=True)
class Measurement:
    offsets: np.ndarray  # counts on each femur, signed by step_signs
    rolls: np.ndarray  # the roll each step added (degrees)
    predicted: np.ndarray  # the roll the fit predicts for each
    fit: ScaleFit


def roll_samples(robot: Robot, seconds: float) -> np.ndarray:
    samples = []
    for _ in range(max(1, round(seconds / SAMPLE_S))):
        robot.sleep(SAMPLE_S)
        samples.append(robot.state().roll)
    return np.asarray(samples)


class FemurSteps:
    """Writes the femurs raw PWMs about the stand's, one count at a time."""

    def __init__(self, robot: Robot, calibration: ServoCalibration, stand_pwm: np.ndarray):
        self.robot, self.calibration, self.stand_pwm = robot, calibration, stand_pwm
        self.offset = 0

    def write_stand(self) -> None:
        for servo, pwm in enumerate(self.stand_pwm):
            self.robot.set_servo_pwm(servo, int(pwm))
        self.offset = 0

    def ramp(self, offset: int) -> None:
        step = 1 if offset > self.offset else -1
        while self.offset != offset:
            self.offset += step
            pwm = stepped(self.calibration, self.stand_pwm, self.offset)
            for servo in FEMURS:
                self.robot.set_servo_pwm(servo, int(pwm[servo]))
            self.robot.sleep(RAMP_S_PER_COUNT)

    def held(self, offset: int) -> float:
        self.ramp(offset)
        self.robot.sleep(SETTLE_S)
        return float(roll_samples(self.robot, DWELL_S).mean())


def measure(robot: Robot, steps=STEPS) -> Measurement:
    """Steps the femurs and fits the counts per degree; leaves the robot standing under the motion's control again."""
    state = robot.state()
    if state.mode is not Mode.STAND:
        name = state.mode.name if state.mode is not None else "in an unknown mode"
        raise Refused(f"the robot is {name}: stand it on a flat, level floor first")
    if state.roll is None:
        raise Refused("the robot reports no roll (no IMU, or a backend that does not pass it on), and the roll is "
                      "what this measures")
    robot.stand()
    robot.sleep(STAND_SETTLE_S)
    calibration = robot.servo_calibration()
    stand_pwm = calibration.pwm(KINEMATICS.inverse_kinematics(BodyState()))
    offsets = np.array([sign * n for n in steps for sign in (1, -1)])
    robot.idle()
    femurs = FemurSteps(robot, calibration, stand_pwm)
    rolls = []
    try:
        femurs.write_stand()
        # Stillness is checked where the measurement runs: in IDLE nothing moves the servos, while in STAND the
        # firmware's self-levelling keeps answering the IMU.
        robot.sleep(STAND_SETTLE_S)
        spread = roll_samples(robot, STILL_S).std()
        if spread > MAX_STILL_SPREAD_DEG:
            raise Refused(f"the robot is not still: its roll wavers by {spread:.2f} degrees")
        before = femurs.held(0)
        for offset in offsets:
            during = femurs.held(int(offset))
            after = femurs.held(0)
            rolls.append(during - (before + after) / 2)
            print(f"{offset:+4d} counts: roll {rolls[-1]:+.2f} degrees")
            if abs(rolls[-1]) > MAX_ROLL_DEG:
                raise RobotError(f"the body rolled {rolls[-1]:.1f} degrees for {offset} counts, over {MAX_ROLL_DEG}")
            before = after
    finally:
        with contextlib.suppress(RobotError):  # without a link the robot holds its servos where they are
            femurs.ramp(0)
            robot.stand()  # applies afresh, which hands the servos back to the motion
    fit = fit_scale(calibration, stand_pwm, offsets, rolls)
    predicted = np.array([predicted_roll(calibration, stand_pwm, offset, fit.scale) for offset in offsets])
    return Measurement(offsets, np.asarray(rolls), predicted, fit)


def report(result: Measurement, calibration: ServoCalibration) -> int:
    fit = result.fit
    assumed = float(np.mean(np.asarray(calibration.conversion)[FEMURS]))
    print(f"{fit.scale:.3f} +- {fit.sigma:.3f} PWM counts per degree (the firmware assumes {assumed:g}); "
          f"the fit leaves {fit.residual_rms:.2f} degrees RMS")
    print(f"every commanded angle turns {assumed / fit.scale:.3f} times what the firmware means")
    if fit.residual_rms <= MAX_RESIDUAL_DEG:
        return 0
    print(f"the residual is over {MAX_RESIDUAL_DEG} degrees: the body did not roll as the leg geometry expects")
    if np.all(np.sign(result.rolls) == -np.sign(result.predicted)):
        print("every step rolled the body the other way from the prediction: the IMU's roll sign is the suspect")
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("address", help="the robot's address, or `simulation`")
    parser.add_argument("--sim-scale", type=float, default=2.28,
                        help="the simulated servos' true counts per degree (simulation only)")
    args = parser.parse_args()
    if args.address == "simulation":
        from src.hexapod.backends.sim import SimBackend

        backend = SimBackend(servo_scale=args.sim_scale)
    else:
        from src.hexapod.backends.real import RealBackend

        backend = RealBackend(args.address)
    # The robot is left as the operator stood it: a refusal has moved nothing, and a measurement ends in STAND.
    try:
        require_servo_access(backend)
        robot = Robot(backend=backend)
        robot.connect()
        try:
            if args.address == "simulation":  # nobody stands a simulated robot by hand
                robot.stand()
                robot.sleep(STAND_SETTLE_S)
            result = measure(robot)
            return report(result, robot.servo_calibration())
        finally:
            robot.close()
    except Refused as refusal:
        print(f"refused: {refusal}")
        return 2
    except RobotError as error:
        print(f"stopped: {error}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
