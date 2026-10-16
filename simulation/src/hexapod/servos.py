"""The servos' angle-to-PWM mapping, as ServoController (firmware/include/peripherals/servo_controller.h) applies it.

setAngles adds 90 degrees to each tibia's IK angle, and toPwm writes
    pwm = (angle + center_angle) * direction * conversion + center_pwm
in PCA9685 counts (4096 per 20 ms period at 50 Hz, 4.88 us each), clipped to 0-4095 and truncated to a whole count.
`conversion` is the counts per degree the firmware assumes for each servo (platform_shared/api.proto Servo, factory 2);
a servo that truly turns one degree per `scale` counts goes, for a PWM, to the angle `angles(pwm, scale)` gives, so it
turns every commanded angle about its centre PWM by conversion / scale.
"""
from dataclasses import dataclass

import numpy as np

NUM_SERVOS = 18
TIBIA_OFFSET_DEG = np.array([0.0, 0.0, 90.0] * 6)  # setAngles: target = angle + 90 on joint 2 of each leg
PWM_MAX = 4095


@dataclass(frozen=True)
class ServoCalibration:
    """Per servo, in servo order (leg * 3 + coxa, femur, tibia)."""

    center_pwm: tuple[float, ...]
    direction: tuple[float, ...]
    center_angle: tuple[float, ...]
    conversion: tuple[float, ...]  # PWM counts per degree the firmware assumes

    def pwm(self, angles_deg) -> np.ndarray:
        """The PWM the firmware writes for each servo's IK angle (degrees)."""
        servo_deg = np.asarray(angles_deg, dtype=float) + TIBIA_OFFSET_DEG + np.asarray(self.center_angle)
        pwm = servo_deg * np.asarray(self.direction) * np.asarray(self.conversion) + np.asarray(self.center_pwm)
        return np.floor(np.clip(pwm, 0.0, PWM_MAX))

    def angles(self, pwm, scale: float | None = None) -> np.ndarray:
        """The IK angle (degrees) each servo goes to at these PWMs, turning one degree per `scale` counts (default:
        the counts per degree the firmware assumes)."""
        scale = np.asarray(self.conversion) if scale is None else scale
        turned = (np.asarray(pwm, dtype=float) - np.asarray(self.center_pwm)) / (np.asarray(self.direction) * scale)
        return turned - np.asarray(self.center_angle) - TIBIA_OFFSET_DEG


# ServoSettings_defaults (firmware/include/settings/servo_factory.h).
FACTORY = ServoCalibration(
    center_pwm=(306.0,) * NUM_SERVOS,
    direction=(1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0),
    center_angle=(0.0,) * NUM_SERVOS,
    conversion=(2.0,) * NUM_SERVOS,
)
