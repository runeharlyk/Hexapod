"""The MuJoCo hexapod behind the backend interface: the firmware's STAND/WALK loop, stepped only inside sleep().

It takes the same sticks the real robot does, from the same speed model, so a script behaves alike on both. It does
not model the link: no command is ever lost, so the dead-man stop never fires and link_drops stays 0.
"""
import math
import time
from dataclasses import replace

import numpy as np

from ...sim.firmware_loop import FIRMWARE_DT, FirmwareLoop
from ...sim.mj_runtime import HexapodSim
from .. import speed_model
from ..servos import FACTORY, NUM_SERVOS, ServoCalibration
from .base import Gait, Goal, GoalStatus, Mode, Odometry, RobotDisconnected, RobotError, RobotState, Style, Velocity

POWERED = (Mode.STAND, Mode.WALK, Mode.IDLE)
# Why the firmware refuses a goal (main.cpp goalRefusal), for the transport the simulation has none of.
REFUSALS = {400: "a malformed goal or style", 409: "only a standing or walking robot takes a goal",
            422: "no goal walks in this style: TUNED, or no speed at its posture"}


class ServoOutput:
    """The HexapodSim as FirmwareLoop drives it, through ServoController: each joint target becomes the PWM the firmware
    writes for it, which a raw PWM replaces until the motion writes again. With a true `scale` (counts per degree)
    every joint goes where its PWM turns such a servo; without, a motion target is taken as is, as before the PWM."""

    def __init__(self, sim: HexapodSim, calibration: ServoCalibration, scale: float | None):
        self.sim, self.calibration, self.scale = sim, calibration, scale
        self.pwm = calibration.pwm(np.degrees(sim.joint_target))

    def __getattr__(self, name):
        return getattr(self.sim, name)

    def set_joint_targets(self, angles_rad) -> None:
        self.pwm = self.calibration.pwm(np.degrees(angles_rad))
        self.sim.set_joint_targets(angles_rad if self.scale is None else self._joints())

    def write_pwm(self, servo: int, pwm: int) -> None:
        self.pwm = self.pwm.copy()
        self.pwm[servo] = pwm
        self.sim.set_joint_targets(self._joints())

    def _joints(self) -> np.ndarray:
        return np.radians(self.calibration.angles(self.pwm, self.scale))


class SimBackend:
    """`viewer` opens the MuJoCo viewer and runs sleep() in real time; without it the simulation runs flat out.
    `servo_scale` is the servos' true PWM counts per degree, where the firmware assumes the factory calibration's."""

    def __init__(self, viewer: bool = False, servo_scale: float | None = None):
        self.sim = HexapodSim()
        self._servo_scale = servo_scale
        self._servos: ServoOutput | None = None
        self._show = viewer
        self._viewer = None
        self._loop: FirmwareLoop | None = None
        self._mode = Mode.DEACTIVATED  # as the firmware boots
        self._gait = Gait.TRI_GATE
        self._style = Style()  # the posture and stride every command carries: set_height's height, then each goal's
        self._sticks = speed_model.CENTRED

    def connect(self) -> None:
        self.sim.reset_to_stand()
        self._servos = ServoOutput(self.sim, FACTORY, self._servo_scale)
        self._loop = FirmwareLoop(self._servos, self._gait)
        self.set_mode(self._mode)
        if self._show:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()

    # The firmware applies the mode. STAND holds the body still: the firmware would shift and tilt it with the sticks,
    # but the facade only sends sticks in WALK. IDLE leaves the servos powered where the last write put them.
    def set_mode(self, mode: Mode) -> None:
        if mode not in (*POWERED, Mode.DEACTIVATED):
            raise RobotError(f"the simulation runs STAND, WALK, IDLE and DEACTIVATED, not {mode.name}")
        self._mode = mode
        self._loop.set_mode(int(mode))
        self._command()

    def set_gait(self, gait: Gait) -> None:
        self._gait = gait
        self._loop.select_gait(int(gait))

    def set_height(self, height: float) -> None:
        self._style = replace(self._style, h=height)
        self._command()

    def set_velocity(self, velocity: Velocity) -> Velocity:
        self._sticks = speed_model.sticks_for(self._gait, velocity)
        self._command()
        return speed_model.velocity_of(self._gait, self._sticks)

    def _command(self) -> None:
        style = self._style
        if self._mode is Mode.WALK:
            s = self._sticks
            self._loop.walk(lx=s.lx, ly=s.ly, rx=s.rx, h=style.h, s1=style.s1, fd=style.fd, stride=style.stride)
        elif self._mode is Mode.STAND:
            self._loop.hold(h=style.h, s1=style.s1, fd=style.fd, stride=style.stride)

    def max_velocity(self) -> Velocity:
        return speed_model.max_velocity(self._gait)

    def state(self) -> RobotState:
        w, x, y, z = self.sim.base_quat()
        # The rotation about the body's forward axis, +Y: the world z of the body's +X (right) axis is -sin(roll).
        roll = math.degrees(math.atan2(-2.0 * (x * z - w * y), 1.0 - 2.0 * (x * x + y * y)))
        return RobotState(t=self.now(), mode=self._mode, gait=self._gait, yaw=math.degrees(self.sim.base_yaw()),
                          link_drops=0, roll=roll)

    def fresh_mode(self) -> Mode | None:
        return self._mode

    def now(self) -> float:
        return self._loop.ticks * FIRMWARE_DT

    def sleep(self, seconds: float) -> None:
        start = time.monotonic()
        for tick in range(1, round(seconds / FIRMWARE_DT) + 1):
            self._loop.tick()
            if self._viewer is not None:
                if not self._viewer.is_running():
                    raise RobotDisconnected("the simulation viewer was closed")
                self._viewer.sync()
                time.sleep(max(0.0, start + tick * FIRMWARE_DT - time.monotonic()))

    def servo_calibration(self) -> ServoCalibration:
        return FACTORY

    def supports_goals(self) -> bool:
        return True

    def goal_move(self, goal: Goal, style: Style) -> GoalStatus:
        self._sticks = speed_model.CENTRED
        self._command()
        kind, distance, angle, radius = goal.in_body_frame()
        status = self._loop.goal_move(int(kind), distance, angle, radius, style.speed, int(style.gait), style.stride,
                                      style.s1, style.h, style.fd)
        if status != 202:
            raise RobotError(f"the simulation refused the goal with status {status}: {REFUSALS.get(status, '')}")
        self._mode, self._gait, self._style = Mode.WALK, style.gait, style
        self._command()
        return self.goal_status()

    def goal_cancel(self) -> None:
        self._loop.goal_cancel()

    def goal_status(self) -> GoalStatus:
        return GoalStatus.from_body(*self._loop.goal_status())

    def odometry(self) -> Odometry:
        return Odometry.from_body(*self._loop.odometry())

    def reset_odometry(self) -> None:
        """The firmware zeroes the pose on its next tick, so one is run. It refuses while a goal drives, as with 409."""
        if not self._loop.reset_odometry():
            raise RobotError("the simulation refused the reset with status 409: No odometry reset while a goal runs")
        self.sleep(FIRMWARE_DT)

    def set_servo_pwm(self, servo: int, pwm: int) -> None:
        """ServoController::servoEvent: holds until the motion writes again, so in IDLE until the mode changes."""
        if not 0 <= servo < NUM_SERVOS:
            raise RobotError(f"no servo {servo} (0-{NUM_SERVOS - 1})")
        self._servos.write_pwm(servo, pwm)
