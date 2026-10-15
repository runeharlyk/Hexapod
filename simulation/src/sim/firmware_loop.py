"""The firmware's control loop driving the MuJoCo robot: each 5 ms tick the firmware's own Hexapod (virtual_robot.py,
the code main.cpp's control task runs) reads the simulated IMU and drives the servos, and physics stays locked to the
tick. Modes, gaits and sticks reach it as a transport delivers them, and the current sticks are sent again every 20 ms,
as the app's heartbeat does, so a held command outlasts the firmware's dead-man timeout as it does on the robot.
"""
import numpy as np

from src.sim.mj_runtime import HexapodSim
from src.sim.virtual_robot import DEACTIVATED, STAND, WALK, VirtualRobot

FIRMWARE_DT = 0.005  # control-task period (firmware/src/main.cpp)
HEARTBEAT_TICKS = 4  # the app's 20 ms stick period


def imu_reading(quat: np.ndarray, gyro: np.ndarray):
    """The MPU6050 as the training environment derives it from the base: roll, pitch and yaw of the base quaternion in
    radians, world up in the body frame, and the gyro sensor."""
    w, x, y, z = quat
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    up = (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
    return (roll, pitch, yaw), up, gyro


class FirmwareLoop:
    def __init__(self, sim: HexapodSim, gait: int):
        self.sim = sim
        self.robot = VirtualRobot()
        self.robot.set_gait(gait)
        self.ticks = 0
        self.physics_steps = 0
        self._mode = DEACTIVATED
        self._sticks = None
        self._writes = 0

    def set_mode(self, mode: int) -> None:
        if mode != self._mode:
            self._mode = mode
            self.robot.set_mode(mode)

    def walk(self, lx: float, ly: float, rx: float, s: float = 0.0, s1: float = 0.0, h: float = 0.0) -> None:
        """WALK with these sticks."""
        self.set_mode(WALK)
        self._send(dict(lx=lx, ly=ly, rx=rx, s=s, s1=s1, h=h))

    def hold(self, h: float = 0.0) -> None:
        """STAND with centred sticks: no stride, only the height."""
        self.set_mode(STAND)
        self._send(dict(h=h))

    def select_gait(self, gait: int) -> None:
        self.robot.set_gait(gait)

    def tick(self) -> None:
        if self._sticks is not None and self.ticks % HEARTBEAT_TICKS == 0:
            self.robot.command(**self._sticks)
        self.robot.set_imu(*imu_reading(self.sim.base_quat(), self.sim.gyro()))
        self.robot.tick()
        self.sim.set_powered(self.robot.servos_driven())
        # Only a tick that wrote a pose moves the targets: in IDLE a raw PWM write holds, as on the robot.
        if self.robot.servo_writes() != self._writes:
            self._writes = self.robot.servo_writes()
            self.sim.set_joint_targets(np.radians(self.robot.servo_angles()))
        self.ticks += 1
        # 5 ms is not a whole number of physics steps, so keep physics time locked to tick time.
        due = int(round(self.ticks * FIRMWARE_DT / self.sim.model.opt.timestep))
        self.sim.step_physics(due - self.physics_steps)
        self.physics_steps = due

    def _send(self, sticks: dict) -> None:
        self._sticks = sticks
        self.robot.command(**sticks)
