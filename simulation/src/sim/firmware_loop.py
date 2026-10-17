"""The firmware's control loop driving the MuJoCo robot: each 5 ms tick the firmware's own Hexapod (virtual_robot.py,
the code main.cpp's control task runs) reads the simulated IMU and drives the servos, and physics stays locked to the
tick. Modes, gaits, sticks and goals reach it as a transport delivers them, and the current sticks are sent again every
20 ms, as the app's heartbeat does, so a held command outlasts the firmware's dead-man timeout as it does on the robot.
"""
import numpy as np

from src.sim.firmware_policy import dmp_angles
from src.sim.mj_runtime import HexapodSim
from src.sim.virtual_robot import DEACTIVATED, STAND, WALK, VirtualRobot

FIRMWARE_DT = 0.005  # control-task period (firmware/src/main.cpp)
HEARTBEAT_TICKS = 4  # the app's 20 ms stick period


def imu_reading(quat: np.ndarray, gyro: np.ndarray):
    """The MPU6050 as the robot reports it: the base quaternion's roll, pitch and yaw in radians, with the DMP's signs,
    world up in the body frame, and the gyro sensor's rate, low-passed over the last tick (HexapodSim.mean_gyro)."""
    w, x, y, z = quat
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    up = (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
    return dmp_angles((roll, pitch, yaw)), up, gyro


class FirmwareLoop:
    def __init__(self, sim: HexapodSim, gait: int):
        self.sim = sim
        self.robot = VirtualRobot()
        self.robot.set_gait(gait)
        self.ticks = 0
        self.physics_steps = 0
        self._start = sim.data.time
        self._mode = DEACTIVATED
        self._mode_undelivered = False  # a mode request reaches the motion on the next tick
        self._sticks = None
        self._writes = 0

    def set_mode(self, mode: int) -> None:
        if mode != self._mode:
            self._mode = mode
            self.robot.set_mode(mode)
            self._mode_undelivered = True

    def walk(self, lx: float, ly: float, rx: float, s: float = 0.0, s1: float = 0.0, h: float = 0.0,
             stride: float = 0.0, fd: float = 0.0) -> None:
        """WALK with these sticks."""
        self.set_mode(WALK)
        self._send(dict(lx=lx, ly=ly, rx=rx, s=s, s1=s1, h=h, stride=stride, fd=fd))

    def hold(self, h: float = 0.0, s1: float = 0.0, fd: float = 0.0, stride: float = 0.0) -> None:
        """STAND with centred sticks: the posture only."""
        self.set_mode(STAND)
        self._send(dict(h=h, s1=s1, fd=fd, stride=stride))

    def goal_move(self, kind: int, distance: float, angle: float, radius: float, speed: float, gait: int,
                  stride: float, s1: float, h: float, fd: float) -> int:
        """VirtualRobot.goal_move, after the last mode request, as a transport's round trip has it. An accepted goal
        takes a standing robot into WALK."""
        if self._mode_undelivered:
            self.tick()
        status = self.robot.goal_move(kind, distance, angle, radius, speed, gait, stride, s1, h, fd)
        if status == 202 and self._mode == STAND:
            self._mode = WALK
        return status

    def goal_cancel(self) -> None:
        self.robot.goal_cancel()

    def goal_status(self) -> list[float]:
        return self.robot.goal_status()

    def odometry(self) -> list[float]:
        return self.robot.odometry()

    def reset_odometry(self) -> bool:
        return self.robot.reset_odometry()

    def select_gait(self, gait: int) -> None:
        self.robot.set_gait(gait)

    def tick(self) -> None:
        if self._sticks is not None and self.ticks % HEARTBEAT_TICKS == 0:
            self.robot.command(**self._sticks)
        # Physics runs whole steps locked to the ticks, up to half a step behind a tick's time, so the gyro is
        # averaged over the 5 ms that end half a step before it: the windows tile time and integrate to the true yaw.
        end = self._start + self.ticks * FIRMWARE_DT - self.sim.model.opt.timestep / 2
        gyro = self.sim.mean_gyro(end - FIRMWARE_DT, end)
        self.robot.set_imu(*imu_reading(self.sim.base_quat(), gyro))
        self.robot.tick()
        self.sim.set_powered(self.robot.servos_driven())
        # Only a tick that wrote a pose moves the targets: in IDLE a raw PWM write holds, as on the robot.
        if self.robot.servo_writes() != self._writes:
            self._writes = self.robot.servo_writes()
            self.sim.set_joint_targets(np.radians(self.robot.servo_angles()))
        self._mode_undelivered = False
        self.ticks += 1
        # 5 ms is not a whole number of physics steps, so keep physics time locked to tick time.
        due = int(round(self.ticks * FIRMWARE_DT / self.sim.model.opt.timestep))
        self.sim.step_physics(due - self.physics_steps)
        self.physics_steps = due

    def _send(self, sticks: dict) -> None:
        self._sticks = sticks
        self.robot.command(**sticks)
