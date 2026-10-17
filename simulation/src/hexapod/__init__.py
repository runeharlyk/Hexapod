"""One Robot for the simulation and the real hexapod: Robot("simulation"), Robot("192.168.4.1") or Robot("COM5")."""
from .backends.base import (Gait, GoalState, GoalStatus, GoalTimedOut, Mode, Odometry, RobotDisconnected, RobotError,
                            RobotState, RobotTimeout, Style, Velocity)
from .robot import Robot
from .speed_model import UnmodelledGait

__all__ = ["Robot", "Gait", "Mode", "Velocity", "RobotState", "RobotError", "RobotTimeout", "RobotDisconnected",
           "UnmodelledGait", "Style", "GoalState", "GoalStatus", "GoalTimedOut", "Odometry"]
