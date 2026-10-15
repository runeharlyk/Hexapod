"""One Robot for the simulation and the real hexapod: Robot("simulation") or Robot("192.168.4.1")."""
from .backends.base import Gait, Mode, RobotDisconnected, RobotError, RobotState, RobotTimeout, Velocity
from .robot import Robot
from .speed_model import UnmodelledGait

__all__ = ["Robot", "Gait", "Mode", "Velocity", "RobotState", "RobotError", "RobotTimeout", "RobotDisconnected",
           "UnmodelledGait"]
