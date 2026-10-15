"""MotionService's STAND/WALK control tick (firmware/include/motion.h) driving the MuJoCo robot.

The stick command reaches the gait through the firmware's first-order ramp (approachGaitCommand), the body height
through its lerp, a gait selection is honoured only while the gait is not stepping (updateAutoGait), and physics
stays locked to the 5 ms control tick.
"""

from src.robot.firmware_gait import (AUTO, BodyState, GaitController, GaitState, approach_gait_command,
                                     command_speed01, command_to_walk_gait, gait_is_commanded, select_auto_gait,
                                     set_gait)
from src.sim.mj_runtime import HexapodSim

FIRMWARE_DT = 0.005  # control-task period (firmware/src/main.cpp)
BODY_SMOOTHING = 0.06  # MotionService::smoothing_factor
HEIGHT_MM_PER_UNIT = 50.0  # handleCommand: target_body_state.zm = c.h * 50


class FirmwareLoop:
    def __init__(self, sim: HexapodSim, gait: int):
        self.sim = sim
        self.target = GaitState()
        self.live = GaitState()
        self.requested_gait = gait
        self.target_zm = 0.0
        self.gc, self.body = GaitController(), BodyState()
        self.ticks = 0
        self.physics_steps = 0
        self._apply_requested_gait()

    def walk(self, lx: float, ly: float, rx: float, s: float = 0.0, s1: float = 0.0, h: float = 0.0) -> None:
        """handleCommand's WALK branch."""
        self.target_zm = command_to_walk_gait(lx=lx, ly=ly, rx=rx, s=s, s1=s1, gait=self.target, h=h)

    def hold(self, h: float = 0.0) -> None:
        """handleCommand's STAND branch for centred sticks: no stride, only the height."""
        self.stop(snap=False)
        self.target_zm = h * HEIGHT_MM_PER_UNIT

    def stop(self, snap: bool) -> None:
        """stopLocomotionCommand: zeroes the command; `snap` also collapses the live gait onto it."""
        default = GaitState()
        for name in ("step_x", "step_y", "step_angle", "step_speed", "step_height", "step_depth", "phase_rate"):
            setattr(self.target, name, getattr(default, name))
        if snap:
            approach_gait_command(self.live, self.target, 0.0, 0.0)
            self.gc.set_phase(0.0)

    def select_gait(self, gait: int) -> None:
        """handleInputGait: a schedule cannot be swapped mid-stride, so a selection while stepping waits."""
        self.requested_gait = gait
        if not gait_is_commanded(self.live):
            self._apply_requested_gait()

    def _apply_requested_gait(self) -> None:
        self.live.gait_type = (select_auto_gait(command_speed01(self.target), self.live.gait_type)
                               if self.requested_gait == AUTO else self.requested_gait)
        set_gait(self.live)
        self.target.gait_type = self.live.gait_type
        self.target.offset = self.live.offset.copy()
        self.target.stand_frac = self.live.stand_frac

    def tick(self, actuated: bool = True) -> None:
        """One control tick; an unactuated tick (DEACTIVATED) only advances physics."""
        if actuated:
            if not gait_is_commanded(self.live):
                self._apply_requested_gait()
            approach_gait_command(self.live, self.target, FIRMWARE_DT)
            self.body.zm += (self.target_zm - self.body.zm) * BODY_SMOOTHING
            self.gc.step(self.live, self.body, FIRMWARE_DT)
            self.sim.set_joint_targets(self.sim.body_targets_from_feet(self.body))
        self.ticks += 1
        # 5 ms is not a whole number of physics steps, so keep physics time locked to tick time.
        due = int(round(self.ticks * FIRMWARE_DT / self.sim.model.opt.timestep))
        self.sim.step_physics(due - self.physics_steps)
        self.physics_steps = due
