"""One Robot for the simulation and the real hexapod: Robot("simulation"), Robot("192.168.4.1") over WiFi, or
Robot("COM5") (any COMx or /dev/ port) over the robot's native USB port.

Velocities are REP-103 (forward, left, counter-clockwise) in m/s and rad/s, angles in degrees; backends/base.py has
the mapping onto the robot's X-right, Y-forward body frame. Leaving the `with` block, on an exception or Ctrl-C
included, stops the robot, sits it down and switches its servos off.

Walks, turns and arcs are goals: the robot walks each to its end on its own odometry, in the style last set. A turn
or an arc turns from the heading the script's goals meant to leave the robot at, not the one a walk drifted to, so a
square closes on the robot's own heading.

    with Robot("simulation") as robot:
        robot.stand()
        robot.style(speed=0.8)
        for _ in range(4):
            robot.walk(1.0)
            robot.turn(90)
"""
from contextlib import contextmanager, suppress
from dataclasses import replace

from src.robot.firmware_gait import DEFAULT_STEP_HEIGHT_MM, GaitState, approach_gait_command

from ..sim.recording import RecordingWriter
from . import motions
from .backends.base import (ZERO, Backend, Gait, Goal, GoalKind, GoalState, GoalStatus, GoalTimedOut, Mode, RobotError,
                            RobotState, RobotTimeout, Stance, Style, Velocity)
from .servos import PWM_MAX, ServoCalibration

# The modes a script may walk from. DEACTIVATED is the app's stop button and IDLE and ANIMATE are the user's:
# walking would take over servos the user put somewhere else.
WALKS_FROM = (Mode.STAND, Mode.WALK)
HEIGHT_RANGE_MM = 50.0  # motion.h handleCommand: zm = h * 50, and a positive zm lowers the body
SIT_S = 0.6  # the body's height lerp (0.06 per 5 ms tick) settles well inside this
STEP_HEIGHT_MM = 20.0  # motion.h: the feet lift (s1 + 1) * 20 mm
GOAL_POLL_S = 0.05
# A stance change steps the feet at the firmware's repositioning pace, one cycle of the gait a second (gait.h: its phase
# advances at step_speed 1). Every foot swings once a cycle, and one whose swing's last tick leaves it more than 0.5 mm
# short of its target swings again the next cycle, which TRI's long swing never does and the shorter swings do
# (virtual robot from a still stand: TRI 1.00 s, BI 1.42, WAVE and RIPPLE 1.98). AUTO and TUNED count as the longest.
REPOSITION_HZ = 1.0
STANCE_CYCLES = {Gait.TRI_GATE: 1, Gait.BI_GATE: 2, Gait.WAVE: 2, Gait.RIPPLE: 2}
# The stance is taken at the control task's next tick, and a foot lands on the tick after its swing ends.
STANCE_TAKE_S = 2 * motions.CONTROL_TICK_S
POSTURE_SETTLED_MM = 0.5  # motion.h: a lift has arrived within this
NO_GOALS = "the robot's firmware takes no goals"


def _height(height_mm: float) -> float:
    """The firmware's height input for the body `height_mm` above the nominal stance."""
    if abs(height_mm) > HEIGHT_RANGE_MM:
        raise ValueError(f"the body moves {HEIGHT_RANGE_MM:g} mm either way, not {height_mm:g}")
    return -height_mm / HEIGHT_RANGE_MM


def _lift_seconds(from_mm: float, to_mm: float) -> float:
    """How long the firmware's command ramp (approachGaitCommand, gait.h) takes to bring the lift from `from_mm` within
    POSTURE_SETTLED_MM of `to_mm`."""
    live, target, seconds = GaitState(step_height=from_mm), GaitState(step_height=to_mm), 0.0
    while abs(live.step_height - to_mm) >= POSTURE_SETTLED_MM:
        approach_gait_command(live, target, motions.CONTROL_TICK_S)
        seconds += motions.CONTROL_TICK_S
    return seconds


def stance_step_seconds(gait: Gait | None, walking: bool, lift_mm: float) -> float:
    """How long the feet take to stand on a stance asked for now, in `gait`, once the robot has stopped. A walk settles
    into it as into a posture: its lift comes down to the stand's first and goes back up to `lift_mm` after."""
    seconds = STANCE_CYCLES.get(gait, max(STANCE_CYCLES.values())) / REPOSITION_HZ + STANCE_TAKE_S
    if walking:
        seconds += _lift_seconds(lift_mm, DEFAULT_STEP_HEIGHT_MM) + _lift_seconds(DEFAULT_STEP_HEIGHT_MM, lift_mm)
    return seconds


def _wrap(degrees: float) -> float:
    """Into (-180, 180]."""
    return 180.0 - (180.0 - degrees) % 360.0


def _slider(name: str, value: float) -> float:
    if not -1.0 <= value <= 1.0:
        raise ValueError(f"{name} is a slider in [-1, 1], not {value:g}")
    return value


class Robot:
    def __init__(self, target: str = "simulation", *, gait: Gait = Gait.TRI_GATE, viewer: bool = False,
                 backend: Backend | None = None):
        if backend is None:
            if target == "simulation":
                from .backends.sim import SimBackend

                backend = SimBackend(viewer=viewer)
            else:
                from .backends.real import RealBackend

                backend = RealBackend(target)
        self._backend = backend
        self._style = Style(gait=gait)
        # The odometry heading the goals so far meant to leave the robot at, in degrees; None until a turn takes the
        # heading it finds, and again once a velocity walk or a goal that did not end where it meant has moved it.
        self._intended: float | None = None

    def __enter__(self) -> "Robot":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        try:
            self.rest()
        except RobotError:
            pass  # the robot did not answer; if the link is gone, its dead-man stop halts a walk within 600 ms
        finally:
            self.close()

    # The speed model is per gait, so the robot is put in the one the script asked for.
    def connect(self) -> None:
        self._intended = None
        self._backend.connect()
        try:
            self._backend.set_gait(self._style.gait)
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        self._backend.close()

    def stand(self, height_mm: float = 0.0) -> None:
        """Stands with the body `height_mm` above the nominal stance, within +-50 mm; the style's body height from
        now."""
        height = _height(height_mm)
        self.stop()
        self._backend.set_height(height)
        self._backend.set_mode(Mode.STAND)
        self._style = replace(self._style, h=height)

    def rest(self) -> None:
        """Lowers a standing robot onto its belly, then switches the servos off: a drop of a few millimetres
        instead of the full 50 mm a standing robot falls when deactivated. Lowering takes STAND, which would stand up a
        robot that is not standing, so the robot is asked for its mode rather than trusted to be in the one last heard,
        and a robot whose mode stays unknown is switched off where it is. It is switched off however the stop or the
        lowering ends, a second Ctrl-C included."""
        try:
            self.stop()
            if self._backend.fresh_mode() in WALKS_FROM:
                self._backend.set_mode(Mode.STAND)
                self._backend.set_height(1.0)
                self._backend.sleep(SIT_S)
        finally:
            self._backend.set_mode(Mode.DEACTIVATED)

    def deactivate(self) -> None:
        self.stop()
        self._backend.set_mode(Mode.DEACTIVATED)

    def idle(self) -> None:
        """Stops the motion and leaves every servo powered where it is (IDLE): the one mode in which a PWM written
        with set_servo_pwm stays, since every other mode writes the motion's angles over it on its next tick."""
        self.stop()
        self._backend.set_mode(Mode.IDLE)

    def servo_calibration(self) -> ServoCalibration:
        return self._backend.servo_calibration()

    def set_servo_pwm(self, servo: int, pwm: int) -> None:
        """Writes one servo's raw PWM in PCA9685 counts (0-4095); see idle(). The robot writes it in any mode and holds
        it to no joint's travel or servo's range, so keep it within the servo's own (servo_calibration()); a PWM out of
        0-4095 is refused here, as the firmware would take 4096 as a channel switched fully on."""
        if not 0 <= pwm <= PWM_MAX:
            raise ValueError(f"a PWM is 0-{PWM_MAX} PCA9685 counts, not {pwm}")
        self._backend.set_servo_pwm(servo, pwm)

    def set_joint(self, servo: int, degrees: float) -> None:
        """Drives one joint (servo 0-17, coxa, femur and tibia per leg) to `degrees` as the telemetry reports its
        angles, held within the joint's travel; the robot takes it in IDLE alone and ignores it in every other mode."""
        self._backend.set_joint(servo, degrees)

    def features(self) -> dict:
        """The robot's features: firmwareVersion and firmwareElfSha256 among them."""
        return self._backend.features()

    @contextmanager
    def recording(self, path):
        """Writes the robot's per-tick telemetry to the .hexrec file `path` while the block runs, as record.py does
        (src/sim/recording.py); the file is complete up to the last batch also when the block raises."""
        writer = RecordingWriter(path, self._backend.features())
        try:
            self._backend.stream_telemetry(writer.write)
            yield
        finally:
            try:
                self._backend.stream_telemetry(None)
            finally:
                writer.close()

    def set_gait(self, gait: Gait) -> None:
        """Stops first: the firmware swaps the leg schedule only once the gait has stopped stepping. The style's gait
        from now."""
        self.stop()
        self._backend.set_gait(gait)
        self._style = replace(self._style, gait=gait)
        self._backend.sleep(motions.SETTLE_S)

    def set_stance(self, stance: Stance) -> None:
        """Stands the feet on `stance`: DEFAULT, or CRAB, every leg straight out to its side, which walks sideways.
        Stops first, and returns once the robot reports the stance and its feet stand on it, stepped there through the
        gait's own schedule, a walk's after its stop (stance_step_seconds); a walk's lift may still be rising back for up
        to 0.32 s. The robot refuses AUTO, TUNED and
        WALK_NN in the crab and switches a selected AUTO or TUNED to TRI_GATE, so the style's gait follows; a stance it
        refuses, the crab in WALK_NN, raises RobotError. Goals walk the stance in force at the speeds asked;
        set_velocity's speed model was measured on the default stance, so in the crab it sends the sticks a velocity
        takes there, which the crab walks at its own speeds."""
        walking = self._backend.state().mode is Mode.WALK
        self.stop()
        self._backend.set_stance(stance)
        if stance is Stance.CRAB and self._style.gait in (Gait.AUTO, Gait.TUNED):
            self._style = replace(self._style, gait=Gait.TRI_GATE)
        lift = (self._style.s1 + 1.0) * STEP_HEIGHT_MM
        self._backend.sleep((motions.SETTLE_S if walking else 0.0)
                            + stance_step_seconds(self._backend.state().gait, walking, lift))

    def set_velocity(self, vx: float, vy: float, yaw_rate: float) -> Velocity:
        """Walks at `vx`, `vy` (m/s) and `yaw_rate` (rad/s), REP-103, through the speed model's sticks, and returns what
        was commanded after clamping. The speed model was measured on the default stance: in the crab (set_stance) it
        sends the sticks the velocity takes there, which the crab walks at its own speeds, so the velocity is not the
        one asked; goals walk the crab at the speeds asked."""
        velocity = Velocity(vx, vy, yaw_rate)
        if velocity != ZERO:
            mode = self._backend.state().mode
            if mode not in WALKS_FROM:
                raise RobotError(f"the robot is {mode.name if mode is not None else 'in an unknown mode'}: "
                                 "stand() it before walking")
            if mode is not Mode.WALK:
                self._backend.set_mode(Mode.WALK)  # STAND reads the same sticks as a shift and tilt of the body
            self._intended = None
        return self._backend.set_velocity(velocity)

    def stop(self) -> None:
        self._backend.set_velocity(ZERO)

    def state(self) -> RobotState:
        state = self._backend.state()
        return replace(state, odometry=self._backend.odometry()) if self.takes_goals() else state

    def reset_odometry(self) -> None:
        """Zeroes the odometry's pose in the frame the robot faces, which is then the heading the goals mean; returns
        once the robot's odometry reads it, so call it with the robot still. The robot refuses it, a RobotError, while
        a goal runs."""
        self._backend.reset_odometry()
        self._intended = 0.0

    def max_velocity(self) -> Velocity:
        return self._backend.max_velocity()

    def time(self) -> float:
        return self._backend.now()

    def sleep(self, seconds: float) -> None:
        self._backend.sleep(seconds)

    def move_forward(self, metres: float, speed: float | None = None) -> None:
        motions.move(self, metres, 0.0, speed)

    def move_sideways(self, metres: float, speed: float | None = None) -> None:
        """Positive is to the robot's left."""
        motions.move(self, 0.0, metres, speed)

    def rotate(self, degrees: float, rate: float | None = None) -> None:
        """Positive is counter-clockwise seen from above."""
        motions.rotate(self, degrees, rate)

    # --- goals ----------------------------------------------------------------------------------------------------
    def style(self, gait: Gait | None = None, speed: float | None = None, stride: float | None = None,
              step_height: float | None = None, body_height: float | None = None,
              feet_distance: float | None = None) -> Style:
        """Sets how goals walk; an omitted field keeps its value. `speed` is a share in (0, 1] of the gait's top speed
        at the posture, `step_height` how high the feet lift in mm (0 to 40), `body_height` mm above the nominal stance
        (within +-50, as stand()'s), and `stride` and `feet_distance` are the app's sliders in [-1, 1]. Every command
        after a goal carries its posture, so the robot keeps it. Returns the style in the firmware's terms."""
        changes = {}
        if gait is not None:
            changes["gait"] = gait
        if speed is not None:
            if not 0.0 < speed <= 1.0:
                raise ValueError(f"speed is a share in (0, 1] of the gait's top, not {speed:g}")
            changes["speed"] = speed
        if stride is not None:
            changes["stride"] = _slider("stride", stride)
        if step_height is not None:
            if not 0.0 <= step_height <= 2 * STEP_HEIGHT_MM:
                raise ValueError(f"the feet lift 0 to {2 * STEP_HEIGHT_MM:g} mm, not {step_height:g}")
            changes["s1"] = step_height / STEP_HEIGHT_MM - 1.0
        if body_height is not None:
            changes["h"] = _height(body_height)
        if feet_distance is not None:
            changes["fd"] = _slider("feet_distance", feet_distance)
        self._style = replace(self._style, **changes)
        return self._style

    def takes_goals(self) -> bool:
        return self._backend.supports_goals()

    def walk(self, metres: float, sideways: bool = False, on_progress=None,
             speed: float | None = None) -> GoalStatus | None:
        """Walks `metres` forward (negative is backwards), or to the left with `sideways` (negative is right), holding
        the heading it starts on; the next turn takes up what it drifts. `speed` is a share, as run_goal's."""
        if metres == 0:
            return None
        if not self.takes_goals():
            raise RobotError(NO_GOALS)
        if self._intended is None:
            self._intended = self._backend.odometry().yaw
        try:
            return self.run_goal(Goal(GoalKind.SIDEWAYS if sideways else GoalKind.FORWARD, distance=metres),
                                 speed, on_progress)
        except BaseException:
            self._intended = None
            raise

    def turn(self, degrees: float, on_progress=None, speed: float | None = None) -> GoalStatus | None:
        """Turns in place, counter-clockwise when positive, from the heading the goals so far meant: after a walk that
        drifted 2 deg to the left a 90 deg turn turns 88. `speed` is a share, as run_goal's."""
        if degrees == 0:
            return None
        return self._turning(GoalKind.TURN, degrees, 0.0, speed, on_progress)

    def arc(self, radius: float, degrees: float, on_progress=None) -> GoalStatus | None:
        """Walks forward along a circle of `radius` m through `degrees`, curving left when positive; 360 is a circle.
        Its angle takes up the heading drift as a turn's does."""
        if radius <= 0:
            raise ValueError(f"an arc needs a positive radius, not {radius:g}; a negative angle curves right")
        if degrees == 0:
            return None
        return self._turning(GoalKind.ARC, degrees, radius, None, on_progress)

    # Only the drift is wrapped, so a turn or a circle of 360 deg stays one.
    def _turning(self, kind: GoalKind, degrees: float, radius: float, speed: float | None,
                 on_progress) -> GoalStatus | None:
        if not self.takes_goals():
            raise RobotError(NO_GOALS)
        current = self._backend.odometry().yaw
        intended = current if self._intended is None else self._intended
        angle = degrees + _wrap(intended - current)
        self._intended = None
        status = None if angle == 0 else self.run_goal(Goal(kind, angle=angle, radius=radius), speed, on_progress)
        self._intended = intended + degrees
        return status

    def run_goal(self, goal: Goal, speed: float | None = None, on_progress=None) -> GoalStatus:
        """Sends `goal` in the style, at the share `speed` instead of the style's when given, and blocks until it ends.
        Returns its REACHED status; raises GoalTimedOut when it TIMED_OUT, and RobotError when it was CANCELLED or
        REFUSED or another client's goal replaced it. `on_progress` is called with each new status. Any other
        exception before the goal is known to have ended, a lost status reply (RobotTimeout) and Ctrl-C included,
        cancels it as far as the link still allows and is raised."""
        if not self.takes_goals():
            raise RobotError(NO_GOALS)
        style = self._style if speed is None else replace(self._style, speed=speed)
        try:
            status = self._backend.goal_move(goal, style)
        except (RobotTimeout, KeyboardInterrupt):
            self._cancel_goal()  # the robot may have taken the goal and lost only the reply
            raise
        number, reported = status.id, None
        try:
            # Until a newer goal replaces this one, which ends it: a cancel then would end the newer one.
            while status.id <= number:
                if status.id == number and status != reported:
                    reported = status
                    if on_progress is not None:
                        on_progress(status)
                    if status.ended:
                        break
                self._backend.sleep(GOAL_POLL_S)
                status = self._backend.goal_status()
        except BaseException:
            if not (status.id == number and status.ended):
                self._cancel_goal()
            raise
        if status.id > number:
            raise RobotError(f"another goal replaced goal {number}")
        if status.state is GoalState.REACHED:
            return status
        error = GoalTimedOut if status.state is GoalState.TIMED_OUT else RobotError
        raise error(f"the {goal.kind.name} goal ended {status.state.name} at {status.progress:.0%} of "
                    f"{status.target:g} after {status.passes} passes")

    def _cancel_goal(self) -> None:
        with suppress(RobotError):  # a link that is gone ends the goal itself, through its owner's silence
            self._backend.goal_cancel()
