"""One Robot for the simulation and the real hexapod: Robot("simulation") or Robot("192.168.4.1").

Velocities are REP-103 (forward, left, counter-clockwise) in m/s and rad/s, angles in degrees; backends/base.py has
the mapping onto the robot's X-right, Y-forward body frame. Leaving the `with` block, on an exception or Ctrl-C
included, stops the robot, sits it down and switches its servos off.

    with Robot("simulation") as robot:
        robot.stand()
        robot.move_forward(0.3)
        robot.rotate(90)
"""
from . import motions
from .backends.base import ZERO, Backend, Gait, Mode, RobotError, RobotState, Velocity

# The modes a script may walk from. DEACTIVATED is the app's stop button and IDLE, POSE and ANIMATE are the user's:
# walking would take over servos the user put somewhere else.
WALKS_FROM = (Mode.STAND, Mode.WALK)
HEIGHT_RANGE_MM = 50.0  # motion.h handleCommand: zm = h * 50, and a positive zm lowers the body
SIT_S = 0.6  # the body's height lerp (0.06 per 5 ms tick) settles well inside this


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
        self._gait = gait

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
        self._backend.connect()
        try:
            self._backend.set_gait(self._gait)
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        self._backend.close()

    def stand(self, height_mm: float = 0.0) -> None:
        """Stands with the body `height_mm` above the nominal stance, within +-50 mm."""
        if abs(height_mm) > HEIGHT_RANGE_MM:
            raise ValueError(f"the body moves {HEIGHT_RANGE_MM:g} mm either way, not {height_mm:g}")
        self.stop()
        self._backend.set_height(-height_mm / HEIGHT_RANGE_MM)
        self._backend.set_mode(Mode.STAND)

    def rest(self) -> None:
        """Lowers a standing robot onto its belly, then switches the servos off: a drop of a few millimetres
        instead of the full 50 mm a standing robot falls when deactivated."""
        self.stop()
        try:
            if self.state().mode in WALKS_FROM:
                self._backend.set_mode(Mode.STAND)
                self._backend.set_height(1.0)
                self._backend.sleep(SIT_S)
        finally:
            self._backend.set_mode(Mode.DEACTIVATED)

    def deactivate(self) -> None:
        self.stop()
        self._backend.set_mode(Mode.DEACTIVATED)

    def set_gait(self, gait: Gait) -> None:
        """Stops first: the firmware swaps the leg schedule only once the gait has stopped stepping."""
        self.stop()
        self._backend.set_gait(gait)
        self._gait = gait
        self._backend.sleep(motions.SETTLE_S)

    def set_velocity(self, vx: float, vy: float, yaw_rate: float) -> Velocity:
        velocity = Velocity(vx, vy, yaw_rate)
        if velocity != ZERO:
            mode = self._backend.state().mode
            if mode not in WALKS_FROM:
                raise RobotError(f"the robot is {mode.name if mode is not None else 'in an unknown mode'}: "
                                 "stand() it before walking")
            if mode is not Mode.WALK:
                self._backend.set_mode(Mode.WALK)  # STAND reads the same sticks as a shift and tilt of the body
        return self._backend.set_velocity(velocity)

    def stop(self) -> None:
        self._backend.set_velocity(ZERO)

    def state(self) -> RobotState:
        return self._backend.state()

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
