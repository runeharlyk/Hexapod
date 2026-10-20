"""The firmware's Hexapod as a native library: firmware/host/virtual_robot.h, the real MotionService, gait,
kinematics, policy and animation runner on host stand-ins, built with the flags in firmware/features.ini. The same
build the app's simulation view runs as WebAssembly, so the simulation drives the robot with the firmware's code
instead of a port of it.

The library is compiled on first use and again whenever a firmware source is newer than it; the compiler is $CXX,
or g++, and needs C++20. Like the robot it holds one Hexapod per process: a new VirtualRobot starts it over.
The animation clips are not on the host's file system, so ANIMATE has nothing to play here.
The library also holds the WALK_NN control path the training environment runs (firmware_policy.py).
"""
import ctypes
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
FIRMWARE = ROOT / "firmware"
GENERATED = FIRMWARE / "src" / "platform_shared"
WINDOWS = sys.platform == "win32"
LIBRARY = ROOT / ".pio" / "virtual_robot" / ("virtual_robot.dll" if WINDOWS else "libvirtual_robot.so")
SOURCES = [FIRMWARE / "host" / name for name in ("virtual_robot_api.cpp", "policy_api.cpp", "filesystem_host.cpp")]
C_SOURCES = ([ROOT / "submodules" / "nanopb" / name for name in ("pb_common.c", "pb_decode.c", "pb_encode.c")]
             + [GENERATED / f"{name}.pb.c" for name in ("animation", "api", "message")])

DEACTIVATED, STAND, WALK = 0, 3, 4  # ModesEnum's wire values, which setMode takes


def _feature_flags() -> list[str]:
    flags = []
    for line in (FIRMWARE / "features.ini").read_text().splitlines():
        line = line.strip()
        if line.startswith("-D"):
            flags.append("-D" + line[2:].strip())
    return flags


def _inputs() -> list[Path]:
    trees = [FIRMWARE / "host", FIRMWARE / "include", ROOT / "platform_shared"]
    return [p for tree in trees for p in tree.rglob("*") if p.is_file()] + [FIRMWARE / "features.ini",
                                                                            Path(__file__)]


def _build() -> None:
    if not all(p.exists() for p in C_SOURCES[3:]):
        subprocess.run([sys.executable, str(FIRMWARE / "scripts" / "compile_protos.py")], cwd=ROOT, check=True)
    LIBRARY.parent.mkdir(parents=True, exist_ok=True)
    cxx = os.environ.get("CXX", "g++")
    includes = [FIRMWARE / "host", FIRMWARE / "include", FIRMWARE / "src", GENERATED, ROOT / "submodules" / "nanopb"]
    command = [cxx, "-std=gnu++2a", "-O2", "-shared", *_feature_flags(), *(f"-I{d}" for d in includes),
               *(str(s) for s in SOURCES), "-x", "c", *(str(s) for s in C_SOURCES), "-x", "none",
               *(["-static", "-static-libgcc", "-static-libstdc++"] if WINDOWS else ["-fPIC"]), "-o", str(LIBRARY)]
    try:
        subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    except FileNotFoundError as error:
        raise RuntimeError(f"no C++ compiler '{cxx}' for the virtual robot: set CXX to a C++20 g++ or clang++") from error
    except subprocess.CalledProcessError as error:
        raise RuntimeError(f"building the virtual robot failed:\n{error.stderr}") from error


def _library() -> ctypes.CDLL:
    if not LIBRARY.exists() or max(p.stat().st_mtime for p in _inputs()) > LIBRARY.stat().st_mtime:
        _build()
    lib = ctypes.CDLL(str(LIBRARY))
    floats = lambda n: [ctypes.c_float] * n  # noqa: E731
    lib.setMode.argtypes = lib.setGait.argtypes = [ctypes.c_int]
    lib.command.argtypes = floats(9)
    lib.setImu.argtypes = floats(9)
    lib.servosDriven.restype = lib.resetOdometry.restype = ctypes.c_bool
    lib.servoAngles.restype = ctypes.POINTER(ctypes.c_float)
    lib.servoWrites.restype = ctypes.c_uint
    lib.classicWalkTarget.argtypes = [ctypes.c_int, *floats(8), ctypes.POINTER(ctypes.c_float)]
    lib.autoWalk.argtypes = [ctypes.c_int, *floats(8), ctypes.POINTER(ctypes.c_float)]
    lib.autoWalk.restype = ctypes.c_int
    lib.clampPosture.argtypes = [ctypes.POINTER(ctypes.c_float)]
    lib.goalMove.argtypes = [ctypes.c_int, *floats(4), ctypes.c_int, *floats(4)]
    lib.goalMove.restype = ctypes.c_int
    lib.goalStatus.argtypes = lib.odometry.argtypes = [ctypes.POINTER(ctypes.c_float)]
    return lib


_lib: ctypes.CDLL | None = None


def native_library() -> ctypes.CDLL:
    """The firmware's host build, compiled when it is missing or older than a firmware source."""
    global _lib
    _lib = _lib or _library()
    return _lib


class VirtualRobot:
    """One pass of main.cpp's 5 ms control loop per tick(); modes, gaits and commands reach the motion through its
    own EventBus subscriptions, as from a transport."""

    def __init__(self):
        self._lib = native_library()
        self._lib.reset()

    def set_mode(self, mode: int) -> None:
        self._lib.setMode(mode)

    def set_gait(self, gait: int) -> None:
        self._lib.setGait(gait)

    def command(self, lx=0.0, ly=0.0, rx=0.0, ry=0.0, h=0.0, s=0.0, s1=0.0, fd=0.0, stride=0.0) -> None:
        """A ControllerInputData message, as main.cpp's handler turns it into a CommandMsg."""
        self._lib.command(lx, ly, rx, ry, h, s, s1, fd, stride)

    def set_imu(self, rpy_rad, gravity_up, gyro_rad) -> None:
        """The IMU as Peripherals reports it: roll, pitch and yaw in radians, world up in the body frame, and
        body rates in rad/s."""
        self._lib.setImu(*rpy_rad, *gravity_up, *gyro_rad)

    def tick(self) -> None:
        self._lib.tick()

    def servos_driven(self) -> bool:
        """Whether the servos are driven: the boards are awake and a pose has been sent since boot."""
        return self._lib.servosDriven()

    def servo_angles(self) -> list[float]:
        """The 18 joint angles the servos are driven to, in degrees, coxa, femur and tibia per leg in leg order."""
        return self._lib.servoAngles()[:18]

    def servo_writes(self) -> int:
        """Poses sent to the servos since the robot started over."""
        return self._lib.servoWrites()

    def goal_move(self, kind: int, distance: float, angle: float, radius: float, speed: float, gait: int,
                  stride: float, s1: float, h: float, fd: float) -> int:
        """A GoalMove request in the body frame (m and rad) from a client with no transport, so no silence ends it:
        the reply's status, 202 when the goal starts on the next tick."""
        return self._lib.goalMove(kind, distance, angle, radius, speed, gait, stride, s1, h, fd)

    def goal_cancel(self) -> None:
        self._lib.goalCancel()

    def goal_status(self) -> list[float]:
        """The newest goal's GoalStatus fields in their protobuf order: id, state, kind, target, progress, x, y, yaw
        and passes."""
        out = (ctypes.c_float * 9)()
        self._lib.goalStatus(out)
        return list(out)

    def odometry(self) -> list[float]:
        """After the last tick, in the body frame: x, y, yaw, kinematic yaw and distance."""
        out = (ctypes.c_float * 5)()
        self._lib.odometry(out)
        return list(out)

    def reset_odometry(self) -> bool:
        """Takes effect on the next tick; refused, False and nothing reset, while a goal drives."""
        return self._lib.resetOdometry()
