"""The robot's WALK_NN control path, from the firmware's native build (firmware/host/policy_api.cpp): its
self-levelling, PolicyCore's observation and action decoding, the gait and the inverse kinematics. The training
environment chooses the actions; everything between the IMU and the joint targets is the code the robot runs.
"""
import ctypes

import numpy as np

from src.sim.virtual_robot import native_library

# firmware/include/imu_frame.h: the MPU6050 DMP's roll, pitch and yaw against the body frame.
DMP_ANGLE_SIGNS = (1.0, -1.0, -1.0)


def dmp_angles(rpy) -> tuple[float, float, float]:
    """Body-frame roll, pitch and yaw as the robot's DMP reports them, which is what the firmware reads."""
    return tuple(sign * angle for sign, angle in zip(DMP_ANGLE_SIGNS, rpy))


class PolicyParams(ctypes.Structure):
    """firmware/include/policy/policy_core.h's PolicyParams, field for field; FirmwarePolicy checks the size."""
    _fields_ = [
        ("action_dim", ctypes.c_int),
        ("obs_history", ctypes.c_int),
        ("hist_stride", ctypes.c_int),
        ("action_tau_s", ctypes.c_float),
        ("foot_residual_mm", ctypes.c_float),
        ("step_xy_mm", ctypes.c_float),
        ("step_angle_rad", ctypes.c_float),
        ("step_height_min_mm", ctypes.c_float),
        ("step_height_max_mm", ctypes.c_float),
        ("phase_rate_min", ctypes.c_float),
        ("phase_rate_max", ctypes.c_float),
        ("step_depth_mm", ctypes.c_float),
        ("body_zm_mm", ctypes.c_float),
        ("body_ride_mm", ctypes.c_float),
        ("gait_delta_gain", ctypes.c_float * 5),
        ("tri_offset", ctypes.c_float * 6),
        ("bi_offset", ctypes.c_float * 6),
        ("tri_stand_frac", ctypes.c_float),
        ("bi_stand_frac", ctypes.c_float),
    ]


class ImuReading(ctypes.Structure):
    """The IMU as Peripherals reports it: world up in the body frame, body rates (rad/s), roll, pitch and yaw (rad)."""
    _fields_ = [("gravity_up", ctypes.c_float * 3), ("gyro_rad", ctypes.c_float * 3), ("rpy_rad", ctypes.c_float * 3)]


def _bind(lib: ctypes.CDLL) -> ctypes.CDLL:
    floats = ctypes.POINTER(ctypes.c_float)
    lib.policyParamsSize.restype = ctypes.c_int
    lib.policyCreate.argtypes = [ctypes.POINTER(PolicyParams)]
    lib.policyCreate.restype = ctypes.c_void_p
    lib.policyDestroy.argtypes = lib.policyReset.argtypes = [ctypes.c_void_p]
    lib.policySetCommand.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.policyObserve.argtypes = [ctypes.c_void_p, ctypes.POINTER(ImuReading), ctypes.c_float, floats]
    lib.policyObserve.restype = ctypes.c_int
    lib.policyAct.argtypes = [ctypes.c_void_p, floats, floats]
    lib.policyAct.restype = floats
    lib.policyAction.argtypes = lib.policyState.argtypes = [ctypes.c_void_p]
    lib.policyAction.restype = lib.policyState.restype = floats
    if lib.policyParamsSize() != ctypes.sizeof(PolicyParams):
        raise RuntimeError("PolicyParams differs from firmware/include/policy/policy_core.h")
    return lib


_lib: ctypes.CDLL | None = None


class FirmwarePolicy:
    """One robot's WALK_NN path. observe() returns what the network sees; act() applies what it chose."""

    def __init__(self, params: PolicyParams):
        global _lib
        _lib = _lib or _bind(native_library())
        self._lib = _lib
        self._handle = self._lib.policyCreate(ctypes.byref(params))
        if not self._handle:
            raise ValueError("PolicyCore runs residual_pure (18 actions) or residual_gait (24) with a history of at "
                             "most 64 sensor frames")
        self.action_dim = params.action_dim
        self.obs_dim = 6 * params.obs_history + 3 + 18 + 2 + 1 + 3 + params.action_dim
        self._obs = np.zeros(self.obs_dim, dtype=np.float32)

    def __del__(self):
        if getattr(self, "_handle", None):
            self._lib.policyDestroy(self._handle)

    def reset(self) -> None:
        self._lib.policyReset(self._handle)

    def set_command(self, vx: float, vy: float, yaw_rate: float) -> None:
        self._lib.policySetCommand(self._handle, vx, vy, yaw_rate)

    def observe(self, gravity_up, gyro_rad, rpy_rad, dt: float) -> np.ndarray:
        imu = ImuReading((ctypes.c_float * 3)(*gravity_up), (ctypes.c_float * 3)(*gyro_rad),
                         (ctypes.c_float * 3)(*rpy_rad))
        n = self._lib.policyObserve(self._handle, ctypes.byref(imu), dt,
                                    self._obs.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
        assert n == self.obs_dim
        return self._obs.copy()

    def act(self, action, base_gait) -> np.ndarray:
        """The joint targets in radians, coxa, femur and tibia per leg in leg order."""
        action = np.ascontiguousarray(action, dtype=np.float32)
        base_gait = np.ascontiguousarray(base_gait, dtype=np.float32)
        angles = self._lib.policyAct(self._handle, action.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                                     base_gait.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
        return np.radians(np.ctypeslib.as_array(angles, (18,)).astype(np.float64))

    def applied_action(self) -> np.ndarray:
        """The low-pass filtered action the last act() applied, which the next observation reports."""
        return np.ctypeslib.as_array(self._lib.policyAction(self._handle), (self.action_dim,)).copy()

    def state(self) -> dict:
        """What the last action commanded."""
        blend, step_height, cadence, duty, phase, zm = np.ctypeslib.as_array(self._lib.policyState(self._handle), (6,))
        return {"blend": float(blend), "step_height_mm": float(step_height), "cadence": float(cadence),
                "duty": float(duty), "phase": float(phase), "body_zm": float(zm)}
