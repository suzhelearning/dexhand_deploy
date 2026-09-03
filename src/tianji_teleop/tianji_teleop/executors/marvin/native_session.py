"""ctypes boundary for the dedicated 200 Hz C++ Marvin driver."""
from __future__ import annotations

import ctypes
import os
from pathlib import Path
import sys
import time
from typing import Any, Mapping

import numpy as np

from ...marvin_hardware import MarvinFeedback, MarvinHardwareError, MarvinHardwareSession


_JOINTS = 7
_ARMS = 2
_ERROR_BYTES = 512


class _NativeConfig(ctypes.Structure):
    _fields_ = [
        ("rate_hz", ctypes.c_int),
        ("velocity_ratio", ctypes.c_int),
        ("acceleration_ratio", ctypes.c_int),
        ("velocity_estimation_step_ms", ctypes.c_long),
        ("joint_stiffness", ctypes.c_double * _JOINTS),
        ("joint_damping", ctypes.c_double * _JOINTS),
        ("tool_kinematics", ctypes.c_double * 6),
        ("tool_dynamics", (ctypes.c_double * 10) * _ARMS),
        ("command_timeout_s", ctypes.c_double),
    ]


class _NativeFeedback(ctypes.Structure):
    _fields_ = [
        ("joints_deg", (ctypes.c_double * _JOINTS) * _ARMS),
        ("arm_states", ctypes.c_int * _ARMS),
        ("command_states", ctypes.c_int * _ARMS),
        ("error_codes", ctypes.c_int * _ARMS),
        ("frame_serials", ctypes.c_int * _ARMS),
        ("velocity_ratios", ctypes.c_int * _ARMS),
        ("acceleration_ratios", ctypes.c_int * _ARMS),
        ("impedance_types", ctypes.c_int * _ARMS),
        ("servo_error_codes", (ctypes.c_long * _JOINTS) * _ARMS),
        ("control_ticks", ctypes.c_uint64),
        ("deadline_misses", ctypes.c_uint64),
        ("healthy", ctypes.c_int),
        ("soft_stopped", ctypes.c_int),
    ]


def _seven(values: Any, label: str) -> list[float]:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (_JOINTS,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain seven finite values")
    return result.tolist()


def _native_library_path() -> Path:
    configured = os.environ.get("TIANJI_MARVIN_NATIVE_LIBRARY", "").strip()
    repository = Path(__file__).resolve().parents[5]
    candidates = [
        Path(configured) if configured else None,
        repository / "staging/ik/lib/libmarvin_native_driver.so",
        repository / "runtime/tianji_teleop/lib/libmarvin_native_driver.so",
        Path(sys.prefix) / "lib/libmarvin_native_driver.so",
    ]
    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return candidate
    raise MarvinHardwareError(
        "native Marvin driver missing; run `pixi run -e ik-build build-ik`"
    )


def _sdk_library_path() -> Path:
    from marvin_sdk import fx_robot

    result = Path(fx_robot.__file__).resolve().with_name("libMarvinSDK.so")
    if not result.is_file():
        raise MarvinHardwareError(f"Marvin SDK library missing: {result}")
    return result


class NativeMarvinHardwareSession(MarvinHardwareSession):
    """Preserves the Python safety contract while SDK I/O runs in C++."""

    required_state = 3

    def __init__(self, params: Mapping[str, Any]) -> None:
        self._sleep = time.sleep
        self._monotonic = time.monotonic
        self._connected = False
        self._soft_stopped = False
        self._handle: int | None = None
        self._latest_native = _NativeFeedback()
        stiffness = _seven(params["joint_stiffness"], "joint_stiffness")
        damping = _seven(params["joint_damping"], "joint_damping")
        tool_kinematics = np.asarray(params["tool_kinematics"], dtype=np.float64)
        tool_config = params["tool_dynamics"]
        if not isinstance(tool_config, Mapping):
            raise ValueError("tool_dynamics must contain left and right values")
        tool_dynamics = np.asarray(
            [tool_config.get("left"), tool_config.get("right")], dtype=np.float64
        )
        if tool_kinematics.shape != (6,) or not np.isfinite(tool_kinematics).all():
            raise ValueError("tool_kinematics must contain six finite values")
        if tool_dynamics.shape != (_ARMS, 10) or not np.isfinite(tool_dynamics).all():
            raise ValueError("tool_dynamics must contain ten finite values per arm")
        self._config = _NativeConfig(
            int(params["rate_hz"]),
            int(params["velocity_ratio"]),
            int(params["acceleration_ratio"]),
            int(params["velocity_estimation_step_ms"]),
            (ctypes.c_double * _JOINTS)(*stiffness),
            (ctypes.c_double * _JOINTS)(*damping),
            (ctypes.c_double * 6)(*tool_kinematics),
            ((ctypes.c_double * 10) * _ARMS)(
                (ctypes.c_double * 10)(*tool_dynamics[0]),
                (ctypes.c_double * 10)(*tool_dynamics[1]),
            ),
            float(params["command_timeout_s"]),
        )
        self._library = ctypes.CDLL(str(_native_library_path()))
        self._bind()
        error = ctypes.create_string_buffer(_ERROR_BYTES)
        handle = self._library.tianji_marvin_native_create(
            str(_sdk_library_path()).encode(), ctypes.byref(self._config),
            error, len(error),
        )
        if not handle:
            raise MarvinHardwareError(error.value.decode(errors="replace"))
        self._handle = int(handle)

    def _bind(self) -> None:
        self._library.tianji_marvin_native_create.argtypes = [
            ctypes.c_char_p, ctypes.POINTER(_NativeConfig),
            ctypes.c_char_p, ctypes.c_size_t,
        ]
        self._library.tianji_marvin_native_create.restype = ctypes.c_void_p
        self._library.tianji_marvin_native_connect.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t,
        ]
        self._library.tianji_marvin_native_connect.restype = ctypes.c_int
        for name in (
            "tianji_marvin_native_set_position_mode",
            "tianji_marvin_native_set_impedance_mode",
        ):
            function = getattr(self._library, name)
            function.argtypes = [
                ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t,
            ]
            function.restype = ctypes.c_int
        vector = ctypes.POINTER(ctypes.c_double)
        self._library.tianji_marvin_native_submit.argtypes = [
            ctypes.c_void_p, vector, vector, ctypes.c_char_p, ctypes.c_size_t,
        ]
        self._library.tianji_marvin_native_submit.restype = ctypes.c_int
        self._library.tianji_marvin_native_read.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(_NativeFeedback),
            ctypes.c_char_p, ctypes.c_size_t,
        ]
        self._library.tianji_marvin_native_read.restype = ctypes.c_int
        self._library.tianji_marvin_native_soft_stop.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p,
        ]
        self._library.tianji_marvin_native_destroy.argtypes = [ctypes.c_void_p]

    def _call(self, function: Any, *args: Any) -> None:
        if self._handle is None:
            raise MarvinHardwareError("native Marvin driver is closed")
        error = ctypes.create_string_buffer(_ERROR_BYTES)
        if function(ctypes.c_void_p(self._handle), *args, error, len(error)) != 1:
            raise MarvinHardwareError(error.value.decode(errors="replace"))

    def connect_and_prepare(
        self,
        robot_ip: str,
        *,
        velocity_ratio: int = 10,
        acceleration_ratio: int = 10,
        lower_limits_deg=None,
        upper_limits_deg=None,
        hard_limit_padding_deg: float = 5.0,
        **_: Any,
    ) -> MarvinFeedback:
        if velocity_ratio != self._config.velocity_ratio or acceleration_ratio != self._config.acceleration_ratio:
            raise MarvinHardwareError("native Marvin ratio/config mismatch")
        self._call(
            self._library.tianji_marvin_native_connect,
            robot_ip.encode(),
        )
        self._connected = True
        feedback = self.read_feedback(include_servo_errors=True)
        hard_limits = self._hard_limit_bounds(
            lower_limits_deg, upper_limits_deg, hard_limit_padding_deg
        )
        self._require_feedback_within_hard_limits(feedback, hard_limits)
        self._require_healthy_state_feedback(
            feedback, self.required_state, "after native impedance enable"
        )
        return feedback

    def send_joint_targets(self, left_joints_deg, right_joints_deg) -> None:
        left = (ctypes.c_double * _JOINTS)(
            *self._joints(left_joints_deg, "left_joints_deg")
        )
        right = (ctypes.c_double * _JOINTS)(
            *self._joints(right_joints_deg, "right_joints_deg")
        )
        self._call(self._library.tianji_marvin_native_submit, left, right)

    def read_feedback(self, *, include_servo_errors: bool = False) -> MarvinFeedback:
        del include_servo_errors
        native = _NativeFeedback()
        self._call(self._library.tianji_marvin_native_read, ctypes.byref(native))
        self._latest_native = native
        reports = tuple(
            "None"
            if not any(native.servo_error_codes[arm])
            else ",".join(
                f"0x{int(value):X}"
                for value in native.servo_error_codes[arm]
                if value
            )
            for arm in range(_ARMS)
        )
        return MarvinFeedback(
            np.asarray(native.joints_deg[0], dtype=np.float64),
            np.asarray(native.joints_deg[1], dtype=np.float64),
            tuple(native.arm_states),
            tuple(native.command_states),
            tuple(native.error_codes),
            tuple(native.frame_serials),
            tuple(native.velocity_ratios),
            tuple(native.acceleration_ratios),
            reports,
        )

    @property
    def diagnostics(self) -> dict[str, Any]:
        native = self._latest_native
        return {
            "hardware_driver": "native_cpp",
            "control_mode": "joint_impedance",
            "required_arm_state": self.required_state,
            "control_ticks": int(native.control_ticks),
            "deadline_misses": int(native.deadline_misses),
            "impedance_types": list(native.impedance_types),
        }

    def move_to_home(self, *args: Any, **kwargs: Any) -> MarvinFeedback:
        # Low-stiffness impedance is correct for teleoperation but leaves a
        # repeatable residual on a large Home traverse.  Home in the SDK's
        # rigid position mode, then restore impedance before returning.
        self._call(self._library.tianji_marvin_native_set_position_mode)
        kwargs["required_state"] = 1
        super().move_to_home(*args, **kwargs)
        self._call(self._library.tianji_marvin_native_set_impedance_mode)
        return self.read_feedback(include_servo_errors=True)

    def soft_stop_once(self) -> None:
        if self._soft_stopped or self._handle is None:
            return
        self._library.tianji_marvin_native_soft_stop(
            ctypes.c_void_p(self._handle), b"executor soft stop"
        )
        self._soft_stopped = True

    def shutdown(self, **_: Any) -> None:
        handle, self._handle = self._handle, None
        if handle is not None:
            self._library.tianji_marvin_native_destroy(ctypes.c_void_p(handle))
        self._connected = False


def create_native_marvin_session(params: Mapping[str, Any]) -> NativeMarvinHardwareSession:
    return NativeMarvinHardwareSession(params)


__all__ = ["NativeMarvinHardwareSession", "create_native_marvin_session"]
