"""Optional XRoboToolkit input boundary for the PICO + VR/Manus route.

The native ``xrobotoolkit_sdk`` module is intentionally imported only when a
live source is constructed.  Importing this module therefore remains safe on
machines which only run PICO2 hand tracking, replay, or MuJoCo tests.

This module owns acquisition and explicit device binding only.  It does not
map poses into robot coordinates, solve IK, publish commands, or authorize a
session.  ``XrFrame`` is a transport-independent snapshot that can be passed
to the canonical observation publisher.
"""
from __future__ import annotations

from dataclasses import dataclass
import importlib
import math
import os
import sys
import time
from typing import Any, Callable, Mapping

import numpy as np


SIDES = ("left", "right")
XR_ARM_INPUTS = frozenset(("xr_controller", "xr_tracker"))
_REQUIRED_XR_SDK_API = (
    "init",
    "get_headset_pose",
    "get_left_controller_pose",
    "get_right_controller_pose",
    "num_motion_data_available",
    "get_motion_tracker_pose",
    "get_motion_tracker_serial_numbers",
    "get_left_trigger",
    "get_right_trigger",
    "get_left_grip",
    "get_right_grip",
    "get_left_axis",
    "get_right_axis",
    "get_motion_timestamp_ns",
)


def validate_xr_sdk_module(sdk: Any) -> tuple[str, ...]:
    """Return missing required Pybind symbols without initializing the SDK.

    The reference Pybind package exposes a module-level API.  Keep the
    required surface explicit so a launcher can reject an incompatible SDK
    before starting the rest of a managed session.  Motion-tracker velocity,
    acceleration and ``close`` remain optional because older reference
    bindings do not expose all three.
    """
    if sdk is None:
        return _REQUIRED_XR_SDK_API
    return tuple(
        name for name in _REQUIRED_XR_SDK_API
        if not callable(getattr(sdk, name, None))
    )


def _finite_nonnegative_int(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or int(value) < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return int(value)


def _identity(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip() or "/" in value:
        raise ValueError(f"{field} must be a non-empty path-safe string")
    return value


def _pose(value: Any, field: str) -> np.ndarray | None:
    if value is None:
        return None
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (7,) or not np.isfinite(result).all():
        raise ValueError(f"{field} must be a finite 7-vector")
    norm = float(np.linalg.norm(result[3:]))
    if norm < 1.0e-12:
        raise ValueError(f"{field} quaternion must be non-zero")
    result[3:] /= norm
    return result


def _optional_pose(value: Any, field: str) -> np.ndarray | None:
    """Decode an SDK pose, treating unavailable/malformed samples as invalid."""
    if value is None:
        return None
    try:
        return _pose(value, field)
    except (TypeError, ValueError):
        return None


def _scalar(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be a finite scalar")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a finite scalar") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be a finite scalar")
    return min(1.0, max(0.0, result))


def _axis(value: Any, field: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (2,) or not np.isfinite(result).all():
        raise ValueError(f"{field} must be a finite 2-vector")
    return np.clip(result, -1.0, 1.0).copy()


def _items(value: Any) -> tuple[Any, ...]:
    """Materialize an SDK sequence without testing its truth value.

    The reference Pybind wrapper returns lists, while some compatible builds
    expose NumPy arrays or other iterable containers.  ``value or []`` is not
    safe for an ndarray because its truth value is ambiguous.  Treat a
    missing/non-iterable result as an empty SDK response and let each item be
    validated at the typed boundary below.
    """
    if value is None or isinstance(value, (str, bytes, bytearray, memoryview)):
        return ()
    try:
        return tuple(value)
    except (TypeError, ValueError):
        return ()


def _serial_matches(actual: str, configured: str) -> bool:
    """Match both full XR serials and the six-digit IDs used by the reference."""
    actual = str(actual).strip()
    configured = str(configured).strip()
    if not actual or not configured:
        return False
    return actual == configured or actual.endswith(configured)


@dataclass(frozen=True)
class XrControllerState:
    side: str
    pose: np.ndarray | None
    valid: bool
    available: bool
    trigger: float
    grip: float
    axis: np.ndarray

    def __post_init__(self) -> None:
        if self.side not in SIDES:
            raise ValueError("controller side must be left or right")
        if not isinstance(self.valid, (bool, np.bool_)) or not isinstance(self.available, (bool, np.bool_)):
            raise ValueError("controller valid/available must be boolean")
        pose = _pose(self.pose, f"{self.side}.controller.pose")
        if bool(self.valid) and pose is None:
            raise ValueError("valid controller requires a pose")
        if not bool(self.available) and bool(self.valid):
            raise ValueError("unavailable controller cannot be valid")
        trigger = _scalar(self.trigger, f"{self.side}.controller.trigger")
        grip = _scalar(self.grip, f"{self.side}.controller.grip")
        axis = _axis(self.axis, f"{self.side}.controller.axis")
        object.__setattr__(self, "pose", pose)
        object.__setattr__(self, "trigger", trigger)
        object.__setattr__(self, "grip", grip)
        object.__setattr__(self, "axis", axis)


@dataclass(frozen=True)
class XrTrackerState:
    serial_number: str
    pose: np.ndarray | None
    valid: bool
    side: str | None = None
    velocity: np.ndarray | None = None
    acceleration: np.ndarray | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.serial_number, str) or not self.serial_number.strip():
            raise ValueError("tracker serial_number is required")
        if self.side is not None and self.side not in SIDES:
            raise ValueError("tracker side must be left, right, or None")
        if not isinstance(self.valid, (bool, np.bool_)):
            raise ValueError("tracker valid must be boolean")
        pose = _pose(self.pose, "tracker.pose")
        if bool(self.valid) and pose is None:
            raise ValueError("valid tracker requires a pose")
        velocity = None if self.velocity is None else np.asarray(self.velocity, dtype=np.float64)
        acceleration = None if self.acceleration is None else np.asarray(self.acceleration, dtype=np.float64)
        for value, field in ((velocity, "tracker.velocity"), (acceleration, "tracker.acceleration")):
            if value is not None and (value.shape != (6,) or not np.isfinite(value).all()):
                raise ValueError(f"{field} must be a finite 6-vector")
        object.__setattr__(self, "pose", pose)
        object.__setattr__(self, "velocity", None if velocity is None else velocity.copy())
        object.__setattr__(self, "acceleration", None if acceleration is None else acceleration.copy())


@dataclass(frozen=True)
class XrFrame:
    sequence: int
    source_timestamp_ns: int | None
    received_timestamp_ns: int
    hmd_pose: np.ndarray | None
    controllers: Mapping[str, XrControllerState]
    trackers: tuple[XrTrackerState, ...]
    receiver_instance_id: str = "xr"
    connection_generation: int = 1

    def __post_init__(self) -> None:
        sequence = _finite_nonnegative_int(self.sequence, "sequence")
        source_timestamp = (None if self.source_timestamp_ns is None else
                            _finite_nonnegative_int(self.source_timestamp_ns, "source_timestamp_ns"))
        received_timestamp = _finite_nonnegative_int(self.received_timestamp_ns, "received_timestamp_ns")
        hmd_pose = _pose(self.hmd_pose, "hmd_pose")
        controllers = dict(self.controllers)
        if set(controllers) != set(SIDES) or any(
            not isinstance(value, XrControllerState) or value.side != side
            for side, value in controllers.items()
        ):
            raise ValueError("controllers must contain exactly valid left/right states")
        trackers = tuple(self.trackers)
        if any(not isinstance(value, XrTrackerState) for value in trackers):
            raise ValueError("trackers must contain XrTrackerState values")
        receiver_instance_id = _identity(self.receiver_instance_id, "receiver_instance_id")
        connection_generation = _finite_nonnegative_int(
            self.connection_generation, "connection_generation"
        )
        object.__setattr__(self, "sequence", sequence)
        object.__setattr__(self, "source_timestamp_ns", source_timestamp)
        object.__setattr__(self, "received_timestamp_ns", received_timestamp)
        object.__setattr__(self, "hmd_pose", hmd_pose)
        object.__setattr__(self, "controllers", controllers)
        object.__setattr__(self, "trackers", trackers)
        object.__setattr__(self, "receiver_instance_id", receiver_instance_id)
        object.__setattr__(self, "connection_generation", connection_generation)

    def controller(self, side: str) -> XrControllerState:
        if side not in SIDES:
            raise ValueError("side must be left or right")
        return self.controllers[side]

    def tracker(self, side: str) -> XrTrackerState:
        if side not in SIDES:
            raise ValueError("side must be left or right")
        matches = [item for item in self.trackers if item.side == side]
        if len(matches) != 1:
            raise ValueError(f"expected exactly one bound {side} tracker, got {len(matches)}")
        return matches[0]

    def to_dict(self) -> dict[str, Any]:
        def pose(value: np.ndarray | None):
            return None if value is None else value.tolist()

        return {
            "sequence": self.sequence,
            "source_timestamp_ns": self.source_timestamp_ns,
            "received_timestamp_ns": self.received_timestamp_ns,
            "receiver_instance_id": self.receiver_instance_id,
            "connection_generation": self.connection_generation,
            "hmd_pose": pose(self.hmd_pose),
            "controllers": {
                side: {
                    "pose": pose(value.pose),
                    "valid": bool(value.valid),
                    "available": bool(value.available),
                    "trigger": value.trigger,
                    "grip": value.grip,
                    "axis": value.axis.tolist(),
                }
                for side, value in self.controllers.items()
            },
            "trackers": [
                {
                    "serial_number": value.serial_number,
                    "side": value.side,
                    "pose": pose(value.pose),
                    "valid": bool(value.valid),
                    "velocity": None if value.velocity is None else value.velocity.tolist(),
                    "acceleration": None if value.acceleration is None else value.acceleration.tolist(),
                }
                for value in self.trackers
            ],
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "XrFrame":
        """Strictly decode the complete raw XR wire snapshot."""
        frame_keys = {
            "sequence", "source_timestamp_ns", "received_timestamp_ns",
            "receiver_instance_id", "hmd_pose", "controllers", "trackers",
        }
        if not isinstance(value, Mapping) or not frame_keys <= set(value) or set(value) - (
            frame_keys | {"connection_generation"}
        ):
            raise ValueError("invalid XR frame fields")
        controllers = value["controllers"]
        if not isinstance(controllers, Mapping) or set(controllers) != set(SIDES):
            raise ValueError("XR frame controllers must contain exactly left/right")
        controller_keys = {"pose", "valid", "available", "trigger", "grip", "axis"}
        decoded_controllers = {}
        for side in SIDES:
            item = controllers[side]
            if not isinstance(item, Mapping) or set(item) != controller_keys:
                raise ValueError(f"invalid {side} controller fields")
            decoded_controllers[side] = XrControllerState(
                side=side,
                pose=item["pose"],
                valid=item["valid"],
                available=item["available"],
                trigger=item["trigger"],
                grip=item["grip"],
                axis=item["axis"],
            )
        trackers = value["trackers"]
        if not isinstance(trackers, (list, tuple)):
            raise ValueError("XR frame trackers must be a list")
        tracker_keys = {
            "serial_number", "side", "pose", "valid", "velocity", "acceleration",
        }
        decoded_trackers = []
        for index, item in enumerate(trackers):
            if not isinstance(item, Mapping) or set(item) != tracker_keys:
                raise ValueError(f"invalid tracker[{index}] fields")
            decoded_trackers.append(XrTrackerState(
                serial_number=item["serial_number"],
                side=item["side"],
                pose=item["pose"],
                valid=item["valid"],
                velocity=item["velocity"],
                acceleration=item["acceleration"],
            ))
        return cls(
            sequence=value["sequence"],
            source_timestamp_ns=value["source_timestamp_ns"],
            received_timestamp_ns=value["received_timestamp_ns"],
            receiver_instance_id=value["receiver_instance_id"],
            connection_generation=value.get("connection_generation", 1),
            hmd_pose=value["hmd_pose"],
            controllers=decoded_controllers,
            trackers=tuple(decoded_trackers),
        )


@dataclass(frozen=True)
class XrButtonBinding:
    """A threshold binding over one SDK controller analog control."""

    side: str
    control: str
    threshold: float = 0.8

    def __post_init__(self) -> None:
        if self.side not in SIDES:
            raise ValueError("button binding side must be left or right")
        if self.control not in {"trigger", "grip", "axis_x_positive", "axis_x_negative",
                                "axis_y_positive", "axis_y_negative"}:
            raise ValueError("unsupported XR controller control")
        threshold = _scalar(self.threshold, "button binding threshold")
        if threshold <= 0.0:
            raise ValueError("button binding threshold must be positive")
        object.__setattr__(self, "threshold", threshold)

    def value(self, frame: XrFrame) -> float:
        controller = frame.controller(self.side)
        if self.control == "trigger":
            return controller.trigger
        if self.control == "grip":
            return controller.grip
        index = 0 if self.control.startswith("axis_x") else 1
        value = float(controller.axis[index])
        return value if self.control.endswith("positive") else -value

    def pressed(self, frame: XrFrame) -> bool:
        controller = frame.controller(self.side)
        return bool(controller.available and self.value(frame) >= self.threshold)


@dataclass(frozen=True)
class XrBindingConfig:
    """Explicitly binds robot arm roles to controllers or tracker serials."""

    arm_input: str
    tracker_serials: Mapping[str, str] | None = None
    controller_sides: Mapping[str, str] | None = None
    elbow_tracker_serials: Mapping[str, str] | None = None

    def __post_init__(self) -> None:
        if self.arm_input not in XR_ARM_INPUTS:
            raise ValueError("arm_input must be xr_controller or xr_tracker")
        controller_sides = dict(self.controller_sides or {side: side for side in SIDES})
        if set(controller_sides) != set(SIDES) or set(controller_sides.values()) != set(SIDES):
            raise ValueError("controller_sides must be a one-to-one left/right mapping")
        for side, controller_side in controller_sides.items():
            if side not in SIDES or controller_side not in SIDES:
                raise ValueError("controller_sides must contain left/right values")
        tracker_serials = dict(self.tracker_serials or {})
        if set(tracker_serials) - set(SIDES):
            raise ValueError("tracker_serials must contain only left/right")
        if tracker_serials:
            if any(not isinstance(value, str) or not value.strip()
                   for value in tracker_serials.values()):
                raise ValueError("tracker_serials must contain non-empty serials")
            if len(set(tracker_serials.values())) != len(tracker_serials):
                raise ValueError("tracker_serials must contain distinct serial bindings")
        if self.arm_input == "xr_tracker":
            if set(tracker_serials) != set(SIDES):
                raise ValueError("xr_tracker requires left/right tracker_serials")
        elbow_tracker_serials = dict(self.elbow_tracker_serials or {})
        if set(elbow_tracker_serials) - set(SIDES):
            raise ValueError("elbow_tracker_serials must contain only left/right")
        if any(not isinstance(value, str) or not value.strip()
               for value in elbow_tracker_serials.values()):
            raise ValueError("elbow_tracker_serials must contain non-empty serials")
        if len(set(elbow_tracker_serials.values())) != len(elbow_tracker_serials):
            raise ValueError("elbow_tracker_serials must contain distinct serial bindings")
        if set(tracker_serials.values()) & set(elbow_tracker_serials.values()):
            raise ValueError("wrist and elbow tracker bindings must be distinct")
        object.__setattr__(self, "controller_sides", controller_sides)
        object.__setattr__(self, "tracker_serials", tracker_serials)
        object.__setattr__(self, "elbow_tracker_serials", elbow_tracker_serials)

    def pose_for_arm(self, frame: XrFrame, side: str) -> np.ndarray | None:
        if not isinstance(frame, XrFrame) or side not in SIDES:
            raise ValueError("frame and side are required")
        if self.arm_input == "xr_controller":
            controller = frame.controller(self.controller_sides[side])
            return controller.pose.copy() if controller.valid and controller.available else None
        configured = self.tracker_serials[side]
        matches = [item for item in frame.trackers if _serial_matches(item.serial_number, configured)]
        if len(matches) > 1:
            raise ValueError(f"tracker binding {side} matches multiple serials")
        if not matches or not matches[0].valid:
            return None
        return matches[0].pose.copy() if matches[0].pose is not None else None

    def elbow_pose_for_arm(self, frame: XrFrame, side: str) -> np.ndarray | None:
        """Return the optionally bound forearm tracker pose for one arm.

        The reference PICO route binds a wrist and a forearm tracker per side.
        Keep this binding independent from ``tracker_serials`` so controller
        input can still be selected for the arm target without losing the
        forearm signal used for the elbow-direction hint.
        """
        if not isinstance(frame, XrFrame) or side not in SIDES:
            raise ValueError("frame and side are required")
        configured = self.elbow_tracker_serials.get(side)
        if configured is None:
            return None
        matches = [item for item in frame.trackers
                   if _serial_matches(item.serial_number, configured)]
        if len(matches) > 1:
            raise ValueError(f"elbow tracker binding {side} matches multiple serials")
        if not matches or not matches[0].valid:
            return None
        return matches[0].pose.copy() if matches[0].pose is not None else None

    def bound_tracker_side(self, serial_number: str) -> str | None:
        matches = [side for side, configured in self.tracker_serials.items()
                   if _serial_matches(serial_number, configured)]
        if len(matches) > 1:
            raise ValueError("tracker serial matches multiple arm sides")
        return matches[0] if matches else None


class XRoboToolkitClient:
    """Small lazy wrapper matching the reference XRoboToolkit Pybind API."""

    def __init__(self, host: str = "127.0.0.1", port: int = 60061, *, sdk_module: Any = None):
        self.host = host
        self.port = int(port)
        self._sdk = sdk_module
        self._connected = False

    def _load_sdk(self) -> Any:
        if self._sdk is None:
            # Native XRoboToolkit bindings are deployment artifacts.  The
            # launcher may inject their package parent for this process only;
            # no checkout path is embedded in the repository.
            injected = os.environ.get("TIANJI_XR_SDK_PYTHONPATH", "")
            for entry in reversed(tuple(item for item in injected.split(os.pathsep) if item)):
                if entry not in sys.path:
                    sys.path.insert(0, entry)
            try:
                self._sdk = importlib.import_module("xrobotoolkit_sdk")
            except ImportError as exc:
                raise RuntimeError(
                    "xrobotoolkit_sdk is unavailable; install XRoboToolkit PC-Service Pybind "
                    "and start its PC-Service before selecting the XR route"
                ) from exc
        return self._sdk

    def init(self) -> bool:
        # A missing native binding is a deployment/configuration error, not a
        # transient PC-Service disconnect.  Let the explicit error reach the
        # managed observation process so it can terminate with actionable
        # guidance instead of waiting forever.  Failures from the already
        # loaded SDK remain reconnectable below.
        sdk = self._load_sdk()
        try:
            result = sdk.init()
        except Exception:
            self._connected = False
            return False
        self._connected = bool(True if result is None else result)
        return self._connected

    def close(self) -> None:
        sdk = self._sdk
        try:
            close = getattr(sdk, "close", None) if self._connected else None
            if callable(close):
                close()
        except Exception:
            # Closing is a best-effort cleanup path.  The source still has to
            # become locally disconnected so a later initialize() can try a
            # fresh SDK session instead of reusing stale state.
            pass
        finally:
            self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def _call(self, name: str, default: Any) -> Any:
        if not self._connected:
            return default
        try:
            return getattr(self._load_sdk(), name)()
        except Exception:
            return default

    def get_headset_pose(self): return self._call("get_headset_pose", None)
    def get_left_controller_pose(self): return self._call("get_left_controller_pose", None)
    def get_right_controller_pose(self): return self._call("get_right_controller_pose", None)
    def num_motion_data_available(self): return self._call("num_motion_data_available", 0)
    def get_motion_tracker_pose(self): return self._call("get_motion_tracker_pose", None)
    def get_motion_tracker_velocity(self): return self._call("get_motion_tracker_velocity", None)
    def get_motion_tracker_acceleration(self): return self._call("get_motion_tracker_acceleration", None)
    def get_motion_tracker_serial_numbers(self): return self._call("get_motion_tracker_serial_numbers", [])
    def get_left_trigger(self): return self._call("get_left_trigger", 0.0)
    def get_right_trigger(self): return self._call("get_right_trigger", 0.0)
    def get_left_grip(self): return self._call("get_left_grip", 0.0)
    def get_right_grip(self): return self._call("get_right_grip", 0.0)
    def get_left_axis(self): return self._call("get_left_axis", [0.0, 0.0])
    def get_right_axis(self): return self._call("get_right_axis", [0.0, 0.0])
    def get_time_stamp_ns(self): return self._call("get_motion_timestamp_ns", 0)


class XrRoboToolkitSource:
    """Read one validated XR snapshot per call; native SDK remains optional."""

    def __init__(
        self,
        *,
        client: Any | None = None,
        binding: XrBindingConfig,
        receiver_instance_id: str,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not isinstance(binding, XrBindingConfig):
            raise TypeError("binding must be XrBindingConfig")
        self.client = client if client is not None else XRoboToolkitClient()
        if not callable(getattr(self.client, "init", None)) or not callable(getattr(self.client, "close", None)):
            raise TypeError("client must provide init and close")
        self.binding = binding
        self.receiver_instance_id = _identity(receiver_instance_id, "receiver_instance_id")
        if not callable(clock):
            raise TypeError("clock must be callable")
        self.clock = clock
        self.initialized = False
        self.connection_generation = 0
        self.sequence = 0
        self._last_received_timestamp_ns = -1
        self._last_source_timestamp_ns = -1

    def initialize(self) -> bool:
        was_initialized = self.initialized
        result = self.client.init()
        self.initialized = bool(result)
        if self.initialized and not was_initialized:
            self.connection_generation += 1
            self.sequence = 0
            self._last_received_timestamp_ns = -1
            self._last_source_timestamp_ns = -1
        return self.initialized

    def close(self) -> None:
        try:
            self.client.close()
        finally:
            self.initialized = False

    def _read_controller(self, side: str) -> XrControllerState:
        sdk_side = self.binding.controller_sides[side]
        pose_getter = getattr(self.client, f"get_{sdk_side}_controller_pose")
        trigger = getattr(self.client, f"get_{sdk_side}_trigger")()
        grip = getattr(self.client, f"get_{sdk_side}_grip")()
        axis = getattr(self.client, f"get_{sdk_side}_axis")()
        pose = _optional_pose(pose_getter(), f"{side}.controller.pose")
        available = pose is not None
        return XrControllerState(side=side, pose=pose, valid=available, available=available,
                                 trigger=trigger, grip=grip, axis=axis)

    def _read_trackers(self) -> tuple[XrTrackerState, ...]:
        count = self.client.num_motion_data_available()
        try:
            count = max(0, int(count))
        except (TypeError, ValueError):
            count = 0
        poses = _items(self.client.get_motion_tracker_pose())
        serials = _items(self.client.get_motion_tracker_serial_numbers())
        velocity_reader = getattr(self.client, "get_motion_tracker_velocity", lambda: None)
        acceleration_reader = getattr(self.client, "get_motion_tracker_acceleration", lambda: None)
        velocities = _items(velocity_reader())
        accelerations = _items(acceleration_reader())
        result: list[XrTrackerState] = []
        for index in range(min(count, len(poses))):
            serial = str(serials[index]) if index < len(serials) else f"tracker_{index}"
            pose = _optional_pose(poses[index], f"tracker[{index}].pose")
            velocity = velocities[index] if index < len(velocities) else None
            acceleration = accelerations[index] if index < len(accelerations) else None
            side = self.binding.bound_tracker_side(serial)
            result.append(XrTrackerState(serial, pose, pose is not None, side, velocity, acceleration))
        return tuple(result)

    def read_frame(self) -> XrFrame:
        if not self.initialized:
            raise RuntimeError("XR source is not initialized")
        received = _finite_nonnegative_int(self.clock(), "received_timestamp_ns")
        if received <= self._last_received_timestamp_ns:
            raise ValueError("XR receive clock rolled back or did not advance")
        raw_source_timestamp = self.client.get_time_stamp_ns()
        source_timestamp = None
        try:
            candidate = _finite_nonnegative_int(raw_source_timestamp, "source_timestamp_ns")
            # The reference wrapper returns 0 when the SDK timestamp is not
            # available.  It is a sentinel, not a valid monotonic sample.
            if candidate > 0 and candidate > self._last_source_timestamp_ns:
                source_timestamp = candidate
                self._last_source_timestamp_ns = candidate
        except ValueError:
            source_timestamp = None
        controllers = {side: self._read_controller(side) for side in SIDES}
        frame = XrFrame(
            sequence=self.sequence,
            source_timestamp_ns=source_timestamp,
            received_timestamp_ns=received,
            hmd_pose=_optional_pose(self.client.get_headset_pose(), "hmd_pose"),
            controllers=controllers,
            trackers=self._read_trackers(),
            receiver_instance_id=self.receiver_instance_id,
            connection_generation=self.connection_generation,
        )
        self.sequence += 1
        self._last_received_timestamp_ns = received
        return frame


__all__ = [
    "SIDES",
    "XR_ARM_INPUTS",
    "XrBindingConfig",
    "XrButtonBinding",
    "XrControllerState",
    "XrFrame",
    "XrRoboToolkitClient",
    "XrRoboToolkitSource",
    "XrTrackerState",
    "validate_xr_sdk_module",
]
