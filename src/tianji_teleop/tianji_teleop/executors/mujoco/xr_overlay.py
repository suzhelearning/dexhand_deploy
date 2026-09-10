"""Passive MuJoCo visualization for raw XRoboToolkit frames.

The XR/Manus route uses the XR frame for arm input and the Manus callback for
hand input.  This overlay renders only the raw XR frame, in a translated
diagnostic workspace, so it cannot change targets, IK state, or robot qpos.
"""
from __future__ import annotations

from threading import Lock
from typing import Any, Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from ...hand_tracking.xr_input import XrFrame


DISPLAY_OFFSET_M = np.array([0.0, 1.2, 0.8], dtype=np.float64)
STALE_NS = 500_000_000


class XrRawOverlay:
    """Validate and render one ordered raw XR stream without side effects."""

    def __init__(self, router_zid: str):
        if not isinstance(router_zid, str) or not router_zid.strip():
            raise ValueError("XR overlay router identity is required")
        self.router_zid = router_zid
        self._lock = Lock()
        self._frame: XrFrame | None = None
        self._error: str | None = None
        self._count = 0

    def ingest(self, payload: Mapping[str, Any], now_ns: int) -> bool:
        try:
            if not isinstance(payload, Mapping):
                raise ValueError("raw XR payload must be an object")
            value = dict(payload)
            if value.pop("router_zid", None) != self.router_zid:
                raise ValueError("raw XR router mismatch")
            frame = XrFrame.from_dict(value)
            age = int(now_ns) - frame.received_timestamp_ns
            if not 0 <= age <= STALE_NS:
                raise ValueError("raw XR timestamp is stale or from the future")
            with self._lock:
                previous = self._frame
                if previous is not None:
                    if frame.receiver_instance_id != previous.receiver_instance_id:
                        raise ValueError("raw XR receiver mismatch")
                    if frame.connection_generation < previous.connection_generation:
                        raise ValueError("raw XR connection generation rollback")
                    if (frame.connection_generation == previous.connection_generation and
                            frame.sequence <= previous.sequence):
                        raise ValueError("raw XR frame sequence rollback")
                self._frame = frame
                self._count += 1
                self._error = None
            return True
        except (TypeError, ValueError, KeyError) as exc:
            self.reject(str(exc))
            return False

    def reject(self, error: str) -> None:
        with self._lock:
            self._error = str(error)

    def snapshot(self, now_ns: int) -> tuple[XrFrame | None, dict[str, Any]]:
        with self._lock:
            frame = self._frame
            error = self._error
            count = self._count
        if frame is None:
            state = "waiting"
        else:
            age = int(now_ns) - frame.received_timestamp_ns
            state = "live" if 0 <= age <= STALE_NS else "stale"
        return (
            frame if state == "live" else None,
            {
                "state": state,
                "frames_received": count,
                "error": error,
                "sequence": None if frame is None else frame.sequence,
                "connection_generation": (
                    None if frame is None else frame.connection_generation
                ),
                "receiver_instance_id": None if frame is None else frame.receiver_instance_id,
                "tracker_count": 0 if frame is None else len(frame.trackers),
            },
        )

    @staticmethod
    def _pose_marker(
        markers: list[dict[str, Any]],
        value: np.ndarray,
        color: tuple[float, float, float, float],
        label: str,
    ) -> None:
        marker = {
            "label": label,
            "position": value[:3].copy() + DISPLAY_OFFSET_M,
            "color": color,
            "rotation": None,
        }
        if np.linalg.norm(value[3:]) >= 1.0e-12:
            marker["rotation"] = Rotation.from_quat(value[3:]).as_matrix()
        markers.append(marker)

    def geometry(self, now_ns: int) -> tuple[list[dict[str, Any]], list[tuple[np.ndarray, np.ndarray]]]:
        frame, diagnostics = self.snapshot(now_ns)
        markers: list[dict[str, Any]] = []
        bones: list[tuple[np.ndarray, np.ndarray]] = []
        label = "XR raw " + diagnostics["state"]
        if diagnostics["error"]:
            label += " / rejected frame"
        markers.append({
            "label": label,
            "position": DISPLAY_OFFSET_M + np.array([0.0, 0.0, 0.2]),
            "color": (0.8, 0.8, 0.8, 1.0),
            "rotation": None,
        })
        if frame is None:
            return markers, bones

        if frame.hmd_pose is not None:
            self._pose_marker(markers, frame.hmd_pose, (0.9, 0.9, 0.9, 1.0), "XR HMD")
        for side, color in (("left", (0.0, 0.8, 1.0, 1.0)), ("right", (1.0, 0.5, 0.0, 1.0))):
            controller = frame.controller(side)
            if controller.available and controller.valid and controller.pose is not None:
                self._pose_marker(markers, controller.pose, color, f"XR {side} controller")
        for tracker in frame.trackers:
            if tracker.valid and tracker.pose is not None:
                side = f" {tracker.side}" if tracker.side is not None else ""
                self._pose_marker(
                    markers,
                    tracker.pose,
                    (0.2, 1.0, 0.2, 1.0),
                    f"XR tracker {tracker.serial_number}{side}",
                )
        return markers, bones

    def append(self, scene: Any, mujoco_module: Any, now_ns: int) -> None:
        """Append diagnostics under the caller's viewer lock."""
        markers, bones = self.geometry(now_ns)
        capacity = min(int(scene.maxgeom), len(scene.geoms))
        identity = np.eye(3, dtype=np.float64).ravel()

        def sphere(position, color, label):
            if scene.ngeom >= capacity:
                return False
            geom = scene.geoms[scene.ngeom]
            mujoco_module.mjv_initGeom(
                geom,
                mujoco_module.mjtGeom.mjGEOM_SPHERE,
                [0.012, 0.012, 0.012],
                position,
                identity,
                color,
            )
            geom.label = label
            scene.ngeom += 1
            return True

        def capsule(start, end, color):
            if scene.ngeom >= capacity or np.linalg.norm(end - start) < 1.0e-8:
                return False
            geom = scene.geoms[scene.ngeom]
            mujoco_module.mjv_initGeom(
                geom,
                mujoco_module.mjtGeom.mjGEOM_CAPSULE,
                np.zeros(3),
                np.zeros(3),
                identity,
                color,
            )
            geom.label = ""
            mujoco_module.mjv_connector(
                geom, mujoco_module.mjtGeom.mjGEOM_CAPSULE, 0.002, start, end
            )
            scene.ngeom += 1
            return True

        for start, end in bones:
            if not capsule(start + DISPLAY_OFFSET_M, end + DISPLAY_OFFSET_M, (0.2, 0.8, 1.0, 0.8)):
                return
        for marker in markers:
            if not sphere(marker["position"], marker["color"], marker["label"]):
                return
            rotation = marker["rotation"]
            if rotation is None:
                continue
            for axis, color in enumerate(((1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0), (0.0, 0.3, 1.0, 1.0))):
                if not capsule(
                    marker["position"],
                    marker["position"] + 0.08 * rotation[:, axis],
                    color,
                ):
                    return


__all__ = ["DISPLAY_OFFSET_M", "STALE_NS", "XrRawOverlay"]
