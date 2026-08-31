"""Live Motive wrist/hammer poses for Regrind policy runtimes."""
from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any

import numpy as np

from ...protocol import topics
from ...zenoh_util import ZenohJsonSub
from .h5 import compose_pose
from .motive import MotiveFrame, MotiveFrameSource


HAMMER_RIGID_TO_OBJECT = np.asarray(
    [0.002, -0.005, 0.0, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476]
)
WRIST_RIGID_TO_MARKER = np.asarray(
    [0.001, -0.004, 0.002, -0.0086933284, 0.0871524241, 0.0007605677, 0.9961567661]
)
MARKER_TO_MOUNT = np.asarray(
    [0.004, 0.0, 0.0, 0.0, -0.7071067811865476, 0.0, 0.7071067811865476]
)
MOUNT_TO_WRIST = np.asarray(
    [0.003, 0.00025016, -0.0285, 0.0, 0.0, 0.0000081994999999, 0.9999999999663841]
)


@dataclass(frozen=True)
class RegrindMotiveSample:
    frame_number: int
    received_at: float
    wrist_xyzw: np.ndarray
    hammer_xyzw: np.ndarray


class RegrindMotiveTracker:
    def __init__(
        self,
        session: Any,
        *,
        wrist_name: str = "tianji_wrist",
        hammer_name: str = "hammer",
        rigid_to_wrist: np.ndarray | None = None,
        rigid_to_object: np.ndarray | None = None,
    ) -> None:
        self._parser = MotiveFrameSource()
        self._wrist_name = wrist_name
        self._hammer_name = hammer_name
        self._rigid_to_wrist = (
            compose_pose(compose_pose(WRIST_RIGID_TO_MARKER, MARKER_TO_MOUNT), MOUNT_TO_WRIST)
            if rigid_to_wrist is None else np.asarray(rigid_to_wrist, dtype=np.float64)
        )
        self._rigid_to_object = (
            HAMMER_RIGID_TO_OBJECT.copy()
            if rigid_to_object is None else np.asarray(rigid_to_object, dtype=np.float64)
        )
        self._lock = threading.Lock()
        self._names: dict[int, str] = {}
        self._frame: MotiveFrame | None = None
        self._received_at = 0.0
        self._error: str | None = None
        self._names_sub = ZenohJsonSub(session, topics.MOCAP_RIGID_BODY_NAMES, self._on_names)
        self._frame_sub = ZenohJsonSub(session, topics.MOCAP_HANDS_FRAME, self._on_frame)

    @property
    def error(self) -> str | None:
        with self._lock:
            return self._error

    def _on_names(self, payload: Any) -> None:
        try:
            names = self._parser.parse_names(payload)
            for wanted in (self._wrist_name, self._hammer_name):
                if list(names.values()).count(wanted) != 1:
                    raise ValueError(f"Motive must contain exactly one rigid body named {wanted!r}")
        except (TypeError, ValueError) as exc:
            with self._lock:
                self._error = str(exc)
            return
        with self._lock:
            self._names = names

    def _on_frame(self, payload: Any) -> None:
        try:
            frame = self._parser.parse(payload)
        except (TypeError, ValueError) as exc:
            with self._lock:
                self._error = str(exc)
            return
        with self._lock:
            self._frame = frame
            self._received_at = time.monotonic()

    def latest(self) -> RegrindMotiveSample | None:
        with self._lock:
            frame, names, received_at = self._frame, dict(self._names), self._received_at
        if frame is None:
            return None
        ids = {name: rigid_id for rigid_id, name in names.items()}
        if self._wrist_name not in ids or self._hammer_name not in ids:
            return None
        wrist_rigid = frame.rigid_pose(ids[self._wrist_name])
        hammer_rigid = frame.rigid_pose(ids[self._hammer_name])
        if wrist_rigid is None or hammer_rigid is None:
            return None
        return RegrindMotiveSample(
            frame.frame_number,
            received_at,
            compose_pose(wrist_rigid, self._rigid_to_wrist),
            compose_pose(hammer_rigid, self._rigid_to_object),
        )

    def close(self) -> None:
        self._names_sub.close()
        self._frame_sub.close()


__all__ = [
    "HAMMER_RIGID_TO_OBJECT",
    "MARKER_TO_MOUNT",
    "MOUNT_TO_WRIST",
    "RegrindMotiveSample",
    "RegrindMotiveTracker",
    "WRIST_RIGID_TO_MARKER",
]
