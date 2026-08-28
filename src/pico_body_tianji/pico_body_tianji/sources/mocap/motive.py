"""Shared Motive rigid-body frame parser for H5 and diagnostics only."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np


@dataclass(frozen=True)
class MotiveRigidBody:
    rigid_id: int
    tracking_valid: bool
    pose: np.ndarray | None


@dataclass(frozen=True)
class MotiveFrame:
    frame_number: int
    rigid_bodies: tuple[MotiveRigidBody, ...]

    def rigid_pose(self, rigid_id: int) -> np.ndarray | None:
        for body in self.rigid_bodies:
            if body.rigid_id == rigid_id:
                return None if body.pose is None else body.pose.copy()
        return None


class MotiveFrameSource:
    """Validate raw ``mocap/hands/frame`` payloads without producing targets."""

    def parse(self, payload: Mapping[str, Any]) -> MotiveFrame:
        if not isinstance(payload, Mapping):
            raise ValueError("Motive frame must be an object")
        frame_number = payload.get("frame_number")
        if isinstance(frame_number, bool) or not isinstance(frame_number, int) or frame_number < 0:
            raise ValueError("Motive frame_number must be a non-negative integer")
        bodies = payload.get("rigid_bodies")
        if not isinstance(bodies, list):
            raise ValueError("Motive rigid_bodies must be a list")
        parsed: list[MotiveRigidBody] = []
        for item in bodies:
            if not isinstance(item, Mapping):
                raise ValueError("Motive rigid body must be an object")
            rigid_id = item.get("id")
            if isinstance(rigid_id, bool) or not isinstance(rigid_id, int) or rigid_id <= 0:
                raise ValueError("Motive rigid body id must be a positive integer")
            valid = item.get("tracking_valid")
            if not isinstance(valid, bool):
                raise ValueError("Motive tracking_valid must be boolean")
            pose = None
            if valid:
                position = item.get("position")
                quaternion = item.get("quaternion_xyzw")
                if (
                    not isinstance(position, (list, tuple))
                    or not isinstance(quaternion, (list, tuple))
                    or len(position) != 3
                    or len(quaternion) != 4
                ):
                    raise ValueError("Motive pose must contain 3 position and 4 quaternion values")
                values = np.asarray(list(position) + list(quaternion), dtype=np.float64)
                if values.shape != (7,) or not np.isfinite(values).all():
                    raise ValueError("valid Motive rigid body pose must be finite 7-vector")
                norm = float(np.linalg.norm(values[3:]))
                if not 0.999 <= norm <= 1.001:
                    raise ValueError("Motive quaternion must be normalized")
                pose = values
            parsed.append(MotiveRigidBody(rigid_id, valid, pose))
        return MotiveFrame(frame_number, tuple(parsed))


__all__ = ["MotiveFrame", "MotiveFrameSource", "MotiveRigidBody"]
