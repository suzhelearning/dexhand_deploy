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
        try:
            frame_number = int(payload["frame_number"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("Motive frame_number must be an integer") from exc
        bodies = payload.get("rigid_bodies")
        if not isinstance(bodies, list):
            raise ValueError("Motive rigid_bodies must be a list")
        parsed: list[MotiveRigidBody] = []
        for item in bodies:
            if not isinstance(item, Mapping):
                raise ValueError("Motive rigid body must be an object")
            try:
                rigid_id = int(item["id"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("Motive rigid body id must be an integer") from exc
            valid = item.get("tracking_valid")
            if not isinstance(valid, bool):
                raise ValueError("Motive tracking_valid must be boolean")
            pose = None
            if valid:
                position = item.get("position")
                quaternion = item.get("quaternion_xyzw")
                values = np.asarray(
                    list(position) + list(quaternion), dtype=np.float64
                ) if isinstance(position, (list, tuple)) and isinstance(quaternion, (list, tuple)) else np.empty(0)
                if values.shape != (7,) or not np.isfinite(values).all():
                    raise ValueError("valid Motive rigid body pose must be finite 7-vector")
                norm = float(np.linalg.norm(values[3:]))
                if norm < 1.0e-8:
                    raise ValueError("Motive quaternion must be nonzero")
                values[3:] /= norm
                pose = values
            parsed.append(MotiveRigidBody(rigid_id, valid, pose))
        return MotiveFrame(frame_number, tuple(parsed))


__all__ = ["MotiveFrame", "MotiveFrameSource", "MotiveRigidBody"]
