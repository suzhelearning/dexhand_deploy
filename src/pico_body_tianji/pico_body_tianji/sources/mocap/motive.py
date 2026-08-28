"""Strict Motive rigid-body frames shared by H5 and diagnostics only."""
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
    names: dict[int, str]

    def rigid_pose(self, rigid_id: int) -> np.ndarray | None:
        for body in self.rigid_bodies:
            if body.rigid_id == rigid_id:
                return None if body.pose is None else body.pose.copy()
        return None

    def rigid_name(self, rigid_id: int) -> str | None:
        return self.names.get(rigid_id)


class MotiveFrameSource:
    """Validate raw ``mocap/hands/frame`` payloads without producing targets."""

    @staticmethod
    def parse_names(value: Any) -> dict[int, str]:
        """Parse the top-level ``names`` mapping with canonical decimal IDs."""
        if not isinstance(value, Mapping):
            raise ValueError("Motive names must be an object")
        names_value = value.get("names") if "names" in value else value
        if not isinstance(names_value, Mapping):
            raise ValueError("Motive names must be a top-level object")
        result: dict[int, str] = {}
        seen_names: set[str] = set()
        for raw_id, name in names_value.items():
            if isinstance(raw_id, bool):
                raise ValueError("Motive rigid body name id must be canonical")
            if isinstance(raw_id, int):
                rigid_id = raw_id
            elif isinstance(raw_id, str) and raw_id.isdecimal() and str(int(raw_id)) == raw_id:
                rigid_id = int(raw_id)
            else:
                raise ValueError("Motive rigid body name id must be canonical decimal")
            if rigid_id <= 0 or rigid_id in result:
                raise ValueError("Motive rigid body names contain duplicate id")
            if not isinstance(name, str) or not name or name in seen_names:
                raise ValueError("Motive rigid body names contain duplicate/invalid name")
            result[rigid_id] = name
            seen_names.add(name)
        return result

    def parse(self, payload: Mapping[str, Any]) -> MotiveFrame:
        if not isinstance(payload, Mapping):
            raise ValueError("Motive frame must be an object")
        allowed = {"frame_number", "rigid_bodies", "names"}
        extra = set(payload) - allowed
        if extra:
            raise ValueError(f"Motive frame has unknown fields: {', '.join(sorted(extra))}")
        frame_number = payload.get("frame_number")
        if isinstance(frame_number, bool) or not isinstance(frame_number, int) or frame_number < 0:
            raise ValueError("Motive frame_number must be a non-negative integer")
        bodies = payload.get("rigid_bodies")
        if not isinstance(bodies, list):
            raise ValueError("Motive rigid_bodies must be a list")
        names = self.parse_names(payload["names"]) if "names" in payload else {}
        parsed: list[MotiveRigidBody] = []
        seen_ids: set[int] = set()
        for item in bodies:
            if not isinstance(item, Mapping):
                raise ValueError("Motive rigid body must be an object")
            if "name" in item:
                raise ValueError("Motive rigid body name must be in top-level names")
            rigid_id = item.get("id")
            if isinstance(rigid_id, bool) or not isinstance(rigid_id, int) or rigid_id <= 0:
                raise ValueError("Motive rigid body id must be a positive integer")
            if rigid_id in seen_ids:
                raise ValueError("Motive rigid bodies contain duplicate id")
            seen_ids.add(rigid_id)
            valid = item.get("tracking_valid")
            if not isinstance(valid, bool):
                raise ValueError("Motive tracking_valid must be boolean")
            pose = None
            position = item.get("position")
            quaternion = item.get("quaternion_xyzw")
            if not valid:
                if position is not None or quaternion is not None:
                    raise ValueError("invalid Motive rigid body must use null pose")
            else:
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
                values[3:] /= norm
                pose = values
            parsed.append(MotiveRigidBody(rigid_id, valid, pose))
        return MotiveFrame(frame_number, tuple(parsed), names)


__all__ = ["MotiveFrame", "MotiveFrameSource", "MotiveRigidBody"]
