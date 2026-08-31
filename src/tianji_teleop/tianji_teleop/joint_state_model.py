from __future__ import annotations

from dataclasses import dataclass

import numpy as np


SIDES = ("left", "right")


def urdf_joint_names() -> list[str]:
    """Return movable joint names in the Marvin dual-arm URDF order."""
    return [
        *(f"Joint{index}_L" for index in range(1, 8)),
        *(f"Joint{index}_R" for index in range(1, 8)),
    ]


def extract_side_positions(
    side: str,
    names,
    positions,
) -> list[float]:
    """Normalize one official Marvin degree JointState into joint 1..7."""
    if side not in SIDES:
        raise ValueError(f"side must be left or right, got {side!r}")
    values = list(positions)
    if not names:
        if len(values) != 7:
            raise ValueError("unnamed Marvin joint state must have 7 values")
        return values

    source_names = list(names)
    if len(source_names) != len(values):
        raise ValueError("joint names and positions have different lengths")
    expected = [f"{side}_joint_{index}" for index in range(1, 8)]
    if len(set(source_names)) != len(source_names):
        raise ValueError("joint names contain duplicates")
    by_name = dict(zip(source_names, values))
    if set(by_name) != set(expected):
        raise ValueError(
            f"{side} feedback must contain exactly {expected!r}"
        )
    return [by_name[name] for name in expected]


@dataclass(frozen=True)
class DualArmJointState:
    names: list[str]
    positions_rad: list[float]


class MarvinJointStateAssembler:
    """Combine two Marvin SDK degree streams into one URDF radian state."""

    def __init__(self):
        self._joints_deg: dict[str, np.ndarray | None] = {
            side: None for side in SIDES
        }

    def update(
        self, side: str, joints_deg
    ) -> DualArmJointState | None:
        if side not in SIDES:
            raise ValueError(f"side must be left or right, got {side!r}")

        joints = np.asarray(joints_deg, dtype=np.float64)
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise ValueError(
                "Marvin joint state must contain seven finite degree values"
            )
        self._joints_deg[side] = joints.copy()

        if any(self._joints_deg[item] is None for item in SIDES):
            return None
        combined_deg = np.concatenate(
            [self._joints_deg["left"], self._joints_deg["right"]]
        )
        return DualArmJointState(
            names=urdf_joint_names(),
            positions_rad=np.deg2rad(combined_deg).tolist(),
        )
