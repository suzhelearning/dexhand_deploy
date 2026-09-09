"""Pure coordinate transforms preserved for legacy mocap/replay inputs."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from .config_loader import get_config


def _side(side: str) -> str:
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    return side


def transform_world_to_chest(vector_world, side: str) -> np.ndarray:
    value = np.asarray(vector_world, dtype=np.float64)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("vector_world must be a finite 3-vector")
    _side(side)
    x, y, z = value
    return np.array([x, -z, y] if side == "left" else [x, z, -y])


def transform_chest_to_world(vector_chest, side: str) -> np.ndarray:
    value = np.asarray(vector_chest, dtype=np.float64)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("vector_chest must be a finite 3-vector")
    _side(side)
    x, y, z = value
    return np.array([x, z, -y] if side == "left" else [x, -z, y])


def get_world_to_chest_rotation(side: str) -> np.ndarray:
    return get_config(use_ros=False).get_world_to_chest_rotation(_side(side))


def get_chest_to_world_rotation(side: str) -> np.ndarray:
    return get_world_to_chest_rotation(side).T


def get_tf_quaternion(side: str) -> np.ndarray:
    quat = get_config(use_ros=False).world_to_chest_quat[_side(side)]
    return np.array([-quat[0], -quat[1], -quat[2], quat[3]], dtype=np.float64)


def get_pico_to_robot() -> np.ndarray:
    return get_config(use_ros=False).pico_to_robot.copy()


def apply_world_rotation_to_chest_pose(base_rot_chest, rotation_delta_world: R, side: str) -> np.ndarray:
    base = np.asarray(base_rot_chest, dtype=np.float64)
    if base.shape != (3, 3) or not np.isfinite(base).all():
        raise ValueError("base_rot_chest must be a finite 3x3 matrix")
    if not isinstance(rotation_delta_world, R):
        raise TypeError("rotation_delta_world must be scipy Rotation")
    world_to_chest = get_world_to_chest_rotation(_side(side))
    chest_to_world = world_to_chest.T
    return world_to_chest @ rotation_delta_world.as_matrix() @ chest_to_world @ base


def transform_pico_rotation_to_world(delta_rot_pico: R, pico_to_robot) -> R:
    if not isinstance(delta_rot_pico, R):
        raise TypeError("delta_rot_pico must be scipy Rotation")
    matrix = np.asarray(pico_to_robot, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("pico_to_robot must be a finite 3x3 matrix")
    rotvec = delta_rot_pico.as_rotvec()
    angle = float(np.linalg.norm(rotvec))
    if angle < 1.0e-10:
        return R.identity()
    return R.from_rotvec(matrix @ (rotvec / angle) * angle)


def get_direction_vector_world(mode: str) -> np.ndarray:
    return {
        "forward": np.array([1.0, 0.0, 0.0]),
        "back": np.array([-1.0, 0.0, 0.0]),
        "left": np.array([0.0, 1.0, 0.0]),
        "right": np.array([0.0, -1.0, 0.0]),
        "up": np.array([0.0, 0.0, 1.0]),
        "down": np.array([0.0, 0.0, -1.0]),
    }.get(mode, np.zeros(3))


def get_rotation_axis_world(mode: str) -> np.ndarray:
    return {
        "rotate_x": np.array([1.0, 0.0, 0.0]),
        "rotate_y": np.array([0.0, 1.0, 0.0]),
        "rotate_z": np.array([0.0, 0.0, 1.0]),
    }.get(mode, np.zeros(3))


def elbow_direction_from_angles(pitch_deg: float, yaw_deg: float, side: str) -> np.ndarray:
    _side(side)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)
    x = np.sin(pitch)
    anti_gravity = np.cos(pitch) * np.cos(yaw)
    outward = np.cos(pitch) * np.sin(yaw)
    direction = np.array([x, -anti_gravity if side == "left" else anti_gravity, -outward])
    norm = np.linalg.norm(direction)
    return direction / norm if norm > 1.0e-9 else direction
