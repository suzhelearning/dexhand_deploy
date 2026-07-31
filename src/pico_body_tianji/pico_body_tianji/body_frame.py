from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .body_schema import PICO_BODY_JOINT_INDEX


_SPINE3 = PICO_BODY_JOINT_INDEX["spine3"]
_NECK = PICO_BODY_JOINT_INDEX["neck"]
_LEFT_SHOULDER = PICO_BODY_JOINT_INDEX["left_shoulder"]
_RIGHT_SHOULDER = PICO_BODY_JOINT_INDEX["right_shoulder"]
_CHEST_FRAME_EPSILON = 1e-8


def _normalized(vector: np.ndarray, *, label: str) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < _CHEST_FRAME_EPSILON:
        raise ValueError(
            f"Body torso joints cannot define chest {label} axis"
        )
    return vector / norm


def _world_from_chest_basis(joints: np.ndarray) -> np.ndarray:
    """由实时上躯干关键点构造有解剖意义的正交胸廓坐标系。"""
    # XRoboToolkit/OpenXR 的 +X 指向用户右侧，因此左右肩连线定义
    # 胸廓 +X；SPINE3 到颈部定义胸廓 +Y。正交化后 +Z 保持右手系。
    chest_x = _normalized(
        joints[_RIGHT_SHOULDER, :3] - joints[_LEFT_SHOULDER, :3],
        label="lateral",
    )
    spine_up = joints[_NECK, :3] - joints[_SPINE3, :3]
    chest_y = spine_up - np.dot(spine_up, chest_x) * chest_x
    chest_y = _normalized(chest_y, label="vertical")
    chest_z = _normalized(np.cross(chest_x, chest_y), label="depth")
    chest_y = np.cross(chest_z, chest_x)
    return np.column_stack((chest_x, chest_y, chest_z))


@dataclass(frozen=True)
class BodyFrame:
    """一帧 PICO Full Body 的 24×7 关节位姿。"""

    joints: np.ndarray

    @classmethod
    def from_joints(cls, joints) -> "BodyFrame":
        array = np.asarray(joints, dtype=np.float64)
        if array.shape != (24, 7):
            raise ValueError(
                f"Body joint data must have shape 24x7, got {array.shape}"
            )
        if not np.isfinite(array).all():
            raise ValueError("Body joint data must contain only finite values")
        arm_quaternions = array[16:22, 3:7]
        norms = np.linalg.norm(arm_quaternions, axis=1)
        if np.any((norms < 0.95) | (norms > 1.05)):
            raise ValueError("Body arm joint quaternion must have unit length")
        _world_from_chest_basis(array)
        return cls(array)

    def virtual_trackers(self) -> dict[str, np.ndarray]:
        """将实时胸廓下的肩、肘、腕转换成四个虚拟 Tracker。"""
        chest_from_world = Rotation.from_matrix(
            _world_from_chest_basis(self.joints).T
        )
        roles = {}
        for side, shoulder_idx, elbow_idx, wrist_idx in (
            (
                "left",
                PICO_BODY_JOINT_INDEX["left_shoulder"],
                PICO_BODY_JOINT_INDEX["left_elbow"],
                PICO_BODY_JOINT_INDEX["left_wrist"],
            ),
            (
                "right",
                PICO_BODY_JOINT_INDEX["right_shoulder"],
                PICO_BODY_JOINT_INDEX["right_elbow"],
                PICO_BODY_JOINT_INDEX["right_wrist"],
            ),
        ):
            shoulder = self.joints[shoulder_idx, :3]
            roles[f"pico_{side}_wrist"] = self._pose_in_chest_frame(
                self.joints[wrist_idx],
                shoulder,
                chest_from_world,
            )
            roles[f"pico_{side}_arm"] = self._pose_in_chest_frame(
                self.joints[elbow_idx],
                shoulder,
                chest_from_world,
            )
        return roles

    def arm_keypoints(self, side: str) -> np.ndarray:
        """返回控制器坐标下、以肩为原点的肩/肘/腕真实几何。"""
        indices = {
            "left": (
                PICO_BODY_JOINT_INDEX["left_shoulder"],
                PICO_BODY_JOINT_INDEX["left_elbow"],
                PICO_BODY_JOINT_INDEX["left_wrist"],
            ),
            "right": (
                PICO_BODY_JOINT_INDEX["right_shoulder"],
                PICO_BODY_JOINT_INDEX["right_elbow"],
                PICO_BODY_JOINT_INDEX["right_wrist"],
            ),
        }
        if side not in indices:
            raise ValueError(f"Unknown arm side: {side}")

        shoulder_idx, elbow_idx, wrist_idx = indices[side]
        shoulder = self.joints[shoulder_idx, :3]
        world_points = np.stack(
            (
                np.zeros(3, dtype=np.float64),
                self.joints[elbow_idx, :3] - shoulder,
                self.joints[wrist_idx, :3] - shoulder,
            )
        )
        chest_from_world = Rotation.from_matrix(
            _world_from_chest_basis(self.joints).T
        )
        return chest_from_world.apply(world_points)

    def skeleton_keypoints(self) -> np.ndarray:
        """返回实时胸廓坐标下、以左右肩中心为原点的 24 个 SMPL 点。"""
        shoulder_center = self.shoulder_center()
        world_points = self.joints[:, :3] - shoulder_center
        chest_from_world = Rotation.from_matrix(
            _world_from_chest_basis(self.joints).T
        )
        return chest_from_world.apply(world_points)

    def pose_in_torso_frame(self, pose) -> np.ndarray:
        """把与 Body 同一世界系下的位姿表达为实时 SMPL 胸廓位姿。"""
        values = np.asarray(pose, dtype=np.float64)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError("World pose must be a finite 7-vector")
        quaternion_norm = float(np.linalg.norm(values[3:7]))
        if quaternion_norm < 0.95 or quaternion_norm > 1.05:
            raise ValueError("World pose quaternion must have unit length")

        chest_from_world = Rotation.from_matrix(
            _world_from_chest_basis(self.joints).T
        )
        converted = values.copy()
        converted[:3] = chest_from_world.apply(
            values[:3] - self.shoulder_center()
        )
        converted[3:7] = (
            chest_from_world
            * Rotation.from_quat(values[3:7] / quaternion_norm)
        ).as_quat()
        return converted

    def shoulder_center(self) -> np.ndarray:
        """返回当前 SMPL 左右肩中心在输入世界系中的位置。"""
        return 0.5 * (
            self.joints[_LEFT_SHOULDER, :3]
            + self.joints[_RIGHT_SHOULDER, :3]
        )

    def signature(self) -> bytes:
        """返回覆盖胸廓定义点及左右臂的稳定帧签名。"""
        return self.joints[
            [
                _SPINE3,
                _NECK,
                PICO_BODY_JOINT_INDEX["left_shoulder"],
                PICO_BODY_JOINT_INDEX["right_shoulder"],
                PICO_BODY_JOINT_INDEX["left_elbow"],
                PICO_BODY_JOINT_INDEX["right_elbow"],
                PICO_BODY_JOINT_INDEX["left_wrist"],
                PICO_BODY_JOINT_INDEX["right_wrist"],
            ]
        ].tobytes()

    @staticmethod
    def _pose_in_chest_frame(
        pose: np.ndarray,
        shoulder: np.ndarray,
        chest_from_world: Rotation,
    ) -> np.ndarray:
        """把世界系骨骼 Pose 表达到当前骨架胸廓坐标。"""
        converted = np.asarray(pose, dtype=np.float64).copy()
        converted[:3] = chest_from_world.apply(pose[:3] - shoulder)
        converted[3:7] = (
            chest_from_world * Rotation.from_quat(pose[3:7])
        ).as_quat()
        return converted
