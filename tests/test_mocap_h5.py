from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from pico_body_tianji.controller_only.mocap_h5 import (
    HandPoseTrajectory,
    MocapRecording,
    align_pose_to_reference,
    apply_yaw_world,
    compose_pose,
    invert_pose,
    load_mocap_h5,
    synthetic_reference_pose,
)


def _write_v4_h5(
    path: Path,
    *,
    left_nan: bool = False,
    right_nan: bool = False,
    frames: int = 61,
    output_hz: float = 60.0,
    version: str = "4.0",
    schema_name: str = "compact-aligned-60hz-v1",
    with_external_link: bool = False,
) -> None:
    time_ns = np.arange(frames, dtype=np.int64) * int(1.0e9 / output_hz)
    with h5py.File(path, "w") as f:
        f.attrs["h5_version"] = version
        f.attrs["schema_name"] = "mocap-acquisition"
        f.attrs["schema_layout"] = schema_name
        f.attrs["output_hz"] = output_hz
        f.attrs["take_id"] = 3
        f.attrs["time_domain"] = "linux-clock-monotonic"
        f.create_dataset("time_ns", data=time_ns)
        f.create_dataset("valid", data=np.ones(frames, dtype=np.uint8))
        for side, nan_side in (
            ("left", left_nan),
            ("right", right_nan),
        ):
            group = f.create_group(f"hands/{side}")
            group.attrs["keypoint_count"] = 21
            group.attrs["source"] = "manus"
            if nan_side:
                position = np.full((frames, 3), np.nan, dtype=np.float32)
                quaternion = np.full(
                    (frames, 4), np.nan, dtype=np.float32
                )
                valid = np.zeros(frames, dtype=np.uint8)
                keypoints = np.full(
                    (frames, 21, 3), np.nan, dtype=np.float32
                )
            else:
                # 简单直线运动：x 从 0.1 到 0.3，姿态恒为 Identity。
                position = np.column_stack(
                    (
                        np.linspace(0.1, 0.3, frames, dtype=np.float32),
                        np.full(frames, 0.2, dtype=np.float32),
                        np.full(frames, -0.1, dtype=np.float32),
                    )
                )
                quaternion = np.tile(
                    np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
                    (frames, 1),
                )
                valid = np.ones(frames, dtype=np.uint8)
                offsets = np.zeros((21, 3), dtype=np.float32)
                offsets[:, 0] = np.arange(21, dtype=np.float32) * 0.001
                keypoints = position[:, None, :] + offsets[None, :, :]
            group.create_dataset("wrist_position", data=position)
            group.create_dataset(
                "wrist_quaternion_xyzw", data=quaternion
            )
            group.create_dataset("valid", data=valid)
            group.create_dataset("keypoints_world", data=keypoints)
        if with_external_link:
            f["objects"] = h5py.ExternalLink("outside.h5", "/")


class MocapH5LoaderTest(unittest.TestCase):
    def test_loads_valid_v4_recording(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "take.h5"
            _write_v4_h5(path)
            recording = load_mocap_h5(path)
            self.assertIsInstance(recording, MocapRecording)
            self.assertEqual(recording.frame_count, 61)
            self.assertEqual(recording.take_id, 3)
            self.assertAlmostEqual(recording.output_hz, 60.0)
            self.assertAlmostEqual(recording.duration_s, 1.0, places=2)
            for side in ("left", "right"):
                self.assertEqual(recording.hands[side].wrist.shape, (61, 7))
                self.assertEqual(
                    recording.hands[side].keypoints_world.shape,
                    (61, 21, 3),
                )
                np.testing.assert_allclose(
                    recording.hands[side].keypoints_world[:, 0],
                    recording.hands[side].wrist[:, :3],
                )
                self.assertTrue(recording.hands[side].valid.all())
                self.assertEqual(
                    recording.first_valid_index(side), 0
                )
            self.assertEqual(recording.reference_index(), 0)
            summary = recording.summary()
            self.assertEqual(summary["hands"]["left"]["valid_frames"], 61)

    def test_rejects_wrong_version(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "old.h5"
            _write_v4_h5(path, version="3.0")
            with self.assertRaisesRegex(ValueError, "h5_version"):
                load_mocap_h5(path)

    def test_rejects_external_link(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "linked.h5"
            _write_v4_h5(path, with_external_link=True)
            with self.assertRaisesRegex(ValueError, "ExternalLink"):
                load_mocap_h5(path)

    def test_missing_file_raises(self) -> None:
        with self.assertRaises(FileNotFoundError):
            load_mocap_h5("/nonexistent/take.h5")

    def test_nan_side_marks_all_frames_invalid(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "take.h5"
            _write_v4_h5(path, left_nan=True)
            recording = load_mocap_h5(path)
            self.assertFalse(recording.hands["left"].valid.any())
            self.assertTrue(recording.hands["right"].valid.all())
            self.assertIsNone(recording.first_valid_index("left"))
            self.assertEqual(recording.first_valid_index("right"), 0)
            self.assertEqual(recording.reference_index(), 0)

    def test_apply_yaw_world_rotates_position_and_orientation(self) -> None:
        poses = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
        rotated = apply_yaw_world(poses, 90.0)
        np.testing.assert_allclose(rotated[0, :3], [0.0, 1.0, 0.0],
                                   atol=1e-9)
        # 绕 +Z（Motive 竖直轴）旋转 90°：xyzw 序 [0, 0, sin45, cos45]。
        expected_quat = np.array(
            [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]
        )
        np.testing.assert_allclose(rotated[0, 3:], expected_quat, atol=1e-9)

    def test_apply_yaw_world_zero_is_identity(self) -> None:
        poses = np.array([[1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9]])
        result = apply_yaw_world(poses, 0.0)
        np.testing.assert_array_equal(result, poses)

    def test_apply_yaw_world_rejects_nonfinite(self) -> None:
        with self.assertRaises(ValueError):
            apply_yaw_world(np.zeros((1, 7)), float("nan"))



    def test_synthetic_reference_pose_is_unit_quaternion(self) -> None:
        pose = synthetic_reference_pose()
        self.assertEqual(pose.shape, (7,))
        self.assertAlmostEqual(np.linalg.norm(pose[3:]), 1.0)

    def test_hand_trajectory_interpolates_invalid_middle_frame(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "operator-selected-take.h5"
            _write_v4_h5(path, frames=3, output_hz=2.0)
            with h5py.File(path, "r+") as h5:
                h5["hands/right/valid"][1] = 0
            trajectory = HandPoseTrajectory(load_mocap_h5(path))
            self.assertEqual(trajectory.interpolated_frame_count, 1)
            sample = trajectory.sample(0.5)
            np.testing.assert_allclose(
                sample.pose[:3],
                [0.2, 0.2, -0.1],
                atol=1e-7,
            )
            np.testing.assert_allclose(
                sample.pose[3:],
                [0.0, 0.0, 0.0, 1.0],
                atol=1e-9,
            )
            self.assertEqual(sample.source_frame_index, 1)
            self.assertFalse(sample.complete)

    def test_hand_trajectory_clamps_to_first_and_last_valid_frames(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "another-take.h5"
            _write_v4_h5(path, frames=4, output_hz=2.0)
            with h5py.File(path, "r+") as h5:
                h5["hands/right/valid"][0] = 0
                h5["hands/right/valid"][3] = 0
            trajectory = HandPoseTrajectory(load_mocap_h5(path))
            self.assertEqual(trajectory.start_frame_index, 1)
            self.assertEqual(trajectory.end_frame_index, 2)
            np.testing.assert_allclose(
                trajectory.pose_at_frame(0),
                trajectory.pose_at_frame(1),
            )
            sample = trajectory.sample(99.0)
            np.testing.assert_allclose(
                sample.pose,
                trajectory.pose_at_frame(2),
            )
            self.assertTrue(sample.complete)

    def test_hand_trajectory_rejects_recording_without_selected_hand(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "right-missing.h5"
            _write_v4_h5(path, right_nan=True)
            with self.assertRaisesRegex(ValueError, "right 手腕没有有效位姿"):
                HandPoseTrajectory(load_mocap_h5(path))



class PoseTransformTest(unittest.TestCase):
    def test_compose_and_inverse_are_identity(self) -> None:
        pose = np.array(
            [1.0, -2.0, 0.5, 0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)]
        )
        identity = compose_pose(pose, invert_pose(pose))
        np.testing.assert_allclose(identity[:3], np.zeros(3), atol=1e-9)
        np.testing.assert_allclose(
            np.abs(identity[3:]), [0.0, 0.0, 0.0, 1.0], atol=1e-9
        )

    def test_wrist_frame_zero_alignment_preserves_relative_trajectory(
        self,
    ) -> None:
        source_ref = np.array(
            [1.0, 2.0, 3.0, 0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]
        )
        target_ref = np.array(
            [-0.5, 0.25, 1.2, np.sin(np.pi / 8), 0.0, 0.0, np.cos(np.pi / 8)]
        )
        source_delta = np.array(
            [0.1, -0.2, 0.3, 0.0, np.sin(np.pi / 12), 0.0, np.cos(np.pi / 12)]
        )
        sample = compose_pose(source_ref, source_delta)

        aligned_zero = align_pose_to_reference(
            source_ref, source_ref, target_ref
        )
        aligned_sample = align_pose_to_reference(
            sample, source_ref, target_ref
        )

        np.testing.assert_allclose(
            aligned_zero, target_ref, atol=1e-9
        )
        relative = compose_pose(
            invert_pose(target_ref), aligned_sample
        )
        np.testing.assert_allclose(relative, source_delta, atol=1e-9)

    def test_wrist_to_tcp_round_trip_preserves_endpoint(self) -> None:
        marker = np.array(
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
        )
        marker_to_wrist = np.array(
            [0.0325, 0.00025, 0.003, 0.0, -np.sqrt(0.5), 0.0, np.sqrt(0.5)]
        )
        tcp_to_wrist = np.array(
            [0.00025, 0.003, 0.0365, np.sqrt(0.5), np.sqrt(0.5), 0.0, 0.0]
        )
        wrist = compose_pose(marker, marker_to_wrist)
        virtual_tcp = compose_pose(wrist, invert_pose(tcp_to_wrist))

        reconstructed_wrist = compose_pose(
            virtual_tcp, tcp_to_wrist
        )

        np.testing.assert_allclose(
            reconstructed_wrist, wrist, atol=1e-9
        )


if __name__ == "__main__":
    unittest.main()
