from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from pico_body_tianji.controller_frame import ControllerFrame
from pico_body_tianji.controller_only.controller_only_mapper import (
    ControllerOnlyTeleopMapper,
)
from pico_body_tianji.controller_only.mocap_h5 import load_mocap_h5
from pico_body_tianji.controller_only.mocap_step_h5 import (
    generate_step_h5,
    _ramp_profile,
)
from pico_body_tianji.controller_only.target_conditioner import (
    TargetConditioningSettings,
)
from tianji_world_output.config_loader import TianjiConfig
from tianji_world_output.transform_utils import (
    transform_world_to_chest,
)

_REFERENCE_POSE = np.array([0.10, 0.20, -0.10, 0.0, 0.0, 0.0, 1.0])


def _mapper_with_gain(gain: float) -> ControllerOnlyTeleopMapper:
    config = TianjiConfig.load()
    return ControllerOnlyTeleopMapper(
        config,
        rate=60.0,
        min_cutoff=1.2,
        beta=0.45,
        conditioning_settings=TargetConditioningSettings(
            rate_hz=60.0,
            translation_gain=np.full(3, gain),
            rotation_gain=1.0,
            # 隔离尺度因子：工作空间与限速放宽，不介入 50mm 渐变台阶。
            workspace_relative_radii_m=np.full(3, 10.0),
            workspace_soft_zone_ratio=0.99,
            maximum_linear_speed_m_s=100.0,
            maximum_angular_speed_rad_s=100.0,
            maximum_linear_acceleration_m_s2=10000.0,
            maximum_angular_acceleration_rad_s2=10000.0,
        ),
        default_zsp_directions={
            side: config.get_default_zsp_direction(side)
            for side in ("left", "right")
        },
    )


def _drive_x_step(
    mapper: ControllerOnlyTeleopMapper, mm: float, frames_per_segment: int = 60
) -> dict[str, np.ndarray]:
    """驱动 +x mm 台阶（ramp 1s + hold 1s @60Hz），返回两侧最终目标位姿。"""
    mapper.initialize(
        ControllerFrame.from_poses(_REFERENCE_POSE, _REFERENCE_POSE)
    )
    final = {}
    for _ in range(frames_per_segment * 2):
        x = _REFERENCE_POSE[0] + (mm / 1000.0) * min(
            1.0, _ / frames_per_segment
        )
        pose = _REFERENCE_POSE.copy()
        pose[0] = x
        targets = mapper.map_frame(
            ControllerFrame.from_poses(pose, pose)
        )
        final["left"] = targets.left_pose.copy()
        final["right"] = targets.right_pose.copy()
    return final


class MocapStepH5GeneratorTest(unittest.TestCase):
    def test_generated_h5_loads_via_mocap_h5_loader(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "step50mm_x.h5"
            generate_step_h5(output, axis="x", mm=50.0)
            recording = load_mocap_h5(output)
            # 1s ramp + 1.5s hold + 1s return @60Hz
            self.assertEqual(recording.frame_count, 210)
            self.assertEqual(recording.take_id, 0)
            self.assertAlmostEqual(recording.output_hz, 60.0)
            for side in ("left", "right"):
                self.assertTrue(recording.hands[side].valid.all())
                # 终点在 hold 段：+x 50mm
                self.assertAlmostEqual(
                    recording.hands[side].wrist[120, 0],
                    0.10 + 0.05,
                    places=4,
                )
                self.assertAlmostEqual(
                    recording.hands[side].wrist[-1, 0], 0.10, places=4
                )

    def test_ramp_profile_ramp_hold_return(self) -> None:
        amplitude = _ramp_profile(210, 60, 90)
        self.assertAlmostEqual(amplitude[0], 0.0)
        self.assertAlmostEqual(amplitude[59], 1.0)
        self.assertTrue((amplitude[60:150] == 1.0).all())
        self.assertAlmostEqual(amplitude[-1], 0.0)

    def test_neg_direction_moves_robot_forward_axis(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "forward.h5"
            generate_step_h5(
                output, axis="z", mm=50.0, direction="neg"
            )
            recording = load_mocap_h5(output)
            # 输入 −z 50mm（hold 段终点）：机器人 chest +x 方向。
            self.assertAlmostEqual(
                recording.hands["left"].wrist[120, 2],
                -0.10 - 0.05,
                places=4,
            )

    def test_generator_rejects_bad_args(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                generate_step_h5(Path(tmp) / "bad.h5", axis="w")
            with self.assertRaises(ValueError):
                generate_step_h5(Path(tmp) / "bad.h5", mm=-1.0)


class MocapOneToOneMappingTest(unittest.TestCase):
    """验收：命令位移 → 目标位移严格 1:1（translation_gain=1.0）。"""

    def test_plus_x_50mm_maps_to_exactly_50mm(self) -> None:
        config = TianjiConfig.load()
        mapper = _mapper_with_gain(1.0)
        final = _drive_x_step(mapper, 50.0)
        for side in ("left", "right"):
            displacement = final[side][:3] - config.init_pos[side]
            distance = float(np.linalg.norm(displacement))
            self.assertAlmostEqual(
                distance, 0.050, delta=0.001,
                msg=f"{side} 目标位移 {distance*1000:.1f}mm ≠ 50mm",
            )
            # 方向 = pico_to_robot @ +x，再经 world→chest 轴映射。
            expected_direction = transform_world_to_chest(
                config.pico_to_robot @ np.array([1.0, 0.0, 0.0]), side
            )
            expected_direction /= np.linalg.norm(expected_direction)
            actual_direction = displacement / distance
            self.assertGreater(
                float(actual_direction @ expected_direction), 0.999,
                msg=f"{side} 目标位移方向偏离预期",
            )

    def test_gain_0_9_scales_to_45mm_regression(self) -> None:
        """回归：0.90 增益下 50mm 命令只产生 45mm，证明 gain 是尺度因子。"""
        config = TianjiConfig.load()
        mapper = _mapper_with_gain(0.90)
        final = _drive_x_step(mapper, 50.0)
        for side in ("left", "right"):
            distance = float(
                np.linalg.norm(final[side][:3] - config.init_pos[side])
            )
            self.assertAlmostEqual(
                distance, 0.045, delta=0.001,
                msg=f"{side} 目标位移 {distance*1000:.1f}mm ≠ 45mm",
            )


if __name__ == "__main__":
    unittest.main()
