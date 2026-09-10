from pathlib import Path
import unittest

import numpy as np

from tianji_teleop.hand_tracking.models import ArmInputObservation
from tianji_teleop.sources.common.pose_mapping import create_arm_pose_mapper


REFERENCE_CONFIG = Path(__file__).parents[1] / "src/tianji_teleop/tianji_teleop/hand_tracking/reference_xr/config/tianji_robot.yaml"


def _observation(side, pose, sequence=0, tracked_frame="wrist_tracker", elbow_pose=None):
    return ArmInputObservation(
        source="xr",
        side=side,
        tracked_frame=tracked_frame,
        reference_frame="xr_tracking",
        pose=np.asarray(pose, dtype=np.float64),
        valid=True,
        source_timestamp_ns=sequence + 1,
        received_timestamp_ns=sequence + 1,
        receiver_instance_id="xr-receiver",
        receiver_frame_sequence=sequence,
        mapping_version="xr_raw_v1",
        frame_association_id=f"xr:{sequence}",
        source_sequence=sequence,
        source_instance_id="xr-source",
        elbow_pose=elbow_pose,
    )


class XrIncrementalMappingTest(unittest.TestCase):
    def mapper(self, **overrides):
        value = dict(
            reference_config_path=str(REFERENCE_CONFIG),
            expected_reference_frame="xr_tracking",
            expected_tracked_frame="wrist_tracker",
            rate_hz=90.0,
            min_cutoff=1.0,
            beta=0.7,
            elbow_min_cutoff=0.3,
            dynamic_elbow_direction=True,
            tracked_to_wrist_pose={
                "left": [0, 0, 0, 0, 0, 0, 1],
                "right": [0, 0, 0, 0, 0, 0, 1],
            },
        )
        value.update(overrides)
        return create_arm_pose_mapper("xr_incremental", value)

    def test_initial_pose_is_the_reference_robot_home(self):
        mapper = self.mapper()
        initial = {
            "left": _observation("left", [0, 0, 0, 0, 0, 0, 1]),
            "right": _observation("right", [0, 0, 0, 0, 0, 0, 1]),
        }
        mapper.initialize(initial)
        result = mapper.map(initial["left"])
        np.testing.assert_allclose(result.pose[:3], [0.5733, 0.2237, 0.2762], atol=1e-10)
        self.assertTrue(result.valid)
        self.assertEqual(result.backend, "xr_incremental")

    def test_translation_and_rotation_use_reference_six_axis_semantics(self):
        mapper = self.mapper()
        initial = _observation("left", [0, 0, 0, 0, 0, 0, 1])
        mapper.initialize({"left": initial})
        current = _observation("left", [0.1, 0.2, 0.3, 0, 0, 0, 1], sequence=1)
        result = mapper.map(current)
        # Reference PICO->robot delta is [-.3, -.1, .2], then left chest is
        # [x, -z, y].  This is the exact TJ_arm_control incremental basis.
        np.testing.assert_allclose(
            result.pose[:3],
            np.asarray([0.5733, 0.2237, 0.2762]) + [-0.3, -0.2, -0.1],
            atol=1e-8,
        )

        rotated = _observation("left", [0.1, 0.2, 0.3, 0, 0, np.sin(0.2), np.cos(0.2)], sequence=2)
        rotated_result = mapper.map(rotated)
        self.assertFalse(np.allclose(rotated_result.pose[3:], result.pose[3:]))
        self.assertAlmostEqual(float(np.linalg.norm(rotated_result.pose[3:])), 1.0, places=8)

    def test_clutch_freezes_then_rebases_without_a_target_jump(self):
        mapper = self.mapper()
        initial = _observation("right", [0, 0, 0, 0, 0, 0, 1])
        mapper.initialize({"right": initial})
        moving = _observation("right", [0.1, 0, 0, 0, 0, 0, 1], sequence=1)
        before_clutch = mapper.map(moving)
        mapper.set_clutch("right", True)
        frozen = mapper.map(_observation("right", [0.5, 0, 0, 0, 0, 0, 1], sequence=2))
        np.testing.assert_allclose(frozen.pose, before_clutch.pose, atol=1e-12)
        mapper.set_clutch("right", False)
        rebased = mapper.map(_observation("right", [0.5, 0, 0, 0, 0, 0, 1], sequence=3))
        np.testing.assert_allclose(rebased.pose, before_clutch.pose, atol=1e-10)
        continued = mapper.map(_observation("right", [0.6, 0, 0, 0, 0, 0, 1], sequence=4))
        self.assertGreater(float(continued.pose[2]), float(rebased.pose[2]))

    def test_frame_contract_is_checked_before_mapping(self):
        mapper = self.mapper()
        initial = _observation("left", [0, 0, 0, 0, 0, 0, 1])
        mapper.initialize({"left": initial})
        wrong = _observation("left", [0, 0, 0, 0, 0, 0, 1], tracked_frame="controller")
        with self.assertRaisesRegex(ValueError, "tracked frame"):
            mapper.map(wrong)

    def test_four_tracker_mapping_emits_reference_faithful_dynamic_elbow_direction(self):
        mapper = self.mapper()
        initial = _observation(
            "left", [0, 0, 0, 0, 0, 0, 1],
            elbow_pose=[0.0, 0.2, 0.1, 0, 0, 0, 1],
        )
        mapper.initialize({"left": initial})
        current = _observation(
            "left", [0.02, 0.0, 0.03, 0, 0, 0, 1], sequence=1,
            elbow_pose=[0.02, 0.25, 0.12, 0, 0, 0, 1],
        )
        result = mapper.map(current)
        self.assertIsNotNone(result.elbow_reference_direction)
        self.assertAlmostEqual(np.linalg.norm(result.elbow_reference_direction), 1.0, places=8)

        changed = _observation(
            "left", [0.02, 0.0, 0.03, 0, 0, 0, 1], sequence=2,
            elbow_pose=[0.02, 0.12, 0.28, 0, 0, 0, 1],
        )
        changed_result = mapper.map(changed)
        self.assertFalse(np.allclose(
            result.elbow_reference_direction,
            changed_result.elbow_reference_direction,
        ))

    def test_dynamic_mapping_version_is_preserved_during_clutch(self):
        mapper = self.mapper()
        initial = _observation(
            "right", [0, 0, 0, 0, 0, 0, 1],
            elbow_pose=[0.0, -0.2, 0.1, 0, 0, 0, 1],
        )
        mapper.initialize({"right": initial})
        moving = _observation(
            "right", [0.05, 0.0, 0.02, 0, 0, 0, 1], sequence=1,
            elbow_pose=[0.05, -0.25, 0.12, 0, 0, 0, 1],
        )
        mapped = mapper.map(moving)
        mapper.set_clutch("right", True)

        frozen = mapper.map(_observation(
            "right", [0.5, 0.0, 0.2, 0, 0, 0, 1], sequence=2,
            elbow_pose=[0.5, -0.5, 0.4, 0, 0, 0, 1],
        ))

        self.assertEqual(mapped.mapping_version, "xr_incremental_v2")
        self.assertEqual(frozen.mapping_version, "xr_incremental_v2")
        self.assertEqual(frozen.elbow_reference_direction, mapped.elbow_reference_direction)


if __name__ == "__main__":
    unittest.main()
