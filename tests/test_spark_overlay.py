import importlib.util
import unittest

import mujoco
import numpy as np

from tianji_teleop.hand_tracking.reference_tjvr import parse_reference_tjvr_packet
from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND
from tests.test_reference_tjvr_receiver import packet


class SparkOverlayTest(unittest.TestCase):
    def make(self):
        name = 'tianji_teleop.executors.mujoco.spark_overlay'
        self.assertIsNotNone(importlib.util.find_spec(name))
        from tianji_teleop.executors.mujoco.spark_overlay import SparkOverlay
        return SparkOverlay('receiver')

    def frame(self, seq=1, receiver='receiver'):
        return parse_reference_tjvr_packet(packet(seq), receiver_instance_id=receiver,
                                          receiver_frame_sequence=seq, received_timestamp_ns=1000)

    def native(self):
        return dict(algorithm=SPARK_BACKEND, tick_id=1, timestamp_ns=1000,
            left=dict(target_position=[.3, .2, .1], target_quaternion_xyzw=[0, 0, 0, 1]),
            right=dict(target_position=[.3, -.2, .1], target_quaternion_xyzw=[0, 0, 0, 1]))

    def test_raw_packet_and_actual_ik_targets_have_distinct_world_poses(self):
        overlay = self.make()
        frame = self.frame()
        self.assertTrue(overlay.ingest_raw(frame))
        self.assertTrue(overlay.ingest_native(self.native(), execution_epoch=1))
        markers, bones = overlay.geometry(1001)
        poses = {row['label']: row for row in markers}
        np.testing.assert_array_equal(poses['TJVR corrected palm left']['position'], frame.frame.upper_limb_points[3])
        np.testing.assert_array_equal(poses['TJVR packet target left']['position'], frame.frame.left_pose[:3])
        np.testing.assert_array_equal(poses['SPARK IK target left']['position'], [.3, .2, .1])
        self.assertEqual(len(bones), 7)

    def test_foreign_old_epoch_and_expired_raw_do_not_replace_live_geometry(self):
        overlay = self.make()
        self.assertFalse(overlay.ingest_raw(self.frame(receiver='foreign')))
        self.assertTrue(overlay.ingest_raw(self.frame()))
        self.assertFalse(overlay.ingest_raw(self.frame()))
        self.assertTrue(overlay.ingest_native(self.native(), execution_epoch=2))
        self.assertFalse(overlay.ingest_native(self.native(), execution_epoch=1))
        markers, bones = overlay.geometry(600_001_001)
        self.assertEqual(bones, [])
        self.assertTrue(all(row['label'].startswith('SPARK') and '[stale]' in row['label'] for row in markers))

    def test_render_is_passive_and_respects_scene_capacity(self):
        overlay = self.make()
        overlay.ingest_raw(self.frame())
        overlay.ingest_native(self.native(), execution_epoch=1)
        model = mujoco.MjModel.from_xml_string('<mujoco/>')
        data = mujoco.MjData(model)
        before = data.qpos.copy()
        scene = mujoco.MjvScene(model, 5)
        overlay.append(scene, mujoco, 1001)
        self.assertEqual(scene.ngeom, 5)
        np.testing.assert_array_equal(data.qpos, before)
