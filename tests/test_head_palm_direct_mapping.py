import copy
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from test_head_direct_mapping import CONFIG
from test_pose_mapping import _input
from tianji_teleop.hand_tracking import target_node
from tianji_teleop.sources.common.pose_mapping import create_arm_pose_mapper


class HeadPalmDirectMappingTest(unittest.TestCase):
    def test_separate_selection_and_corrections(self):
        config = target_node._load_config(CONFIG)
        old_relative = copy.deepcopy(config['arm_pose_mapper_config'])
        old_head = copy.deepcopy(config['head_direct_mapper_config'])
        target_node.select_arm_pose_mapper(config, 'head_palm_direct')
        self.assertEqual(config['arm_pose_mapper'], 'head_palm_direct')
        new = create_arm_pose_mapper('head_palm_direct', config['arm_pose_mapper_config'])
        old = create_arm_pose_mapper('head_direct', old_head)
        for side, angle in [('left', 90), ('right', -90)]:
            sample = _input([.4,.2,-.3,*Rotation.from_euler('xyz', [25,-30,15], degrees=True).as_quat()], side=side)
            before = old.map(sample)
            after = new.map(sample)
            self.assertEqual(after.backend, 'head_palm_direct')
            np.testing.assert_allclose(Rotation.from_quat(after.pose[3:]).as_matrix(),
                (Rotation.from_quat(before.pose[3:]) * Rotation.from_euler('z', angle, degrees=True)).as_matrix(), atol=1e-12)
            delta = np.array(old_head['input_to_base_rotation'][side]) @ [.1,0,-.1]
            np.testing.assert_allclose(after.pose[:3], before.pose[:3] + delta, atol=1e-12)
            for reference in ([1,2,3,0,0,0,1], [0,0,0,1,0,0,0]):
                new.initialize(_input(reference, side=side))
                np.testing.assert_allclose(new.map(sample).pose, after.pose)
                new.reset()
        self.assertEqual(config['head_direct_mapper_config'], old_head)
        self.assertEqual(target_node._load_config(CONFIG)['arm_pose_mapper_config'], old_relative)

    def test_invalid_adjustments_are_rejected(self):
        for field, value in [('head_height_offset_m', float('nan')),
                             ('head_forward_offset_m', float('inf')),
                             ('head_forward_offset_m', True),
                             ('head_height_offset_m', True),
                             ('tcp_local_z_correction_deg', {'left': float('inf'), 'right': 90})]:
            with self.subTest(field=field, value=value), self.assertRaisesRegex(ValueError, 'must be finite numeric'):
                create_arm_pose_mapper('head_palm_direct', {field: value})

    def test_status_and_profile_restriction(self):
        import test_hand_tracking_target_node as nodes
        config = target_node._load_config(CONFIG)
        target_node.select_arm_pose_mapper(config, 'head_palm_direct')
        publisher = nodes._FakeTargetPublisher()
        node = target_node.HandTrackingTargetNode(config=config, bridge=nodes._FakeBridge(),
            session_client=nodes._FakeSessionClient(), target_publisher=publisher)
        node.tick()
        self.assertEqual(publisher.status[-1]['diagnostics']['arm_pose_mapper'], 'head_palm_direct')
        config['input_profile'] = 'manus'
        with self.assertRaisesRegex(ValueError, 'requires PICO simulation'):
            target_node.select_arm_pose_mapper(config, 'head_palm_direct')

    def test_local_z_rotation_does_not_move_hand_control_center(self):
        config = target_node._load_config(CONFIG)
        old = create_arm_pose_mapper('head_direct', config['head_direct_mapper_config'])
        new_config = dict(config['head_palm_direct_mapper_config'], head_height_offset_m=0, head_forward_offset_m=0)
        new = create_arm_pose_mapper('head_palm_direct', new_config)
        for side in ('left','right'):
            sample = _input([.4,.2,-.3,*Rotation.from_euler('xyz', [35,20,-15], degrees=True).as_quat()], side=side)
            a, b = old.map(sample).pose, new.map(sample).pose
            np.testing.assert_allclose(a[:3] + Rotation.from_quat(a[3:]).apply([0,0,.0365]),
                b[:3] + Rotation.from_quat(b[3:]).apply([0,0,.0365]), atol=1e-12)

    def test_zero_adjustments_match_head_direct(self):
        config = target_node._load_config(CONFIG)['head_direct_mapper_config']
        new = create_arm_pose_mapper('head_palm_direct', dict(config,
            head_height_offset_m=0, tcp_local_z_correction_deg={'left':0,'right':0}))
        old = create_arm_pose_mapper('head_direct', config)
        sample = _input([.3,-.2,-.4,0,0,0,1])
        np.testing.assert_allclose(new.map(sample).pose, old.map(sample).pose)

    def test_forward_offset_is_in_fixed_head_frame_not_rotating_wrist(self):
        config = target_node._load_config(CONFIG)['head_palm_direct_mapper_config']
        # Nonidentity basis detects accidental addition directly in Base axes.
        basis = Rotation.from_euler('z', 35, degrees=True).as_matrix()
        config = dict(config, input_to_base_rotation={'left': basis, 'right': basis})
        baseline = create_arm_pose_mapper('head_palm_direct', dict(config, head_forward_offset_m=0))
        shifted = create_arm_pose_mapper('head_palm_direct', dict(config, head_forward_offset_m=.1))
        for side in ('left', 'right'):
            for angle in (0, 90):
                sample = _input([.3,.2,-.3,*Rotation.from_euler('z', angle, degrees=True).as_quat()], side=side)
                a, b = baseline.map(sample).pose, shifted.map(sample).pose
                np.testing.assert_allclose(b[:3]-a[:3], basis @ [.1,0,0], atol=1e-12)
                np.testing.assert_allclose(b[3:], a[3:])

    def test_recovery_uses_fixed_mapping(self):
        import test_hand_tracking_target_bridge as fixtures
        config = target_node._load_config(CONFIG)
        config['active_sides'] = ('right',)
        config['active_hand_sides'] = ()
        target_node.select_arm_pose_mapper(config, 'head_palm_direct')
        bridge = target_node.create_bridge_from_config(config, router_zid='router', observation_publisher_instance_id='obs')
        bridge.ingest_arm_observation(fixtures._arm(pose=[.1,-.2,-.3,0,0,0,1]))
        bridge.start(now_ns=1_000_000_000)
        bridge.ingest_arm_observation(fixtures._arm(2, timestamp_ns=1_100_000_000, valid=False))
        self.assertFalse(bridge.tick(now_ns=1_100_000_000).arm[0].tracking_valid)
        sample = [.4,-.2,-.3,0,0,0,1]
        bridge.ingest_arm_observation(fixtures._arm(3, timestamp_ns=1_200_000_000, pose=sample))
        actual = bridge.tick(now_ns=1_200_000_000).arm[0]
        expected = create_arm_pose_mapper('head_palm_direct', config['arm_pose_mapper_config']).map(_input(sample))
        np.testing.assert_allclose(actual.pose, expected.pose)
