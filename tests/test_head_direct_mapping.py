import copy
import subprocess
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from test_pose_mapping import _input
from tianji_teleop.sources.common.pose_mapping import create_arm_pose_mapper
from tianji_teleop.hand_tracking import target_node

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'src/tianji_teleop/config/sources/hand_tracking_target.yaml'


class HeadDirectMappingTest(unittest.TestCase):
    def test_fixed_composition_does_not_depend_on_start_pose(self):
        config = dict(input_to_base_rotation={'right': Rotation.from_euler('z', 90, degrees=True).as_matrix()},
            input_origin_in_base_m={'right': [1,2,3]},
            tracked_to_tcp_pose={'right': [0,0,.1,0,0,0,1]})
        mapper = create_arm_pose_mapper('head_direct', config)
        sample = _input([.3,.2,-.4,0,0,0,1])
        for reference in ([0,0,0,0,0,0,1], [1,2,3,1,0,0,0]):
            mapper.initialize(_input(reference))
            out = mapper.map(sample)
            self.assertEqual(out.backend, 'head_direct')
            np.testing.assert_allclose(out.pose[:3], [.8,2.3,2.7])
            np.testing.assert_allclose(Rotation.from_quat(out.pose[3:]).as_matrix(),
                Rotation.from_euler('z', 90, degrees=True).as_matrix(), atol=1e-12)
            mapper.reset()

    def test_rejects_wrong_input_frames(self):
        mapper = create_arm_pose_mapper('head_direct', {})
        for frame, tracked in [('world','wrist'), ('pico_head_current','palm')]:
            with self.assertRaises(ValueError):
                mapper.map(_input([0,0,0,0,0,0,1], reference_frame=frame, tracked_frame=tracked))

    def test_common_world_motion_leaves_head_direct_target_unchanged(self):
        from tianji_teleop.hand_tracking.pico import tracking_pose_to_current_head
        head = np.array([0,0,1.6,0,0,0,1])
        wrist = np.array([.4,-.2,1.3,0,0,0,1])
        rotation = Rotation.from_euler('xyz', [10,20,85], degrees=True)
        def transformed(pose):
            return np.r_[rotation.apply(pose[:3]) + [3,-2,1],
                (rotation * Rotation.from_quat(pose[3:])).as_quat()]
        mapper = create_arm_pose_mapper('head_direct', {})
        before = mapper.map(_input(tracking_pose_to_current_head(head, wrist)))
        after = mapper.map(_input(tracking_pose_to_current_head(transformed(head), transformed(wrist))))
        np.testing.assert_allclose(before.pose, after.pose, atol=1e-12)

    def test_nominal_geometry_preserves_control_center_and_neutral_rotation(self):
        config = target_node._load_config(CONFIG)
        home = copy.deepcopy(config['arm_pose_mapper_config']['home_pose'])
        target_node.select_arm_pose_mapper(config, 'head_direct')
        fixed = config['arm_pose_mapper_config']
        mapper = create_arm_pose_mapper('head_direct', fixed)
        for side in ('left','right'):
            wrist = np.array([.4,.2 if side == 'left' else -.2,-.3])
            out = mapper.map(_input([*wrist,0,0,0,1], side=side))
            rotation = Rotation.from_quat(out.pose[3:])
            np.testing.assert_allclose(rotation.as_matrix(), Rotation.from_quat(home[side][3:]).as_matrix(), atol=1e-12)
            center = np.array(fixed['input_origin_in_base_m'][side]) + np.array(fixed['input_to_base_rotation'][side]) @ wrist
            np.testing.assert_allclose(out.pose[:3] + rotation.apply([0,0,.0365]), center, atol=1e-12)

    def test_override_leaves_default_config_unchanged(self):
        config = target_node._load_config(CONFIG)
        original = copy.deepcopy(config)
        target_node.select_arm_pose_mapper(config, 'head_direct')
        self.assertEqual(config['arm_pose_mapper'], 'head_direct')
        self.assertEqual(original['arm_pose_mapper'], 'relative_home')
        self.assertEqual(config['arm_pose_mapper_config'], original['head_direct_mapper_config'])
        mapper = create_arm_pose_mapper(config['arm_pose_mapper'], config['arm_pose_mapper_config'])
        for side in ('left','right'):
            self.assertTrue(mapper.map(_input([.3,.2,-.3,0,0,0,1], side=side)).valid)

    def test_manus_and_unknown_override_rejected(self):
        for backend, profile in [('head_direct','manus'), ('typo','pico')]:
            config = target_node._load_config(CONFIG)
            config['input_profile'] = profile
            with self.assertRaises(ValueError):
                target_node.select_arm_pose_mapper(config, backend)

    def test_node_reports_selected_mapper(self):
        import test_hand_tracking_target_node as nodes
        config = target_node._load_config(CONFIG)
        target_node.select_arm_pose_mapper(config, 'head_direct')
        publisher = nodes._FakeTargetPublisher()
        node = target_node.HandTrackingTargetNode(config=config, bridge=nodes._FakeBridge(),
            session_client=nodes._FakeSessionClient(), target_publisher=publisher)
        node.tick()
        self.assertEqual(publisher.status[-1]['diagnostics']['arm_pose_mapper'], 'head_direct')

    def test_bridge_recovery_keeps_fixed_geometry(self):
        import test_hand_tracking_target_bridge as fixtures
        config = target_node._load_config(CONFIG)
        config['active_sides'] = ('right',)
        config['active_hand_sides'] = ()
        target_node.select_arm_pose_mapper(config, 'head_direct')
        b = target_node.create_bridge_from_config(config, router_zid='router', observation_publisher_instance_id='obs')
        b.ingest_arm_observation(fixtures._arm(pose=[.1,.2,-.3,0,0,0,1]))
        b.start(now_ns=1_000_000_000)
        b.ingest_arm_observation(fixtures._arm(2, timestamp_ns=1_100_000_000, valid=False))
        self.assertFalse(b.tick(now_ns=1_100_000_000).arm[0].tracking_valid)
        pose = [.4,-.2,-.3,0,0,0,1]
        b.ingest_arm_observation(fixtures._arm(3, timestamp_ns=1_200_000_000, pose=pose))
        out = b.tick(now_ns=1_200_000_000).arm[0]
        expected = create_arm_pose_mapper('head_direct', config['arm_pose_mapper_config']).map(_input(pose))
        np.testing.assert_allclose(out.pose, expected.pose)
        self.assertEqual(out.mapping_backend, 'head_direct')

    def test_launcher_exposes_and_validates_flag_before_starting(self):
        help_result = subprocess.run(['bash','scripts/run_session.sh','--help'], cwd=ROOT, capture_output=True, text=True)
        self.assertIn('--arm-pose-mapper', help_result.stdout)
        for profile, backend in [('hand_tracking_sim_manus','head_direct'), ('hand_tracking_sim','typo')]:
            result = subprocess.run(['bash','scripts/run_session.sh','--profile',profile,
                '--arm-pose-mapper',backend], cwd=ROOT, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn('--arm-pose-mapper', result.stderr)
