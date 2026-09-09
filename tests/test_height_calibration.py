import unittest
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from test_pose_mapping import _input
from test_head_direct_mapping import CONFIG
from tianji_teleop.hand_tracking import target_node
from tianji_teleop.sources.common.pose_mapping import create_arm_pose_mapper


class HeightCalibrationTest(unittest.TestCase):
    def test_model_and_calibrated_control_height(self):
        from tianji_teleop.hand_tracking.height_calibration import horizontal_reference
        reference = horizontal_reference()
        config = target_node._load_config(CONFIG)['head_palm_direct_mapper_config']
        m = create_arm_pose_mapper('head_palm_direct', config)
        means = {'left': -.22, 'right': -.31}
        samples = {s: _input([.4,.2,means[s],0,0,0,1], side=s) for s in means}
        before = {s: m.map(v).pose for s,v in samples.items()}
        m.calibrate_height(means, reference)
        for side, sample in samples.items():
            out = m.map(sample).pose
            up = np.array(config['input_to_base_rotation'][side])[:,2]
            center = out[:3] + Rotation.from_quat(out[3:]).apply([0,0,.0365])
            self.assertAlmostEqual(up @ center, up @ reference[side]['control_position_base_m'], places=10)
            delta = out[:3]-before[side][:3]
            np.testing.assert_allclose(delta-up*(up@delta), 0, atol=1e-12)
            np.testing.assert_allclose(out[3:], before[side][3:])
            raised = replace(sample, pose=sample.pose + [0,0,.12,0,0,0,0])
            np.testing.assert_allclose(m.map(raised).pose[:3]-out[:3], up*.12, atol=1e-12)
        old = m.map(samples['left']).pose.copy()
        with self.assertRaises(ValueError):
            m.calibrate_height({'left': 0, 'right': float('nan')}, reference)
        np.testing.assert_allclose(m.map(samples['left']).pose, old)
        m.reset()
        np.testing.assert_allclose(m.map(samples['left']).pose, old)

    def test_collector_requires_fresh_stable_distinct_frames(self):
        from tianji_teleop.hand_tracking.height_calibration import HeightCalibration
        c = HeightCalibration(('left','right'))
        c.begin(1_000_000_000)
        for i in range(101):
            now = 1_000_000_000+i*20_000_000
            for s in ('left','right'):
                c.add(replace(_input([.4,.2,-.3,0,0,0,1], side=s), received_timestamp_ns=now), now)
            result = c.tick(now)
        self.assertEqual(result, {'left': -.3, 'right': -.3})
        for kind in ('invalid','silence','moving','repeated'):
            with self.subTest(kind=kind):
                c.begin(4_000_000_000)
                for i in range(101):
                    now = 4_000_000_000+i*20_000_000
                    for s in ('left','right'):
                        stamp = 4_000_000_000 if kind=='repeated' else now
                        if kind=='silence' and i>5:
                            continue
                        obs = replace(_input([.4 + (i*.002 if kind=='moving' else 0),.2,-.3,0,0,0,1], side=s),
                            valid=not(kind=='invalid' and i==10), received_timestamp_ns=stamp)
                        c.add(obs, now)
                    c.tick(now)
                self.assertEqual(c.state, 'failed')

    def test_node_calibration_blocks_start_and_is_refused_in_teleop(self):
        import test_hand_tracking_target_node as nodes
        import test_hand_tracking_target_bridge as fixtures
        config = target_node._load_config(CONFIG)
        config['active_hand_sides'] = ()
        target_node.select_arm_pose_mapper(config, 'head_palm_direct')
        clock = [1_000_000_000]
        bridge = target_node.create_bridge_from_config(config, router_zid='router', observation_publisher_instance_id='obs')
        client, publisher = nodes._FakeSessionClient(), nodes._FakeTargetPublisher()
        node = target_node.HandTrackingTargetNode(config=config, bridge=bridge, session_client=client,
            target_publisher=publisher, clock=lambda:clock[0])
        def frame(i):
            for side in ('left','right'):
                node.on_arm_observation(replace(fixtures._arm(i, timestamp_ns=clock[0], pose=[.4,.2,-.3,0,0,0,1]), side=side))
        frame(1)
        node.on_key('c')
        self.assertFalse(node.request_start())
        for i in range(2,103):
            clock[0] += 20_000_000
            frame(i)
            node.tick(now_ns=clock[0])
        self.assertEqual(publisher.status[-1]['diagnostics']['height_calibration']['state'], 'calibrated')
        self.assertEqual(client.start_requests, [])
        previous_means = publisher.status[-1]['diagnostics']['height_calibration']['human_height_m']
        self.assertTrue(node.request_height_calibration())
        clock[0] += 20_000_000
        node.on_arm_observation(replace(fixtures._arm(103, timestamp_ns=clock[0], valid=False), side='left'))
        node.tick()
        status = publisher.status[-1]['diagnostics']['height_calibration']
        self.assertEqual(status['state'], 'failed')
        self.assertEqual(status['human_height_m'], previous_means)
        frame(104)
        self.assertTrue(node.request_start())
        self.assertFalse(node.request_height_calibration())
        client.start_authorized = True
        node.tick()
        self.assertFalse(node.request_height_calibration())

    def test_frame_gap_cannot_be_hidden_by_late_tick(self):
        from tianji_teleop.hand_tracking.height_calibration import HeightCalibration
        c = HeightCalibration(('right',))
        c.begin(1_000_000_000)
        for now in (1_000_000_000, 1_400_000_000):
            c.add(replace(_input([.4,.2,-.3,0,0,0,1]), received_timestamp_ns=now), now)
        self.assertEqual(c.state, 'failed')

    def test_failed_first_calibration_cannot_silently_use_old_fixed_offset(self):
        import test_hand_tracking_target_node as nodes
        import test_hand_tracking_target_bridge as fixtures
        config = target_node._load_config(CONFIG)
        config['active_sides'], config['active_hand_sides'] = ('right',), ()
        target_node.select_arm_pose_mapper(config, 'head_palm_direct')
        clock = [1_000_000_000]
        bridge = target_node.create_bridge_from_config(config, router_zid='router', observation_publisher_instance_id='obs')
        node = target_node.HandTrackingTargetNode(config=config, bridge=bridge,
            session_client=nodes._FakeSessionClient(), target_publisher=nodes._FakeTargetPublisher(), clock=lambda:clock[0])
        node.request_height_calibration()
        clock[0] += 300_000_000
        node.tick()
        node.on_arm_observation(fixtures._arm(timestamp_ns=clock[0], pose=[.4,.2,-.3,0,0,0,1]))
        self.assertFalse(node.request_start())

    def test_model_reference_rejects_nonhorizontal_geometry(self):
        from tianji_teleop.hand_tracking.height_calibration import horizontal_reference
        from unittest.mock import patch
        import xml.etree.ElementTree as ET
        # CONFIG is config/sources/...; assets is alongside config.
        urdf = CONFIG.parents[2] / 'assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf'
        tree = ET.parse(urdf)
        tree.find(".//joint[@name='Joint3_L']/origin").set('xyz', '0 0.287 0.1')
        with patch('tianji_teleop.hand_tracking.height_calibration.ET.parse', return_value=tree):
            with self.assertRaises(ValueError):
                horizontal_reference(urdf)
