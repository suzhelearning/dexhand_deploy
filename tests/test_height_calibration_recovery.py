import unittest
from test_head_direct_mapping import CONFIG
import test_hand_tracking_target_node as nodes
import test_hand_tracking_target_bridge as fixtures
from tianji_teleop.hand_tracking import target_node


class CalibrationRecoveryTest(unittest.TestCase):
    def setup_node(self):
        config = target_node._load_config(CONFIG)
        config['active_sides'], config['active_hand_sides'] = ('right',), ()
        target_node.select_arm_pose_mapper(config, 'head_palm_direct')
        self.clock = [1_000_000_000]
        self.publisher = nodes._FakeTargetPublisher()
        self.client = nodes._FakeSessionClient()
        bridge = target_node.create_bridge_from_config(config, router_zid='router', observation_publisher_instance_id='obs')
        self.node = target_node.HandTrackingTargetNode(config=config, bridge=bridge,
            session_client=self.client, target_publisher=self.publisher, clock=lambda:self.clock[0])

    def successful_retry(self):
        self.assertTrue(self.node.request_height_calibration())
        for i in range(101):
            self.clock[0] += 20_000_000
            self.node.on_arm_observation(fixtures._arm(i+1, timestamp_ns=self.clock[0], pose=[.4,.2,-.3,0,0,0,1]))
            self.node.tick()

    def test_bad_packet_retry_recovers_health_before_start_intent(self):
        self.setup_node()
        self.node.request_height_calibration()
        self.assertFalse(self.node.on_arm_observation({'bad':'packet'}))
        self.assertFalse(self.publisher.status[-1]['healthy'])
        self.successful_retry()
        status = self.publisher.status[-1]
        self.assertEqual(status['diagnostics']['height_calibration']['state'], 'calibrated')
        self.assertTrue(status['healthy'])
        self.assertTrue(status['ready'])
        self.assertIsNone(status['error'])
        count = len(self.publisher.status)
        original = self.client.request_start
        def request(reason):
            self.assertGreater(len(self.publisher.status), count)
            self.assertTrue(self.publisher.status[-1]['healthy'])
            self.assertTrue(self.publisher.status[-1]['ready'])
            return original(reason)
        self.client.request_start = request
        self.assertTrue(self.node.request_start())

    def test_retry_does_not_clear_unrelated_error(self):
        self.setup_node()
        self.node._set_error('independent coordinator failure')
        self.node.request_height_calibration()
        self.node.on_arm_observation({'bad':'packet'})
        self.successful_retry()
        self.assertFalse(self.publisher.status[-1]['healthy'])
        self.assertEqual(self.publisher.status[-1]['error'], 'independent coordinator failure')
        self.assertFalse(self.node.request_start())
        self.assertEqual(self.client.start_requests, [])
