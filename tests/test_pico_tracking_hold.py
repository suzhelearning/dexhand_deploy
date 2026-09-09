import unittest
import numpy as np
import test_hand_tracking_target_bridge as fixtures
import test_arm_coordinator as coordinator_fixtures
from tianji_teleop.protocol.messages import ArmTargetCommand, ProtocolEnvelope


class TrackingHoldTest(unittest.TestCase):
    def bridge(self):
        b = fixtures._bridge()
        b.active_hand_sides = ()
        b.hold_on_tracking_loss = True
        b.ingest_arm_observation(fixtures._arm(pose=[0,0,0,0,0,0,1]))
        b.start(now_ns=1_000_000_000)
        return b

    def test_loss_hold_then_recovery_preserves_original_mapping(self):
        b = self.bridge()
        b.ingest_arm_observation(fixtures._arm(2, timestamp_ns=1_100_000_000, valid=False))
        held = b.tick(now_ns=1_100_000_000).arm[0]
        self.assertFalse(held.tracking_valid)
        self.assertTrue(b.started)
        b.ingest_arm_observation(fixtures._arm(3, timestamp_ns=1_200_000_000,
            pose=[.3,0,0,0,0,0,1]))
        recovered = b.tick(now_ns=1_200_000_000).arm[0]
        self.assertTrue(recovered.tracking_valid)
        np.testing.assert_allclose(recovered.pose[:3], [1.3,2,3])

    def test_silence_is_not_tracking_loss(self):
        b = self.bridge()
        b.ingest_arm_observation(fixtures._arm(2, timestamp_ns=1_100_000_000, valid=False))
        with self.assertRaises(fixtures.TargetBridgeInputRejected):
            b.tick(now_ns=2_000_000_000)

    def test_invalid_foreign_or_rollback_packet_is_not_hold(self):
        b = self.bridge()
        for packet in [fixtures._arm(2, valid=False, publisher='foreign'),
                       fixtures._arm(1, valid=False),
                       fixtures._arm(2, valid=False, reference='different_frame')]:
            with self.assertRaises(fixtures.TargetBridgeInputRejected):
                b.ingest_arm_observation(packet)

    def test_initial_invalid_cannot_start(self):
        b = fixtures._bridge()
        b.active_hand_sides = ()
        b.hold_on_tracking_loss = True
        b.ingest_arm_observation(fixtures._arm(valid=False))
        with self.assertRaises(fixtures.TargetBridgeInputRejected):
            b.start(now_ns=1_000_000_000)

    def test_node_keeps_authorization_and_original_reference_until_disconnect(self):
        import test_hand_tracking_target_node as nodes
        b = self.bridge()
        b.reset()
        client = nodes._FakeSessionClient()
        publisher = nodes._FakeTargetPublisher()
        node = nodes.HandTrackingTargetNode(bridge=b, session_client=client,
            target_publisher=publisher, active_sides=('right',), active_hand_sides=(),
            rate_hz=60, clock=lambda: 1_000_000_000)
        self.assertTrue(node.on_arm_observation(fixtures._arm(pose=[0,0,0,0,0,0,1])))
        self.assertTrue(node.request_start())
        client.start_authorized = True
        node.tick(now_ns=1_000_000_000)
        self.assertTrue(node.on_arm_observation(fixtures._arm(2, timestamp_ns=1_100_000_000, valid=False)))
        node.tick(now_ns=1_100_000_000)
        self.assertEqual(node.phase, 'teleop')
        self.assertFalse(publisher.arm[-1]['tracking_valid'])
        self.assertEqual(client.return_requests, [])
        node.on_arm_observation(fixtures._arm(3, timestamp_ns=1_200_000_000, pose=[.3,0,0,0,0,0,1]))
        recovered = node.tick(now_ns=1_200_000_000)
        np.testing.assert_allclose(recovered.arm[0].pose[:3], [1.3,2,3])
        self.assertEqual(client.return_requests, [])
        node.tick(now_ns=2_000_000_000)
        self.assertEqual(node.phase, 'returning')
        self.assertEqual(client.return_requests, ['target_bridge_rejected'])

    def test_real_coordinator_does_not_allow_tracking_hold(self):
        setup = coordinator_fixtures.ArmCommandCoordinatorTest()
        setup.setUp()
        setup.test_one_tick_proposal_lag_is_clipped_not_faulted()
        c = setup.coordinator
        c.profile['required_capability'] = 'real'
        c._proposals['right'].value.diagnostics.update(hold=True, tracking_hold=True)
        c._validate_proposals(1_000_000_000)
        self.assertEqual(c.state.state, 'fault')

    def test_tracking_hold_does_not_bypass_hard_bounds_or_stale_timestamp(self):
        for failure in ('bounds', 'timestamp'):
            with self.subTest(failure=failure):
                setup = coordinator_fixtures.ArmCommandCoordinatorTest()
                setup.setUp()
                setup.test_one_tick_proposal_lag_is_clipped_not_faulted()
                c = setup.coordinator
                p = c._proposals['right'].value
                p.diagnostics.update(hold=True, tracking_hold=True)
                if failure == 'bounds':
                    p.position_rad[0] = 1000.0
                else:
                    c.config['command_step_time_window_s'] = .05
                    p.timestamp_ns = 1
                c._validate_proposals(1_000_000_000)
                self.assertEqual(c.state.state, 'fault')

    def test_optional_hold_wire_roundtrip_and_strict_boolean(self):
        t = ArmTargetCommand(ProtocolEnvelope(schema_version=1, sequence=1, timestamp_ns=1,
            publisher_instance_id='source', router_zid='router'),None,
            'source','right','Base_R',[0,0,0],[0,0,0,1],[0,1,0])
        self.assertNotIn('tracking_valid', t.to_dict())
        raw = dict(t.to_dict(), tracking_valid=False)
        self.assertFalse(ArmTargetCommand.from_dict(raw).tracking_valid)
        for bad in [0, 1, None, 'false']:
            with self.assertRaises(ValueError):
                ArmTargetCommand.from_dict(dict(raw, tracking_valid=bad))

    def test_coordinator_freezes_own_final_command_not_stale_producer_position(self):
        setup = coordinator_fixtures.ArmCommandCoordinatorTest()
        setup.setUp(); setup.test_one_tick_proposal_lag_is_clipped_not_faulted()
        c = setup.coordinator
        final = list(c._safe_command['right'])
        p = c._proposals['right'].value
        p.position_rad[0] -= .2
        p.diagnostics.update(hold=True, tracking_hold=True)
        c._validate_proposals(1_000_000_000)
        self.assertEqual(c.state.state, 'teleop')
        command = c._command('right', 3, 1_000_000_000)
        self.assertEqual(command.position_rad, final)
