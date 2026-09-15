from types import SimpleNamespace
import unittest

from tests.test_arm_coordinator import _status, _arm_state
from tests.test_bilateral_proposal import pair
from tianji_teleop.coordination.arm_command_coordinator import ArmCommandCoordinator


class BilateralCoordinatorTest(unittest.TestCase):
    def make(self, enabled=True):
        profile = dict(active_sides=['left', 'right'], required_capability='simulation')
        if enabled:
            profile['bilateral_proposals'] = dict(run_id='run-1', execution_epoch=1)
        config = ArmCommandCoordinator._coordinator_config(None)
        config['command_step_clipping_enabled'] = False
        co = ArmCommandCoordinator(None, publisher_instance_id='coord-1', router_zid='router-1',
                                  command_math=getattr(self, 'command_math', 'python') if enabled else 'python',
                                  profile=profile, coordinator_config=config, clock=lambda: 1_000_000_000)
        for role, name in (('source', 'src'), ('producer_arm', 'ik'), ('executor_arm', 'mujoco')):
            co.update_component(_status(role, name, 1_000_000_000))
        co.update_arm_state(_arm_state(1_000_000_000, co.robot.home_all))
        self.assertTrue(co.handle_intent(SimpleNamespace(action='start', sequence=1, source='src', reason='test')).accepted)
        return co

    def proposal(self, co, tick=1, delta=.01):
        value = pair(tick)
        for side in ('left', 'right'):
            value[side]['position_rad'] = [v + delta for v in getattr(co.robot, side + '_home_rad')]
        return value

    def ingest(self, co, value):
        self.assertTrue(callable(getattr(co, 'update_bilateral_proposal', None)), 'missing paired coordinator ingress')
        return co.update_bilateral_proposal(value)

    def test_pair_is_accepted_together_and_receipt_is_not_device_feedback(self):
        co = self.make()
        value = self.proposal(co)
        self.assertTrue(self.ingest(co, value))
        commands = co.tick()
        for side in ('left', 'right'):
            self.assertEqual(commands[side].position_rad, value[side]['position_rad'])
        receipt = co.last_bilateral_receipt
        self.assertTrue(receipt['accepted'])
        self.assertEqual(receipt['tick_id'], 1)
        self.assertEqual(receipt['stage'], 'coordinator_command')
        self.assertNotIn('actual_position_rad', receipt)
        self.assertTrue(hasattr(co, 'last_bilateral_command'))
        from tianji_teleop.protocol.bilateral import ArmBilateralCommand
        command_pair = ArmBilateralCommand.from_dict(co.last_bilateral_command)
        self.assertEqual(command_pair.reference_tick_id, 1)
        self.assertEqual(command_pair.left.to_dict(), commands['left'].to_dict())
        self.assertEqual(command_pair.right.to_dict(), commands['right'].to_dict())

    def test_one_bad_side_keeps_both_previous_commands(self):
        co = self.make()
        self.assertTrue(self.ingest(co, self.proposal(co)))
        before = co.tick()
        invalid = self.proposal(co, 2)
        invalid['right']['position_rad'][0] = 99
        self.assertTrue(self.ingest(co, invalid))  # hard bounds checked on command tick
        after = co.tick(now_ns=1_010_000_000)
        self.assertEqual(co.state.state, 'fault')
        for side in ('left', 'right'):
            self.assertEqual(after[side].position_rad, before[side].position_rad)
        self.assertFalse(co.last_bilateral_receipt['accepted'])

    def test_partial_ingress_rejected_only_in_new_mode(self):
        co = self.make()
        self.assertFalse(co.update_proposal(self.proposal(co)['left']))
        self.assertEqual(co.state.state, 'fault')
        legacy = self.make(False)
        self.assertTrue(legacy.update_proposal(self.proposal(legacy)['left']))

    def test_foreign_run_or_old_tick_cannot_replace_pair(self):
        co = self.make()
        value = self.proposal(co)
        self.assertTrue(self.ingest(co, value))
        co.tick()
        self.assertFalse(self.ingest(co, value))
        foreign = self.proposal(co, 2)
        foreign['run_id'] = 'old-run'
        self.assertFalse(self.ingest(co, foreign))
        self.assertEqual(co.state.state, 'teleop')

    def test_schema_failure_cannot_adopt_only_left_side(self):
        co = self.make()
        before = co.tick()
        bad = self.proposal(co)
        bad['right']['publisher_instance_id'] = 'other'
        self.assertFalse(self.ingest(co, bad))
        after = co.tick()
        for side in ('left', 'right'):
            self.assertEqual(before[side].position_rad, after[side].position_rad)

    def test_real_capability_does_not_enable_unvalidated_bilateral_execution(self):
        with self.assertRaisesRegex(ValueError, 'simulation'):
            ArmCommandCoordinator(None, publisher_instance_id='coord-1', router_zid='router-1',
                profile=dict(required_capability='real', active_sides=['left', 'right'],
                             bilateral_proposals=dict(run_id='run-1', execution_epoch=1)))
