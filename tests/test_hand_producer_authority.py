from copy import deepcopy
import unittest

from tests import test_hand_retarget_backend
from tianji_teleop.protocol.messages import SessionState
from tianji_teleop.producers import hand_retarget


class Backend:
    def __init__(self):
        self.calls = 0
        self.row = test_hand_retarget_backend.HandRetargetResultTest().row()

    def retarget(self, points, *, sequence, timestamp_ns):
        self.calls += 1
        row = deepcopy(self.row)
        row.update(callback_sequence=sequence, timestamp_ns=timestamp_ns)
        return hand_retarget.validate_result(row, sequence=sequence, timestamp_ns=timestamp_ns)


class HandProducerAuthorityTest(unittest.TestCase):
    def make(self):
        self.assertTrue(hasattr(hand_retarget, 'HandRetargetProducer'))
        backend = Backend()
        return backend, hand_retarget.HandRetargetProducer(backend,
            publisher_instance_id='hand', router_zid='router', coordinator_instance_id='coord',
            receiver_instance_id='manus', freshness_ns=100)

    def state(self, phase='teleop', peer='coord', seq=1):
        return SessionState(1, seq, 1000, phase, 'test', 'coordinator', 1, peer, 'router')

    def test_input_snapshot_survives_command_consumption_without_faking_missing_side(self):
        backend, producer = self.make()
        self.assertTrue(hasattr(producer, 'input_snapshot'))
        self.assertIsNone(producer.input_snapshot)
        backend.row['left']['valid'] = False
        producer.update_session(self.state())
        producer.update_input([0.] * 126, sequence=1, timestamp_ns=1000,
                              receiver_instance_id='manus', now_ns=1000)
        producer.commands(1000)
        snapshot = producer.input_snapshot
        self.assertEqual(snapshot, dict(sequence=1, timestamp_ns=1000, valid_sides=['right']))
        snapshot['valid_sides'].append('left')
        self.assertEqual(producer.input_snapshot['valid_sides'], ['right'])

    def test_idle_updates_original_filter_but_cannot_publish(self):
        backend, producer = self.make()
        self.assertTrue(producer.update_input([0.] * 126, sequence=1, timestamp_ns=1000,
                                              receiver_instance_id='manus', now_ns=1000))
        self.assertEqual(backend.calls, 1)
        self.assertEqual(producer.commands(1000), {})
        self.assertFalse(producer.update_session(self.state(peer='foreign')))
        self.assertEqual(producer.commands(1000), {})
        self.assertTrue(producer.update_session(self.state()))
        commands = producer.commands(1000)
        self.assertEqual(set(commands), {'left', 'right'})
        self.assertEqual(commands['left'].position_rad, backend.row['left']['position_rad'])
        self.assertEqual(producer.commands(1000), {})  # no callback replay
        self.assertEqual(backend.calls, 1)

    def test_invalid_side_stale_or_foreign_input_never_commands(self):
        backend, producer = self.make()
        producer.update_session(self.state())
        self.assertFalse(producer.update_input([0.] * 126, sequence=1, timestamp_ns=1000,
            receiver_instance_id='old', now_ns=1000))
        self.assertEqual(backend.calls, 0)
        backend.row['left']['valid'] = False
        producer.update_input([0.] * 126, sequence=1, timestamp_ns=1000,
            receiver_instance_id='manus', now_ns=1000)
        self.assertEqual(set(producer.commands(1000)), {'right'})
        producer.update_input([0.] * 126, sequence=2, timestamp_ns=1001,
            receiver_instance_id='manus', now_ns=1001)
        self.assertEqual(producer.commands(1102), {})

    def test_return_blocks_commands_and_requires_new_callback_on_restart(self):
        _, producer = self.make()
        producer.update_input([0.] * 126, sequence=1, timestamp_ns=1000,
            receiver_instance_id='manus', now_ns=1000)
        producer.update_session(self.state())
        producer.update_session(self.state('returning', seq=2))
        self.assertEqual(producer.commands(1000), {})
        producer.update_session(self.state(seq=3))
        self.assertEqual(producer.commands(1000), {})
