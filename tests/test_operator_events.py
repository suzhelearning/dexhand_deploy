import unittest
from dataclasses import replace

from tianji_teleop.hand_tracking import operator_input as operator


class OperatorEventsTest(unittest.TestCase):
    def setUp(self):
        self.gate = operator.OperatorEdgeFilter(source='xr-bound', side='left',
            action='start_request', epoch=1, freshness_ns=100, stable_ns=20)

    def sample(self, seq, time, pressed, **changes):
        return replace(operator.OperatorObservation(source='xr-bound', side='left',
            sequence=seq, epoch=1, receive_time_ns=time, valid=True, available=True,
            action='start_request', pressed=pressed, confidence=1.), **changes)

    def test_requires_release_then_stable_press_and_does_not_repeat(self):
        self.assertIsNone(self.gate.update(self.sample(0, 100, True), now_ns=100))
        self.assertIsNone(self.gate.update(self.sample(1, 110, False), now_ns=110))
        self.assertIsNone(self.gate.update(self.sample(2, 120, True), now_ns=120))
        event = self.gate.update(self.sample(3, 140, True), now_ns=140)
        self.assertEqual(event.action, 'start_request')
        self.assertEqual(event.edge, 'rising')
        self.assertEqual(event.sequence, 3)
        self.assertIsNone(self.gate.update(self.sample(4, 170, True), now_ns=170))

    def test_duplicate_or_foreign_packet_cannot_advance_debounce(self):
        self.gate.update(self.sample(0, 100, False), now_ns=100)
        self.gate.update(self.sample(1, 110, True), now_ns=110)
        self.assertIsNone(self.gate.update(self.sample(1, 140, True), now_ns=140))
        self.assertIsNone(self.gate.update(self.sample(2, 150, True, source='other'), now_ns=150))
        self.assertIsNotNone(self.gate.update(self.sample(2, 160, True), now_ns=160))

    def test_gap_invalid_and_stale_require_new_release(self):
        for changes, now in ((dict(valid=False), 130), ({}, 300), ({'sequence': 8}, 130)):
            with self.subTest(changes=changes):
                self.setUp()
                self.gate.update(self.sample(0, 100, False), now_ns=100)
                self.gate.update(self.sample(1, 110, True), now_ns=110)
                sample = replace(self.sample(2, 130, True), **changes)
                self.assertIsNone(self.gate.update(sample, now_ns=now))
                self.assertIsNone(self.gate.update(self.sample(sample.sequence+1, now+10, True), now_ns=now+10))

    def test_new_epoch_must_be_explicitly_rebound_not_auto_accepted(self):
        self.gate.update(self.sample(0, 100, False), now_ns=100)
        self.assertIsNone(self.gate.update(self.sample(1, 110, True, epoch=2), now_ns=110))
        self.assertIsNone(self.gate.update(self.sample(2, 150, True, epoch=2), now_ns=150))

    def test_low_quality_cannot_generate_request(self):
        self.gate.update(self.sample(0, 100, False), now_ns=100)
        self.gate.update(self.sample(1, 110, True), now_ns=110)
        self.assertIsNone(self.gate.update(self.sample(2, 140, True, confidence=.1), now_ns=140))
        self.assertIsNone(self.gate.update(self.sample(3, 160, True), now_ns=160))

    def test_replay_never_produces_an_action(self):
        for seq, pressed in enumerate((False, True, True)):
            self.assertIsNone(self.gate.update(self.sample(seq, 100+30*seq, pressed),
                now_ns=100+30*seq, replay=True))
        self.assertIsNone(self.gate.update(self.sample(3, 200, True), now_ns=200))

    def test_strict_observation_fields(self):
        for changes in ({'sequence': True}, {'confidence': float('nan')}, {'pressed': 1},
                        {'side': 'unknown'}, {'action': 'drive_robot'}, {'epoch': -1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(self.sample(0, 100, False), **changes)
