import importlib.util
import unittest

from tests.test_pico_hand_tracking import _packet
from tianji_teleop.hand_tracking.pico import parse_pico_packet


class PicoRawInputQueueTest(unittest.TestCase):
    def make(self, capacity=256):
        self.assertIsNotNone(importlib.util.find_spec('tianji_teleop.hand_tracking.pico_raw_input'))
        from tianji_teleop.hand_tracking.pico_raw_input import PicoRawInputQueue
        return PicoRawInputQueue(receiver_instance_id='pico', router_zid='router',
                                 connection_generation=1, capacity=capacity)

    def payload(self, sequence=0):
        frame = parse_pico_packet(_packet(), receiver_instance_id='pico', connection_generation=1,
                                 receiver_frame_sequence=sequence, received_timestamp_ns=1000+sequence)
        return dict(frame.to_dict(), router_zid='router')

    def test_each_frame_is_consumed_once_with_unchanged_native_packet(self):
        source = self.make()
        for seq in range(3):
            self.assertTrue(source.ingest(self.payload(seq)))
        for seq in range(3):
            frame = source.try_read()
            self.assertEqual(frame.receiver_frame_sequence, seq)
            self.assertEqual(frame.raw_packet, _packet())
        self.assertIsNone(source.try_read())
        self.assertFalse(source.ingest(self.payload(1)))
        self.assertIsNone(source.failure)

    def test_foreign_source_and_router_do_not_poison_current_stream(self):
        source = self.make()
        self.assertFalse(source.ingest(dict(self.payload(), receiver_instance_id='foreign')))
        self.assertFalse(source.ingest(dict(self.payload(), router_zid='foreign')))
        self.assertIsNone(source.failure)
        self.assertTrue(source.ingest(self.payload()))

    def test_non_object_wire_payload_latches_a_clear_failure(self):
        for payload in (b'null', b'[]', b'1'):
            with self.subTest(payload=payload):
                source = self.make()
                self.assertFalse(source.ingest(payload))
                self.assertIn('object', source.failure)

    def test_json_geometry_must_match_original_packet(self):
        source = self.make()
        payload = self.payload()
        payload['hands']['left']['joints'][1]['pose'][0] += .1
        self.assertFalse(source.ingest(payload))
        self.assertIn('packet', source.failure)
        self.assertIsNone(source.try_read())

    def test_overflow_and_new_generation_latch_instead_of_coalescing_or_rebinding(self):
        source = self.make(capacity=1)
        self.assertTrue(source.ingest(self.payload()))
        self.assertFalse(source.ingest(self.payload(1)))
        self.assertIn('overflow', source.failure)
        self.assertIsNone(source.try_read())
        source = self.make()
        payload = self.payload()
        payload.update(connection_generation=2, association_id='pico:2:0')
        self.assertFalse(source.ingest(payload))
        self.assertIn('generation', source.failure)
        self.assertFalse(source.ingest(self.payload()))
