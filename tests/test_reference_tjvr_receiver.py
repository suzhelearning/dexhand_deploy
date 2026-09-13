import struct
from threading import Event, Thread
import unittest
import zlib

from tests.test_legacy_pico_palm import _packet
from tests.test_reference_tjvr import changed
from tianji_teleop.hand_tracking import reference_tjvr_receiver as receiver


def packet(sequence, x=10., epoch=9):
    result = bytearray(changed(_packet(flags=0xff), 44, [x]))
    struct.pack_into('<QQ', result, 8, sequence, epoch)
    struct.pack_into('<I', result, len(result) - 4, zlib.crc32(result[:-4]))
    return bytes(result)


class ReferenceReceiverTest(unittest.TestCase):
    def test_optional_decision_sink_records_rejections_and_generation(self):
        captured = []
        source = receiver.ReferenceTjvrReceiver('test', .15, .6,
            decision_sink=lambda observation, decision, generation: captured.append(
                (observation.frame.sequence, decision.accepted, decision.reason, generation)))
        for sequence, x in ((1, 10.), (1, 10.), (2, 10.3), (3, 10.31), (4, 10.32)):
            source.ingest(packet(sequence, x), 100 + len(captured))
        self.assertEqual(captured, [(1, True, 'none', 0), (1, False, 'out_of_order', 0),
            (2, False, 'position_jump', 0), (3, False, 'position_jump', 0), (4, True, 'none', 1)])
        self.assertEqual(source.try_read_latest().resynchronization_generation, 1)

    def test_decision_sink_failure_does_not_expose_new_latest(self):
        def fail(*args):
            raise RuntimeError('audit failure')
        source = receiver.ReferenceTjvrReceiver('test', .15, .6, decision_sink=fail)
        with self.assertRaisesRegex(RuntimeError, 'audit failure'):
            source.ingest(packet(1), 100)
        self.assertIsNone(source.try_read_latest())

    def test_typed_raw_sink_preserves_receiver_ordinal_before_gate(self):
        captured = []
        source = receiver.ReferenceTjvrReceiver('test', .15, .6, raw_frame_sink=captured.append)
        source.ingest(b'bad', 100)
        source.ingest(packet(1), 101)
        source.ingest(packet(1), 102)
        self.assertEqual([row.frame.receiver_frame_sequence for row in captured], [2, 3])
        self.assertEqual([row.frame.sequence for row in captured], [1, 1])

    def make(self, sink=None):
        self.assertTrue(callable(getattr(receiver, 'ReferenceTjvrReceiver', None)))
        return receiver.ReferenceTjvrReceiver('test', .15, .6, raw_sink=sink)

    def test_latest_consumed_once_without_fifo(self):
        source = self.make()
        self.assertIsNone(source.try_read_latest())
        source.ingest(packet(1), 100)
        source.ingest(packet(2), 101)
        self.assertEqual(source.try_read_latest().observation.frame.sequence, 2)
        self.assertIsNone(source.try_read_latest())
        self.assertEqual(source.stats()['superseded'], 1)

    def test_slow_raw_callback_does_not_block_previous_latest(self):
        entered = Event()
        release = Event()

        def slow_callback(_frame):
            entered.set()
            release.wait(1.0)

        source = receiver.ReferenceTjvrReceiver(
            'test', .15, .6, raw_frame_sink=slow_callback
        )
        source.ingest(packet(1), 100)
        reader_done = Event()
        consumed = []

        def read_latest():
            consumed.append(source.try_read_latest())
            reader_done.set()

        ingest_thread = Thread(
            target=lambda: source.ingest(packet(2), 101), daemon=True
        )
        reader_thread = Thread(target=read_latest, daemon=True)
        ingest_thread.start()
        self.assertTrue(entered.wait(1.0))
        reader_thread.start()
        try:
            self.assertTrue(
                reader_done.wait(.1),
                'slow raw callback must not hold the latest-frame lock',
            )
        finally:
            release.set()
            ingest_thread.join(1.0)
            reader_thread.join(1.0)
        self.assertFalse(ingest_thread.is_alive())
        self.assertFalse(reader_thread.is_alive())
        self.assertIsNotNone(consumed[0])
        self.assertEqual(consumed[0].observation.frame.sequence, 1)
        self.assertEqual(source.try_read_latest().observation.frame.sequence, 2)

    def test_resynchronization_generation_survives_superseding(self):
        source = self.make()
        for sequence, x in ((1, 10.), (2, 10.3), (3, 10.31), (4, 10.32), (5, 10.33)):
            source.ingest(packet(sequence, x), 100 + sequence)
        latest = source.try_read_latest()
        self.assertFalse(latest.stream_discontinuity)
        self.assertEqual(latest.resynchronization_generation, 1)
        self.assertEqual(latest.observation.frame.sequence, 5)

    def test_raw_sink_before_gate_and_no_invalid_packet(self):
        captured = []
        source = self.make(lambda raw, receive_ns: captured.append((raw, receive_ns)))
        for seq in (1, 1, 2):
            source.ingest(packet(seq), 100 + seq)
        source.ingest(b'bad', 104)
        self.assertEqual(len(captured), 3)
        self.assertEqual(source.stats()['accepted'], 2)
        self.assertEqual(source.stats()['rejected'], 1)
        self.assertEqual(source.stats()['malformed'], 1)

    def test_schema_roundtrip_and_raw_tampering_rejected(self):
        source = self.make()
        source.ingest(packet(1), 123)
        sample = source.try_read_latest()
        restored = receiver.ReceivedTjvrFrame.from_dict(sample.to_dict())
        self.assertEqual(restored.to_dict(), sample.to_dict())
        invalid = sample.to_dict()
        invalid['raw_packet_base64'] = 'AAAA'
        with self.assertRaises(ValueError):
            receiver.ReceivedTjvrFrame.from_dict(invalid)
        for key, value in (('schema_version', 2), ('resynchronization_generation', -1),
                           ('received_timestamp_ns', True), ('stream_discontinuity', 'false')):
            invalid = sample.to_dict()
            invalid[key] = value
            with self.assertRaises(ValueError):
                receiver.ReceivedTjvrFrame.from_dict(invalid)

    def test_invalid_receive_time_does_not_publish_or_record(self):
        captured = []
        source = self.make(lambda *args: captured.append(args))
        for value in (0, -1, True):
            with self.assertRaises(ValueError):
                source.ingest(packet(1), value)
        self.assertIsNone(source.try_read_latest())
        self.assertEqual(captured, [])

    def test_new_epoch_does_not_clear_receiver_generation(self):
        source = self.make()
        for seq, x in ((1, 10.), (2, 10.3), (3, 10.31), (4, 10.32)):
            source.ingest(packet(seq, x), 100 + seq)
        source.ingest(packet(1, epoch=10), 200)
        latest = source.try_read_latest()
        self.assertEqual(latest.resynchronization_generation, 1)
        self.assertEqual(latest.observation.frame.tracking_epoch, 10)
