import unittest

from tests.test_reference_tjvr_receiver import packet
from tianji_teleop.hand_tracking import spark_replay
from tianji_teleop.hand_tracking.tjvr_trace import TjvrRecord


class SparkReplayTest(unittest.TestCase):
    def scheduler(self, records, **kwargs):
        self.assertTrue(callable(getattr(spark_replay, 'iter_reference_ticks', None)))
        return spark_replay.iter_reference_ticks(records, **kwargs)

    def test_receive_order_then_latest_at_tick_boundary(self):
        records = [TjvrRecord(t, packet(i + 1)) for i, t in enumerate((0, 1, 3, 12))]
        ticks = list(self.scheduler(records, period_ns=5, tail_ns=5))
        self.assertEqual([tick.tick_id for tick in ticks], [1, 2, 3, 4])
        self.assertEqual([tick.sample.observation.frame.sequence if tick.sample else None
                          for tick in ticks], [1, 3, None, 4])
        self.assertEqual([tick.now_ns for tick in ticks], [1000000000 + x for x in (0, 5, 10, 15)])
        self.assertEqual(ticks[1].sample.observation.frame.received_timestamp_ns, 1000000003)

    def test_gate_is_before_latest_selection(self):
        records = [TjvrRecord(t, packet(i + 1, x)) for i, (t, x) in
            enumerate(((0, 10.), (1, 10.3), (2, 10.31), (3, 10.32), (4, 10.33)))]
        ticks = list(self.scheduler(records, period_ns=5, tail_ns=5))
        self.assertEqual(ticks[-1].sample.resynchronization_generation, 1)
        self.assertEqual(ticks[-1].sample.observation.frame.sequence, 5)

    def test_empty_trace_and_bad_time_rejected(self):
        for records in ([], [TjvrRecord(-1, packet(1))],
                        [TjvrRecord(float('nan'), packet(1))],
                        [TjvrRecord(5, packet(1)), TjvrRecord(4, packet(2))]):
            with self.assertRaises(ValueError):
                list(self.scheduler(records))

    def test_explicit_positive_period_and_tail(self):
        for kwargs in ({'period_ns': 0}, {'period_ns': True}, {'tail_ns': -1}):
            with self.assertRaises(ValueError):
                list(self.scheduler([TjvrRecord(0, packet(1))], **kwargs))

    def test_native_line_preserves_raw_and_absent_sample(self):
        ticks = list(self.scheduler([TjvrRecord(0, packet(1))], period_ns=5, tail_ns=5))
        self.assertEqual(spark_replay.encode_tick(ticks[0]),
            'TJSC1 1 1000000000 1000000000 0 0 ' + packet(1).hex() + '\n')
        self.assertEqual(spark_replay.encode_tick(ticks[1]), 'TJSC1 2 1000000005 0 0 0 -\n')
