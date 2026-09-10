import os
from pathlib import Path
import unittest

from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.hand_tracking.spark_replay import ReplayTick
from tianji_teleop.producers.spark.simulation import OfflineSparkSimulation
from tests.test_reference_tjvr_receiver import packet

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional actual simulation session')
class SparkInteractiveSimTest(unittest.TestCase):
    def make(self):
        core = OfflineSparkSimulation(ROOT, automatic_start=False)
        self.addCleanup(core.close)
        source = ReferenceTjvrReceiver('offline-replay', .15, .6)
        source.ingest(packet(1), 1_000_000_000)
        return core, source.try_read_latest()

    def test_valid_input_does_not_authorize_without_explicit_request(self):
        core, sample = self.make()
        self.assertIsNone(core.step(ReplayTick(1, 1_000_000_000, sample)))
        self.assertEqual(core.coordinator.state.state, 'idle')
        self.assertEqual(core.control_ticks, 0)
        self.assertTrue(core.request_start(1_005_000_000))
        self.assertIsNotNone(core.step(ReplayTick(2, 1_005_000_000, None)))
        self.assertEqual(core.control_ticks, 1)

    def test_request_with_stale_input_is_not_queued_for_later_automatic_start(self):
        core, sample = self.make()
        core.step(ReplayTick(1, 1_000_000_000, sample))
        self.assertFalse(core.request_start(1_300_000_000))
        source = ReferenceTjvrReceiver('offline-replay', .15, .6)
        source.ingest(packet(1), 1_000_000_000)
        source.ingest(packet(2), 1_300_000_000)
        self.assertIsNone(core.step(ReplayTick(2, 1_300_000_000, source.try_read_latest())))
        self.assertEqual(core.control_ticks, 0)
