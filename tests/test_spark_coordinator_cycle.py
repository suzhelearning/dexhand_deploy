import importlib.util
import os
from pathlib import Path
from unittest.mock import patch
import unittest

from tianji_teleop.producers.spark.simulation import OfflineSparkSimulation
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.hand_tracking.spark_replay import ReplayTick
from tests.test_reference_tjvr_receiver import packet

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional actual coordinator clock')
class SparkCoordinatorCycleTest(unittest.TestCase):
    def test_shared_cycle_advances_native_and_coordinator_once(self):
        module = 'tianji_teleop.producers.spark.coordinator_cycle'
        self.assertIsNotNone(importlib.util.find_spec(module))
        from tianji_teleop.producers.spark.coordinator_cycle import SparkCoordinatorCycle
        core = OfflineSparkSimulation(ROOT, automatic_start=False)
        self.addCleanup(core.close)
        receiver = ReferenceTjvrReceiver('offline-replay', .15, .6)
        receiver.ingest(packet(1), 1_000_000_000)
        core.step(ReplayTick(1, 1_000_000_000, receiver.try_read_latest()))
        self.assertTrue(core.coordinator.handle_intent(type('Intent', (), dict(
            action='start', sequence=1, source='tjvr', reason='test'))()).accepted)
        cycle = SparkCoordinatorCycle(core.coordinator, core.producer)
        with patch.object(core.coordinator, 'tick', wraps=core.coordinator.tick) as tick:
            result = cycle.step(1_005_000_000)
            self.assertEqual(tick.call_count, 1)
        self.assertTrue(result.receipt_accepted)
        self.assertEqual(result.native_result['guidance_updates'], 1)
        self.assertEqual(set(result.commands), {'left', 'right'})
        self.assertTrue(hasattr(result, 'native_attempt'))
        self.assertEqual(result.native_attempt, core.producer.last_attempt)

    def test_native_failure_faults_coordinator_in_same_cycle(self):
        from tianji_teleop.producers.spark.coordinator_cycle import SparkCoordinatorCycle
        core = OfflineSparkSimulation(ROOT, automatic_start=False)
        self.addCleanup(core.close)
        receiver = ReferenceTjvrReceiver('offline-replay', .15, .6)
        receiver.ingest(packet(1), 1_000_000_000)
        core.step(ReplayTick(1, 1_000_000_000, receiver.try_read_latest()))
        self.assertTrue(core.coordinator.handle_intent(type('Intent', (), dict(
            action='start', sequence=1, source='tjvr', reason='test'))()).accepted)
        cycle = SparkCoordinatorCycle(core.coordinator, core.producer)
        with patch.object(core.producer.backend, 'step', side_effect=RuntimeError('worker lost')):
            result = cycle.step(1_005_000_000)
        self.assertEqual(core.coordinator.state.state, 'fault')
        self.assertFalse(result.receipt_accepted)
        self.assertIsNone(result.native_result)
        self.assertTrue(core.producer.paused)
        self.assertTrue(hasattr(result, 'native_attempt'))
        self.assertIsNotNone(result.native_attempt['sample'])

    def test_idle_cycle_never_authorizes_or_advances_native(self):
        from tianji_teleop.producers.spark.coordinator_cycle import SparkCoordinatorCycle
        core = OfflineSparkSimulation(ROOT, automatic_start=False)
        self.addCleanup(core.close)
        cycle = SparkCoordinatorCycle(core.coordinator, core.producer)
        result = cycle.step(1_000_000_000)
        self.assertNotEqual(core.coordinator.state.state, 'teleop')
        self.assertIsNone(result.native_result)
        self.assertFalse(result.receipt_accepted)
        self.assertTrue(hasattr(result, 'native_attempt'))
        self.assertIsNone(result.native_attempt)
        with self.assertRaises(ValueError):
            cycle.step(1_000_000_000)
