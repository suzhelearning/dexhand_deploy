import importlib.util
import os
import struct
import zlib
from pathlib import Path
import unittest

from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.protocol.messages import SessionState
from tests.test_reference_tjvr_receiver import packet

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional actual native SPARK worker')
class SparkProducerTest(unittest.TestCase):
    def producer(self):
        name = 'tianji_teleop.producers.spark.node'
        self.assertIsNotNone(importlib.util.find_spec(name), 'missing SPARK producer')
        from tianji_teleop.producers.spark.node import SparkProducer
        from tianji_teleop.hand_tracking.spark_worker_client import SparkWorkerClient
        worker = SparkWorkerClient(worker=ROOT / 'build/spark-native/spark_native_worker',
            config=ROOT / 'src/tianji_teleop/config/producers/spark_reference.yaml',
            model=ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_wuji2.xml',
            urdf=ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_s_ccs_696_v4_local.urdf', deterministic_test=True)
        self.addCleanup(worker.close)
        return SparkProducer(worker, run_id='run', execution_epoch=1, publisher_instance_id='producer',
                             coordinator_instance_id='coordinator', router_zid='router',
                             receiver_instance_id='receiver', maximum_receipt_age_ns=100_000_000,
                             session_timeout_ns=200_000_000, max_in_flight=2)

    def state(self, phase, seq=1, peer='coordinator', time=1_000_000_000):
        return SessionState(1, seq, time, phase, 'test', 'coordinator', 1, peer, 'router')

    def sample(self):
        receiver = ReferenceTjvrReceiver('receiver', .15, .6)
        receiver.ingest(packet(1), 1_000_000_000)
        return receiver.try_read_latest()

    def test_no_native_progress_without_authorized_session(self):
        producer = self.producer()
        self.assertFalse(producer.status(1_000_000_000).ready)
        self.assertTrue(producer.status(1_000_000_000).healthy)
        producer.update_input(self.sample())
        self.assertIsNone(producer.tick(1_000_000_000))
        self.assertFalse(producer.update_session(self.state('teleop', peer='foreign')))
        self.assertIsNone(producer.tick(1_005_000_000))
        self.assertTrue(producer.update_session(self.state('teleop')))
        pair = producer.tick(1_010_000_000)
        self.assertEqual(pair.tick_id, 1)
        self.assertEqual(pair.left.sequence, pair.right.sequence)
        self.assertEqual(pair.left.diagnostics['algorithm'],
                         'spark_upper_qpoases_headroom_feedforward_velocity_qp')
        self.assertEqual(producer.last_result['guidance_updates'], 1)

    def test_initial_readiness_requires_fresh_input_not_a_cached_valid_frame(self):
        producer = self.producer()
        producer.update_input(self.sample())
        self.assertTrue(producer.status(1_000_000_000).ready)
        self.assertFalse(producer.status(1_200_000_001).ready)
        self.assertTrue(producer.status(1_200_000_001).healthy)

    def test_native_tick_consumes_sample_once_and_stops_on_timeout(self):
        producer = self.producer()
        producer.update_input(self.sample())
        producer.update_session(self.state('teleop'))
        first = producer.tick(1_000_000_000)
        self.assertTrue(hasattr(producer, 'last_attempt'))
        self.assertEqual(producer.last_attempt['sample'], self.sample().to_dict())
        receipt = dict(schema_version=1, kind='arm_bilateral_receipt', run_id='run', execution_epoch=1,
            tick_id=1, timestamp_ns=1_000_000_001, publisher_instance_id='coordinator', router_zid='router',
            stage='coordinator_command', accepted=True, reason='accepted',
            command_position_rad={side: getattr(first, side).position_rad for side in ('left', 'right')})
        self.assertTrue(producer.observe_execution(receipt, 1_000_000_001))
        self.assertFalse(producer.update_input(self.sample()))  # repeated delivery
        second = producer.tick(1_005_000_000)
        self.assertEqual(second.tick_id, 2)
        self.assertEqual(producer.last_attempt, dict(tick_id=2, timestamp_ns=1_005_000_000, sample=None))
        self.assertIsNone(producer.tick(1_106_000_000))
        self.assertTrue(producer.paused)
        self.assertEqual(producer.last_result['guidance_updates'], 2)

    def test_session_return_freezes_and_does_not_implicitly_resume(self):
        producer = self.producer()
        producer.update_input(self.sample())
        producer.update_session(self.state('teleop'))
        producer.tick(1_000_000_000)
        producer.update_session(self.state('returning', seq=2))
        self.assertIsNone(producer.tick(1_005_000_000))
        producer.update_session(self.state('teleop', seq=3))
        self.assertIsNone(producer.tick(1_010_000_000))

    def test_reference_input_loss_is_not_a_producer_health_failure(self):
        producer = self.producer()
        source = ReferenceTjvrReceiver('receiver', .15, .6)
        source.ingest(packet(1), 1_000_000_000)
        producer.update_input(source.try_read_latest())
        producer.update_session(self.state('teleop'))
        producer.tick(1_000_000_000)
        invalid = bytearray(packet(2))
        struct.pack_into('<I', invalid, 40, 0x3f)
        struct.pack_into('<I', invalid, len(invalid) - 4, zlib.crc32(invalid[:-4]))
        source.ingest(bytes(invalid), 1_005_000_000)
        producer.update_input(source.try_read_latest())
        status = producer.status(1_005_000_000)
        self.assertTrue(status.ready)
        self.assertTrue(status.healthy)
        self.assertFalse(status.diagnostics['latest_skeleton_valid'])
