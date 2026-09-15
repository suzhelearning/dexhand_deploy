import unittest
from threading import Lock
from unittest.mock import patch

from tianji_teleop.hand_tracking.spark_worker_client import SparkWorkerClient
from tianji_teleop.hand_tracking.spark_replay import ReplayTick


class WorkerTimingTest(unittest.TestCase):
    def test_success_timing_does_not_change_result(self):
        client = SparkWorkerClient.__new__(SparkWorkerClient)
        client._lock = Lock()
        client._closed = False
        client._tick = client._now = 0
        client._deterministic = True
        client._binary_results = False
        result = {'sentinel': 1}
        with patch.object(client, '_exchange', return_value=result), \
             patch('tianji_teleop.hand_tracking.spark_worker_client._validate_result'), \
             patch('tianji_teleop.hand_tracking.spark_worker_client.time.perf_counter',
                   side_effect=[1., 1.001, 1.003, 1.004]):
            self.assertIs(client.step(ReplayTick(1, 100, None)), result)
        self.assertAlmostEqual(client.last_timing['request_encode'], .001)
        self.assertAlmostEqual(client.last_timing['ipc_roundtrip_decode'], .002)
        self.assertAlmostEqual(client.last_timing['result_validate'], .001)
        self.assertNotIn('timing', result)
