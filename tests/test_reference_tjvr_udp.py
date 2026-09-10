import importlib.util
import socket
import time
import unittest

from tests.test_reference_tjvr_receiver import packet


class ReferenceTjvrUdpTest(unittest.TestCase):
    def receiver(self, **kwargs):
        name = 'tianji_teleop.hand_tracking.reference_tjvr_udp'
        self.assertIsNotNone(importlib.util.find_spec(name))
        from tianji_teleop.hand_tracking.reference_tjvr_udp import ReferenceTjvrUdp
        receiver = ReferenceTjvrUdp(receiver_instance_id='udp', host='127.0.0.1', port=0, **kwargs)
        self.addCleanup(receiver.close)
        return receiver

    def wait_for(self, condition):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(.002)
        self.fail('receiver condition timed out')

    def test_loopback_latest_only_raw_before_gate_and_malformed_rejection(self):
        raw = []
        receiver = self.receiver(raw_sink=lambda packet, timestamp: raw.append((packet, timestamp)))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for value in (b'bad', packet(1), packet(1), packet(2)):
                sender.sendto(value, receiver.address)
        self.wait_for(lambda: receiver.stats()['datagrams'] == 4)
        self.assertEqual(receiver.stats()['malformed'], 1)
        self.assertEqual(len(raw), 3)  # duplicate preserved before gate
        latest = receiver.try_read_latest()
        self.assertEqual(latest.observation.frame.receiver_frame_sequence, 4)
        self.assertIsNone(receiver.try_read_latest())
        self.assertIsNone(receiver.failure)
        receiver.close()
        self.assertFalse(receiver.running)

    def test_sink_failure_latches_receiver_failure_and_does_not_restart(self):
        def fail(*_):
            raise RuntimeError('recording unavailable')
        receiver = self.receiver(raw_sink=fail)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(packet(1), receiver.address)
        self.wait_for(lambda: receiver.failure is not None)
        self.assertIn('recording unavailable', receiver.failure)
        self.assertIsNone(receiver.try_read_latest())
        self.assertFalse(receiver.running)

    def test_decision_recording_failure_latches_transport_before_consumption(self):
        def fail(*_):
            raise RuntimeError('decision audit unavailable')
        receiver = self.receiver(decision_sink=fail)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(packet(1), receiver.address)
        self.wait_for(lambda: receiver.failure is not None)
        self.assertIn('decision audit unavailable', receiver.failure)
        self.assertIsNone(receiver.try_read_latest())
