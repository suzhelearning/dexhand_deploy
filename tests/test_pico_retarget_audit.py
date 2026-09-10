from pathlib import Path
import tempfile
import time
import unittest

from tests import test_pico_hand_service as service_fixture
from tests import test_dual_input_recorder as recorder_fixture
from tianji_teleop.protocol import topics
from tianji_teleop.recording.session_h5 import SessionH5Reader


class PicoRetargetAuditTest(unittest.TestCase):
    def consumed(self):
        from tianji_teleop.hand_tracking.pico_retarget_audit import TOPIC, validate_consumed
        session, service = service_fixture.PicoHandServiceTest().make(audit_processed_inputs=True)
        self.addCleanup(service.close)
        # Raw begins at receive ordinal 7: no synthetic callback 0..6.
        session.callbacks[topics.RAW_PICO_HAND_TRACKING](dict(
            service_fixture.wire_frame().to_dict(), router_zid='router'))
        deadline = time.monotonic() + 2
        while service._loop.processed_callbacks < 1 and not service.failure and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertIsNone(service.failure)
        self.assertEqual([key for key, _ in session.messages], [TOPIC])
        return validate_consumed(session.messages[0][1])

    def test_idle_processing_is_observed_without_any_command_or_authorization(self):
        value = self.consumed()
        self.assertEqual(value['processing_sequence'], 1)
        self.assertEqual(value['receiver_frame_sequence'], 7)
        self.assertEqual(value['worker_sequence'], 8)
        self.assertEqual(value['received_timestamp_ns'], 1000)
        self.assertEqual(value['valid_sides'], ['left', 'right'])

    def test_new_recorder_records_consumption_boundary_instead_of_guessing_start(self):
        from tianji_teleop.hand_tracking.pico_retarget_audit import TOPIC
        value = self.consumed()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            node = recorder_fixture.DualInputRecorderTest().node(
                recorder_fixture.Session(), path, 'pico2_hands_sim', 'pico')
            try:
                node.receive(TOPIC, value, received_time_ns=2000)
            finally:
                node.close()
            with SessionH5Reader(path) as reader:
                rows = reader.read_dual_audit()
                self.assertEqual(rows[0]['kind'], 'hand_retarget_input')
                self.assertEqual(rows[0]['payload'], value)
                self.assertEqual(reader.read_hand_command('left'), [])

    def test_inconsistent_positive_worker_sequence_is_rejected(self):
        from tianji_teleop.hand_tracking.pico_retarget_audit import validate_consumed
        value = self.consumed()
        value['worker_sequence'] += 1
        with self.assertRaises(ValueError):
            validate_consumed(value)
