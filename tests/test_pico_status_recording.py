from pathlib import Path
import tempfile
import unittest

from tests import test_dual_input_recorder as fixture
from tianji_teleop.protocol import topics
from tianji_teleop.protocol.messages import ComponentStatus
from tianji_teleop.recording.recorder import SessionRecorderNode
from tianji_teleop.recording.session_h5 import SessionH5Reader


class PicoStatusRecordingTest(unittest.TestCase):
    def test_runtime_height_calibration_and_component_identity_are_audited(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'new.h5'
            session = fixture.Session()
            node = fixture.DualInputRecorderTest().node(session, path, 'pico2_hands_sim', 'pico')
            status = ComponentStatus(1, 1, 1000, 'source', 'hand_tracking_target', 'armed',
                True, True, ['simulation'], None,
                {'height_calibration': {'state': 'calibrated', 'means': {'left': -.3, 'right': -.31}}},
                'source-instance', 'router')
            try:
                self.assertIn(topics.SOURCE_STATUS, session.keys)
                node.receive(topics.SOURCE_STATUS, status.to_dict(), received_time_ns=2000)
            finally:
                node.close()
            with SessionH5Reader(path) as reader:
                row = reader.read_dual_audit()[0]
                self.assertEqual(row['kind'], 'component_status')
                self.assertEqual(row['payload'], {'topic': topics.SOURCE_STATUS, 'status': status.to_dict()})

    def test_legacy_schema11_subscriptions_remain_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            session = fixture.Session()
            node = SessionRecorderNode(session, Path(directory) / 'old.h5',
                source_type='hand_tracking_sim', input_profile='pico', robot_model='marvin',
                router_zid='router', recording_config={'flush_interval_s': 1.,
                    'schema_name': 'tianji-teleop-session', 'schema_version': '1.1'})
            try:
                for topic in (topics.SOURCE_STATUS, topics.PRODUCER_STATUS, topics.COORDINATOR_STATUS,
                              topics.EXECUTOR_STATUS, topics.hand_executor_status('left')):
                    self.assertNotIn(topic, session.keys)
            finally:
                node.close()
