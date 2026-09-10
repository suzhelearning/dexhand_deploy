import base64
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from tianji_teleop.recording.recorder import SessionRecorderNode, RecorderProtocolError
from tianji_teleop.recording.session_h5 import SessionH5Reader
from tests.test_reference_tjvr_receiver import packet

TOPIC = 'tianji/raw/tjvr_upper_limb'


class Session:
    def __init__(self):
        self.keys = []

    def declare_subscriber(self, key, callback):
        self.keys.append(key)
        return SimpleNamespace(undeclare=lambda: None)


class DualInputRecorderTest(unittest.TestCase):
    def test_cli_loads_explicit_dual_recording_config(self):
        from tianji_teleop.recording.session_recorder import _load_recording_config
        config = Path(__file__).resolve().parents[1] / 'src/tianji_teleop/config/recording/dual_input.yaml'
        self.assertEqual(_load_recording_config(str(config))['schema_version'], '1.2')

    def payload(self):
        return dict(schema_version=1, kind='tjvr_upper_limb_raw', router_zid='router',
            raw_packet_base64=base64.b64encode(packet(1)).decode(),
            received_timestamp_ns=1000, receiver_instance_id='input', receiver_frame_sequence=1)

    def node(self, session, path, source='vr_manus_sim', profile='manus'):
        return SessionRecorderNode(session, path, source_type=source, input_profile=profile,
            robot_model='spark', router_zid='router', recording_config=dict(flush_interval_s=1.,
                schema_name='tianji-teleop-session', schema_version='1.2'))

    def test_vr_recorder_records_raw_before_gate_without_command_authority(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.h5'
            session = Session()
            node = self.node(session, path)
            try:
                self.assertIn(TOPIC, session.keys)
                node.receive(TOPIC, self.payload(), received_time_ns=2000)
            finally:
                node.close()
            with SessionH5Reader(path) as reader:
                self.assertEqual(reader.read_raw_reference_tjvr()[0]['received_timestamp_ns'], 1000)
                self.assertEqual(reader.read_arm_command('left'), [])

    def test_pico_mode_does_not_subscribe_or_accept_tjvr(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Session()
            node = self.node(session, Path(directory) / 'test.h5', 'pico2_hands_sim', 'pico')
            try:
                self.assertNotIn(TOPIC, session.keys)
                with self.assertRaises(RecorderProtocolError):
                    node.receive(TOPIC, self.payload())
            finally:
                node.close()
