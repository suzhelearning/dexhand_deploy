from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from tests.test_pico_hand_service import wire_frame
from tests import test_dual_input_recorder as recorder_fixture
from tianji_teleop.hand_tracking import pico_gestures
from tianji_teleop.recording.session_h5 import SessionH5Reader


class PicoGestureRuntimeTest(unittest.TestCase):
    def test_cli_is_explicit_and_default_runtime_unchanged(self):
        from tianji_teleop.hand_tracking import observation_node
        from tianji_teleop.hand_tracking.runtime import ObservationRuntime
        parser = observation_node._parser()
        args = parser.parse_args(['--config', 'unused'])
        self.assertIs(observation_node._runtime_class(args, {'input_profile': 'pico'}), ObservationRuntime)
        args = parser.parse_args(['--config', 'unused', '--gesture-observations'])
        self.assertIs(observation_node._runtime_class(args, {'input_profile': 'pico'}), pico_gestures.PicoGestureRuntime)
        with self.assertRaises(ValueError):
            observation_node._runtime_class(args, {'input_profile': 'manus'})

    def test_observation_only_preserves_raw_and_canonical_topics(self):
        messages = []
        runtime = pico_gestures.PicoGestureRuntime(publish=lambda k, v: messages.append((k, v)),
            publisher_instance_id='pico', router_zid='router')
        runtime.ingest_pico(wire_frame())
        rows = [v for k, v in messages if k == pico_gestures.TOPIC]
        self.assertEqual(len(rows), 1)
        value = pico_gestures.validate_observation(rows[0])
        self.assertEqual(value['connection_generation'], 3)
        self.assertEqual(value['receiver_frame_sequence'], 7)
        self.assertEqual(set(value['hands']), {'left', 'right'})
        self.assertFalse(any('/command/' in k or '/request/' in k for k, _ in messages))
        self.assertEqual(len(messages), 6)  # raw + two hands + two arms + gesture observation
        frame = wire_frame()
        runtime.ingest_pico(replace(frame, hands={side: replace(hand, valid=False)
            for side, hand in frame.hands.items()}))
        self.assertFalse(messages[-1][1]['hands']['left']['available'])

    def test_pico_schema12_records_gesture_observation_without_commands(self):
        messages = []
        runtime = pico_gestures.PicoGestureRuntime(publish=lambda k, v: messages.append((k, v)),
            publisher_instance_id='pico', router_zid='router')
        runtime.ingest_pico(wire_frame())
        value = messages[-1][1]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            session = recorder_fixture.Session()
            node = recorder_fixture.DualInputRecorderTest().node(session, path, 'pico2_hands_sim', 'pico')
            try:
                self.assertIn(pico_gestures.TOPIC, session.keys)
                node.receive(pico_gestures.TOPIC, value, received_time_ns=2000)
            finally:
                node.close()
            with SessionH5Reader(path) as reader:
                rows = reader.read_dual_audit()
                self.assertEqual(rows[0]['kind'], 'operator_observation')
                self.assertEqual(rows[0]['payload'], value)
                self.assertEqual(reader.read_arm_command('left'), [])
