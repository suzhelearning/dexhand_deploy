from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from tests.test_pico_hand_service import Session, wire_frame
from tianji_teleop.hand_tracking.pico_gestures import PicoGestureRuntime, TOPIC
from tianji_teleop.hand_tracking import pico_gesture_start


class PicoGestureStartTest(unittest.TestCase):
    def test_request_result_is_recorded_as_audit_not_motion(self):
        from tests import test_dual_input_recorder as fixture
        from tianji_teleop.recording.session_h5 import SessionH5Reader
        self.feed('fist')
        for _ in range(10):
            self.feed('open')
        topic, value = self.session.messages[-1]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            session = fixture.Session()
            node = fixture.DualInputRecorderTest().node(session, path, 'pico2_hands_sim', 'pico')
            try:
                self.assertIn(topic, session.keys)
                node.receive(topic, value, received_time_ns=self.now)
            finally:
                node.close()
            with SessionH5Reader(path) as reader:
                row = reader.read_dual_audit()[0]
                self.assertEqual(row['kind'], 'operator_result')
                self.assertEqual(row['payload'], value)
                self.assertEqual(reader.read_arm_command('left'), [])

    def test_managed_factory_cannot_enable_legacy_or_real_route(self):
        factory = pico_gesture_start.bind_from_environment
        self.assertIsNone(factory({}, session=None, node=None))
        for environment in ({'TIANJI_PICO_OPERATOR_INPUT': 'gesture'},
                {'TIANJI_PICO_OPERATOR_INPUT': 'gesture', 'TIANJI_REQUIRED_CAPABILITY': 'real'}):
            with self.assertRaises(ValueError):
                factory(environment, session=None, node=None)
        class Node:
            phase = 'armed'
            def request_start(self, **kwargs):
                return False
        env = dict(TIANJI_PICO_OPERATOR_INPUT='gesture', TIANJI_REQUIRED_CAPABILITY='simulation',
            TIANJI_REQUIRED_OBSERVATION_PROFILE='pico', TIANJI_ROUTER_ZID='router',
            TIANJI_COMPONENT_INSTANCE_ID='target', TIANJI_OBSERVATION_PUBLISHER_INSTANCE_ID='pico',
            TIANJI_RESOLVED_DUAL_SESSION=json.dumps(dict(profile='pico2_hands_sim',
                config=dict(operator_input='gesture', input_mode='pico2_hands'))))
        binding = factory(env, session=Session(), node=Node())
        binding.close()

    def setUp(self):
        self.session = Session()
        self.now = 1_000_000_000
        self.armed = True
        self.requests = 0
        self.forward = True
        def request():
            self.requests += 1
            return self.forward
        self.binding = pico_gesture_start.PicoGestureStartBinding(self.session,
            router_zid='router', publisher_instance_id='target', observation_instance_id='pico',
            connection_generation=3, request_start=request, armed=lambda: self.armed,
            clock=lambda: self.now)
        self.addCleanup(self.binding.close)
        messages = []
        runtime = PicoGestureRuntime(publish=lambda k, v: messages.append((k, v)),
            publisher_instance_id='pico', router_zid='router')
        runtime.ingest_pico(wire_frame())
        self.template = messages[-1][1]
        self.sequence = 0

    def feed(self, label, **changes):
        value = deepcopy(self.template)
        value.update(receiver_frame_sequence=self.sequence, received_timestamp_ns=self.now)
        for hand in value['hands'].values():
            hand['gesture'] = label
        value.update(changes)
        self.session.callbacks[TOPIC](value)
        self.sequence += 1
        self.now += 100_000_000

    def test_hold_at_start_does_not_authorize_until_release_and_stable_open(self):
        for _ in range(10):
            self.feed('open')
        self.assertEqual(self.requests, 0)
        self.feed('fist')
        for _ in range(8):
            self.feed('open')
        self.assertEqual(self.requests, 0)
        self.feed('open')
        self.assertEqual(self.requests, 1)
        for _ in range(10):
            self.feed('open')
        self.assertEqual(self.requests, 1)
        self.assertTrue(self.session.messages[-1][1]['request_forwarded'])

    def test_not_armed_and_rejected_request_never_queue_or_toggle_home(self):
        self.armed = False
        self.feed('fist')
        for _ in range(10):
            self.feed('open')
        self.armed = True
        self.feed('open')
        self.assertEqual(self.requests, 0)
        self.forward = False
        self.feed('fist')
        for _ in range(12):
            self.feed('open')
        self.assertEqual(self.requests, 1)
        self.assertFalse(self.session.messages[-1][1]['request_forwarded'])

    def test_foreign_generation_invalid_input_and_close_cannot_start(self):
        self.feed('fist')
        for _ in range(10):
            self.feed('open', connection_generation=4)
        self.assertEqual(self.requests, 0)
        self.feed('fist')
        self.session.callbacks[TOPIC]({'bad': 'packet'})
        for _ in range(10):
            self.feed('open')
        self.assertEqual(self.requests, 0)
        callback = self.session.callbacks[TOPIC]
        self.binding.close()
        callback(self.template)
        self.assertEqual(self.requests, 0)
        self.assertNotIn(TOPIC, self.session.callbacks)
