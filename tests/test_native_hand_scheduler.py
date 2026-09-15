import math
import os
from pathlib import Path
import struct
import sys
import time
import unittest
from unittest.mock import patch
from dataclasses import replace


ROOT = Path(__file__).resolve().parents[1]


class NativeHandSchedulerContractTest(unittest.TestCase):
    def test_processed_only_teleop_output_is_rejected_in_async_mode(self):
        from tianji_teleop.producers.native_hand_scheduler import NativeHandSchedulerClient, _OUTPUT
        client = NativeHandSchedulerClient.__new__(NativeHandSchedulerClient)
        output = _OUTPUT.pack(b'TJHO', 1, 2, _OUTPUT.size, 1, 1, 100, 1, 101,
                              2, 1, 1, 0, *([0.] * 40))
        with self.assertRaisesRegex(RuntimeError, 'invalid'):
            client._result(output, 1, 100, expected_flags=2, expected_epoch=1,
                           expected_phase=1, allow_stale=True)

    def test_wire_sizes_and_explicit_backend_defaults(self):
        from tianji_teleop.producers.native_hand_scheduler import (
            _HANDSHAKE_SIZE, _INPUT_SIZE, _OUTPUT_SIZE, _SESSION_SIZE,
        )

        self.assertEqual(_INPUT_SIZE, 1048)
        self.assertEqual(_SESSION_SIZE, 40)
        self.assertEqual(_OUTPUT_SIZE, 376)
        self.assertEqual(_HANDSHAKE_SIZE, 16)
        self.assertEqual(struct.calcsize('<4sBBHQQQII126d'), 1048)
        self.assertEqual(struct.calcsize('<4sBBHQQQBBHI'), 40)
        self.assertEqual(struct.calcsize('<4sBBHQQQQQBBHI40d'), 376)

    def test_missing_scheduler_is_rejected_before_spawn(self):
        from tianji_teleop.producers.native_hand_scheduler import NativeHandSchedulerClient

        with self.assertRaisesRegex(RuntimeError, 'native hand scheduler'):
            NativeHandSchedulerClient(
                python=sys.executable,
                scheduler=ROOT / 'build/hand-native/does-not-exist',
            )

    def test_metadata_is_portable(self):
        import json

        from tianji_teleop.producers.native_hand_scheduler import recording_metadata

        with patch.dict(os.environ, {'TIANJI_HAND_SCHEDULER_BACKEND': 'python'}, clear=False):
            self.assertIsNone(recording_metadata(os.environ))
        if not (ROOT / 'build/hand-native/tianji_hand_native_scheduler').is_file():
            self.skipTest('run pixi run build-native-hand-scheduler')
        with patch.dict(os.environ, {'TIANJI_HAND_SCHEDULER_BACKEND': 'cpp'}, clear=False):
            metadata = recording_metadata(os.environ)
        self.assertEqual(metadata['backend'], 'cpp')
        self.assertEqual(metadata['protocol'], 'TJHS/TJHI/TJHO')
        self.assertNotIn('/home/', json.dumps(metadata, sort_keys=True))


@unittest.skipUnless(
    (ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python').is_file()
    and (ROOT / 'build/hand-native/tianji_hand_native_scheduler').is_file(),
    'requires the pinned Hand2 environment and native scheduler build',
)
class NativeHandSchedulerProcessTest(unittest.TestCase):
    def test_initial_zero_session_sequence_is_accepted_once(self):
        from tianji_teleop.protocol.messages import SessionState
        client = self._client(async_mode=True)
        state = SessionState(1, 0, time.monotonic_ns(), 'idle', 'test',
                             'coordinator', None, 'coord', 'router')
        self.assertTrue(client.update_session(state))
        self.assertFalse(client.update_session(state))
        client.submit_retarget(self._points(), sequence=1, timestamp_ns=time.monotonic_ns())
        self.assertIsNotNone(self._wait_result(client))

    @staticmethod
    def _points():
        points = [0.0, 0.0, 0.0]
        for finger in range(5):
            for joint in range(4):
                points.extend([0.02 * (finger - 2), 0.02 * (joint + 1), 0.002 * joint])
        return points

    def _client(self, side='right', *, async_mode=False):
        from tianji_teleop.producers.native_hand_scheduler import NativeHandSchedulerClient

        client = NativeHandSchedulerClient(
            python=ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
            single_hand_side=side, generation=7, startup_handshake=True,
            timeout_seconds=5.0, async_mode=async_mode,
        )
        # PicoOfficialHandBackend deliberately does not own client lifetime;
        # the production component closes them through its ExitStack. Keep
        # the process-based test equally explicit so bilateral clients cannot
        # survive the test case and emit subprocess/file-descriptor warnings.
        self.addCleanup(client.close)
        return client

    @staticmethod
    def _wait_result(client, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = client.poll_retarget()
            if result is not None:
                return result
            time.sleep(.005)
        raise AssertionError('native hand scheduler did not publish an async result')

    def test_async_submit_does_not_wait_for_solver_and_poll_associates_result(self):
        from tianji_teleop.protocol.messages import SessionState

        client = self._client(async_mode=True)
        stamp = time.monotonic_ns()
        self.assertTrue(client.update_session(
            SessionState(1, 1, stamp, 'teleop', 'test', 'coordinator', None,
                         'coord', 'router')))
        submitted = client.submit_retarget(self._points(), sequence=1, timestamp_ns=stamp)
        self.assertEqual(submitted, dict(sequence=1, timestamp_ns=stamp, valid_sides=['right']))
        result = self._wait_result(client)
        self.assertEqual(result['callback_sequence'], 1)
        self.assertEqual(result['native_scheduler']['status'], 2)
        self.assertEqual(client.poll_retarget(), None)

    def test_buffered_teleop_result_cannot_cross_rearm(self):
        from tianji_teleop.protocol.messages import SessionState
        client = self._client(async_mode=True)
        def session(sequence, state):
            return SessionState(1, sequence, time.monotonic_ns(), state, 'test',
                                'coordinator', None, 'coord', 'router')
        client.update_session(session(1, 'teleop'))
        client.submit_retarget(self._points(), sequence=1, timestamp_ns=time.monotonic_ns())
        deadline = time.monotonic() + 2
        while not client.async_diagnostics['queued_outputs'] and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertEqual(client.async_diagnostics['queued_outputs'], 1)
        client.update_session(session(2, 'returning'))
        client.update_session(session(3, 'idle'))
        client.update_session(session(4, 'teleop'))
        self.assertIsNone(client.poll_retarget())
        client.submit_retarget(self._points(), sequence=2, timestamp_ns=time.monotonic_ns())
        result = self._wait_result(client)
        self.assertEqual(result['callback_sequence'], 2)
        self.assertEqual(result['native_scheduler']['epoch'], 2)
        self.assertIsNone(client.async_diagnostics['failure'])

    def test_async_stale_input_returns_completion_without_valid_joints(self):
        from tianji_teleop.protocol.messages import SessionState
        client = self._client(async_mode=True)
        stamp = time.monotonic_ns()
        client.update_session(SessionState(1, 1, stamp, 'teleop', 'test',
                                          'coordinator', None, 'coord', 'router'))
        client.submit_retarget(self._points(), sequence=1, timestamp_ns=stamp - 1_000_000_000)
        row = self._wait_result(client, timeout=.5)
        self.assertEqual(row['native_scheduler']['status'], 3)
        self.assertFalse(row['left']['valid'])
        self.assertFalse(row['right']['valid'])

    def test_async_hung_worker_times_out_without_new_input(self):
        import signal
        client = self._client(async_mode=True)
        client._timeout = .1
        os.kill(client._process.pid, signal.SIGSTOP)
        try:
            client.submit_retarget(self._points(), sequence=1, timestamp_ns=time.monotonic_ns())
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                self._wait_result(client, timeout=1)
        finally:
            os.kill(client._process.pid, signal.SIGCONT)

    def test_async_latest_poll_coalesces_outputs_without_blocking_input_submission(self):
        from tianji_teleop.protocol.messages import SessionState

        client = self._client(async_mode=True)
        stamp = time.monotonic_ns()
        self.assertTrue(client.update_session(
            SessionState(1, 1, stamp, 'teleop', 'test', 'coordinator', None,
                         'coord', 'router')))
        for sequence in range(1, 17):
            client.submit_retarget(self._points(), sequence=sequence,
                                   timestamp_ns=time.monotonic_ns())
        result = self._wait_result(client)
        self.assertLessEqual(result['callback_sequence'], 16)
        last = result['callback_sequence']
        deadline = time.monotonic() + 5.0
        while last < 16 and time.monotonic() < deadline:
            newer = client.poll_retarget()
            if newer is not None:
                self.assertGreater(newer['callback_sequence'], last)
                last = newer['callback_sequence']
            else:
                time.sleep(.005)
        self.assertEqual(last, 16)

    def test_idle_and_teleop_outputs_are_associated_and_fixed_rate_owned(self):
        from tianji_teleop.protocol.messages import SessionState

        client = self._client()
        base = time.monotonic_ns()
        self.assertTrue(client.update_session(
            SessionState(1, 1, base, 'idle', 'test', 'coordinator', None, 'coord', 'router')))
        idle = client.retarget(self._points(), sequence=1, timestamp_ns=base)
        self.assertEqual(idle['callback_sequence'], 1)
        self.assertEqual(idle['native_scheduler']['phase'], 0)
        self.assertEqual(idle['native_scheduler']['status'], 1)
        self.assertFalse(idle['left']['valid'])
        self.assertTrue(idle['right']['valid'])
        from tianji_teleop.protocol.messages import HAND_JOINT_NAMES
        self.assertEqual(idle['right']['joint_names'], list(HAND_JOINT_NAMES['right']))

        teleop_stamp = time.monotonic_ns()
        self.assertTrue(client.update_session(
            SessionState(1, 2, teleop_stamp, 'teleop', 'test', 'coordinator', None,
                         'coord', 'router')))
        teleop = client.retarget(self._points(), sequence=2,
                                 timestamp_ns=time.monotonic_ns())
        self.assertEqual(teleop['callback_sequence'], 2)
        self.assertEqual(teleop['native_scheduler']['phase'], 1)
        self.assertEqual(teleop['native_scheduler']['status'], 2)
        self.assertEqual(teleop['right']['joint_names'], list(HAND_JOINT_NAMES['right']))
        self.assertTrue(all(math.isfinite(value) for value in teleop['right']['position_rad']))
        self.assertTrue(all(isinstance(value, float) for value in teleop['right']['position_rad']))

    def test_bilateral_input_is_solved_by_both_native_side_pipelines(self):
        from tianji_teleop.protocol.messages import SessionState

        client = self._client()
        stamp = time.monotonic_ns()
        self.assertTrue(client.update_session(
            SessionState(1, 1, stamp, 'teleop', 'test', 'coordinator', None,
                         'coord', 'router')))
        result = client.retarget(self._points() + self._points(), sequence=1,
                                 timestamp_ns=time.monotonic_ns())
        self.assertTrue(result['left']['valid'])
        self.assertTrue(result['right']['valid'])
        self.assertEqual(len(result['left']['position_rad']), 20)
        self.assertEqual(len(result['right']['position_rad']), 20)

    def test_pico_official_backend_accepts_native_canonical_joint_names(self):
        from tests import test_official_pico_input as pico_fixture
        from tianji_teleop.producers.pico_official_hand import (
            PicoOfficialHandBackend, PicoOfficialHandProducer)
        from tianji_teleop.protocol.messages import HAND_JOINT_NAMES, SessionState

        clients = {side: self._client(side) for side in ('left', 'right')}
        backend = PicoOfficialHandBackend(
            clients, receiver_instance_id='pico', connection_generation=7)
        producer = PicoOfficialHandProducer(
            backend, publisher_instance_id='hand', router_zid='router',
            coordinator_instance_id='coord', receiver_instance_id='pico',
            freshness_ns=200_000_000)
        self.addCleanup(backend.close)
        stamp = time.monotonic_ns()
        self.assertTrue(producer.update_session(
            SessionState(1, 1, stamp, 'teleop', 'test', 'coordinator', None,
                         'coord', 'router')))
        frame = replace(pico_fixture.OfficialPicoInputTest().frame(),
                        receiver_instance_id='pico', connection_generation=7,
                        receiver_frame_sequence=1, received_timestamp_ns=stamp)
        self.assertTrue(producer.update_frame(frame, now_ns=stamp + 1_000_000))
        commands = producer.commands(stamp + 1_000_000)
        self.assertEqual(set(commands), {'left', 'right'})
        for side, command in commands.items():
            self.assertEqual(command.names, list(HAND_JOINT_NAMES[side]))

    def test_pico_official_async_backend_waits_for_a_matched_bilateral_result(self):
        from tests import test_official_pico_input as pico_fixture
        from tianji_teleop.producers.pico_official_hand import (
            PicoOfficialHandBackend, PicoOfficialHandProducer)
        from tianji_teleop.protocol.messages import HAND_JOINT_NAMES, SessionState

        clients = {side: self._client(side, async_mode=True) for side in ('left', 'right')}
        backend = PicoOfficialHandBackend(
            clients, receiver_instance_id='pico', connection_generation=3)
        producer = PicoOfficialHandProducer(
            backend, publisher_instance_id='hand', router_zid='router',
            coordinator_instance_id='coord', receiver_instance_id='pico',
            freshness_ns=200_000_000)
        self.addCleanup(backend.close)
        stamp = time.monotonic_ns()
        self.assertTrue(producer.update_session(
            SessionState(1, 1, stamp, 'teleop', 'test', 'coordinator', None,
                         'coord', 'router')))
        frame = replace(pico_fixture.OfficialPicoInputTest().frame(),
                        receiver_frame_sequence=1, received_timestamp_ns=stamp,
                        receiver_instance_id='pico', connection_generation=3)
        self.assertTrue(producer.update_input(
            frame, sequence=2, timestamp_ns=stamp,
            receiver_instance_id='pico', now_ns=stamp + 1_000_000))
        self.assertEqual(producer.commands(stamp + 1_000_000), {})
        deadline = time.monotonic() + 5.0
        commands = {}
        while time.monotonic() < deadline and set(commands) != {'left', 'right'}:
            commands = producer.commands(time.monotonic_ns())
            if set(commands) != {'left', 'right'}:
                time.sleep(.005)
        self.assertEqual(set(commands), {'left', 'right'})
        for side, command in commands.items():
            self.assertEqual(command.names, list(HAND_JOINT_NAMES[side]))
            self.assertEqual(command.sequence, 2)

    def test_async_result_from_previous_session_cannot_be_replayed_after_rearm(self):
        from tianji_teleop.producers.hand_retarget import HandRetargetProducer
        from tianji_teleop.protocol.messages import SessionState

        client = self._client(async_mode=True)
        producer = HandRetargetProducer(
            client, publisher_instance_id='hand', router_zid='router',
            coordinator_instance_id='coord', receiver_instance_id='manus',
            freshness_ns=200_000_000)

        def state(phase, sequence):
            return SessionState(1, sequence, time.monotonic_ns(), phase, 'test',
                                'coordinator', None, 'coord', 'router')

        self.assertTrue(producer.update_session(state('teleop', 1)))
        stamp = time.monotonic_ns()
        self.assertTrue(producer.update_input(
            self._points() + self._points(), sequence=1, timestamp_ns=stamp,
            receiver_instance_id='manus', now_ns=stamp + 1_000_000))
        self.assertTrue(producer.update_session(state('returning', 2)))
        self.assertEqual(producer.commands(time.monotonic_ns()), {})
        self.assertTrue(producer.update_session(state('idle', 3)))
        self.assertEqual(producer.commands(time.monotonic_ns()), {})
        self.assertTrue(producer.update_session(state('teleop', 4)))
        deadline = time.monotonic() + .2
        while time.monotonic() < deadline:
            self.assertEqual(producer.commands(time.monotonic_ns()), {})
            time.sleep(.005)

    def test_manus_producer_accepts_native_bilateral_scheduler_commands(self):
        from tianji_teleop.producers.hand_retarget import HandRetargetProducer
        from tianji_teleop.protocol.messages import HAND_JOINT_NAMES, SessionState

        client = self._client()
        producer = HandRetargetProducer(
            client, publisher_instance_id='hand', router_zid='router',
            coordinator_instance_id='coord', receiver_instance_id='manus',
            freshness_ns=200_000_000)
        stamp = time.monotonic_ns()
        self.assertTrue(producer.update_session(
            SessionState(1, 1, stamp, 'teleop', 'test', 'coordinator', None,
                         'coord', 'router')))
        self.assertTrue(producer.update_input(
            self._points() + self._points(), sequence=1,
            timestamp_ns=stamp, receiver_instance_id='manus',
            now_ns=stamp + 1_000_000))
        commands = producer.commands(stamp + 1_000_000)
        self.assertEqual(set(commands), {'left', 'right'})
        for side, command in commands.items():
            self.assertEqual(command.names, list(HAND_JOINT_NAMES[side]))
            self.assertEqual(command.sequence, 1)

    def test_stale_native_teleop_result_fails_closed(self):
        from tianji_teleop.producers.native_hand_scheduler import NativeHandSchedulerClient
        from tianji_teleop.protocol.messages import SessionState

        client = NativeHandSchedulerClient(
            python=ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
            single_hand_side='right', generation=7, startup_handshake=True,
            freshness_ns=1, timeout_seconds=5.0)
        self.addCleanup(client.close)
        stamp = time.monotonic_ns()
        self.assertTrue(client.update_session(
            SessionState(1, 1, stamp, 'teleop', 'test', 'coordinator', None,
                         'coord', 'router')))
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            client.retarget(self._points(), sequence=1,
                            timestamp_ns=time.monotonic_ns() - 1_000_000)

    def test_native_result_cannot_cross_the_session_phase_boundary(self):
        from tianji_teleop.producers.native_hand_scheduler import (
            NativeHandSchedulerClient, _OUTPUT, _OUTPUT_SIZE,
        )
        from tianji_teleop.protocol.messages import SessionState

        client = self._client()
        stamp = time.monotonic_ns()
        self.assertTrue(client.update_session(
            SessionState(1, 1, stamp, 'teleop', 'test', 'coordinator', None,
                         'coord', 'router')))
        # A result associated with the right input but produced before the
        # teleop heartbeat must not be promoted by the Python command gate.
        output = _OUTPUT.pack(
            b'TJHO', 1, 2, _OUTPUT_SIZE, 1, 1, stamp, 1,
            time.monotonic_ns(), 2, 0, 1, 0, *([0.0] * 40))
        with self.assertRaisesRegex(RuntimeError, 'phase'):
            client._result(output, 1, stamp, expected_flags=2)

    def test_generation_is_admitted_and_epoch_update_resets_native_pipeline(self):
        from tianji_teleop.protocol.messages import SessionState

        client = self._client()
        base = time.monotonic_ns()
        self.assertTrue(client.update_session(
            SessionState(1, 1, base, 'idle', 'test', 'coordinator', None, 'coord', 'router')))
        client.retarget(self._points(), sequence=1, timestamp_ns=base)
        client.set_execution_epoch(2)
        reset_stamp = time.monotonic_ns()
        self.assertTrue(client.update_session(
            SessionState(1, 2, reset_stamp, 'idle', 'rearm', 'coordinator', None, 'coord', 'router')))
        result = client.retarget(self._points(), sequence=2,
                                 timestamp_ns=time.monotonic_ns())
        self.assertEqual(result['native_scheduler']['epoch'], 2)
        self.assertEqual(result['native_scheduler']['phase'], 0)

    def test_return_to_idle_advances_epoch_once_for_direct_native_client(self):
        from tianji_teleop.protocol.messages import SessionState

        client = self._client()

        def state(phase, sequence):
            return SessionState(1, sequence, time.monotonic_ns(), phase, 'test',
                                'coordinator', None, 'coord', 'router')

        self.assertTrue(client.update_session(state('idle', 1)))
        client.retarget(self._points(), sequence=1, timestamp_ns=time.monotonic_ns())
        self.assertTrue(client.update_session(state('teleop', 2)))
        client.retarget(self._points(), sequence=2, timestamp_ns=time.monotonic_ns())
        self.assertTrue(client.update_session(state('returning', 3)))
        client.retarget(self._points(), sequence=3, timestamp_ns=time.monotonic_ns())
        self.assertTrue(client.update_session(state('idle', 4)))
        result = client.retarget(self._points(), sequence=4, timestamp_ns=time.monotonic_ns())
        self.assertEqual(result['native_scheduler']['epoch'], 2)
        self.assertEqual(result['native_scheduler']['phase'], 0)
        self.assertTrue(client.update_session(state('idle', 5)))
        self.assertTrue(client.update_session(state('teleop', 6)))
        result = client.retarget(self._points(), sequence=5, timestamp_ns=time.monotonic_ns())
        self.assertEqual(result['native_scheduler']['epoch'], 2)
        self.assertEqual(result['native_scheduler']['phase'], 1)


if __name__ == '__main__':
    unittest.main()
