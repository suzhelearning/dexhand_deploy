import importlib.util
import os
from dataclasses import replace
from pathlib import Path
from threading import Barrier
import unittest

import numpy as np

from tests import test_official_pico_input as pico_fixture
from tianji_teleop.protocol.messages import HAND_JOINT_NAMES, SessionState


class SideClient:
    def __init__(self, side):
        self.side, self.calls = side, []

    def retarget(self, points, *, sequence, timestamp_ns):
        self.calls.append((points, sequence, timestamp_ns))
        return dict(callback_sequence=sequence, timestamp_ns=timestamp_ns,
            **{side: dict(valid=side == self.side, joint_names=list(HAND_JOINT_NAMES[side]),
                          position_rad=[.1 if side == self.side else 0.] * 20)
               for side in ('left', 'right')})


class PicoOfficialBackendTest(unittest.TestCase):
    def async_clients(self):
        class DeferredSide(SideClient):
            async_retarget = True
            def __init__(self, side):
                super().__init__(side)
                self.pending = []
                self.ready = False
            def submit_retarget(self, points, **metadata):
                self.pending.append((points, metadata))
            def poll_retarget(self):
                if not self.ready or not self.pending:
                    return None
                points, metadata = self.pending.pop(0)
                row = self.retarget(points, **metadata)
                row['native_scheduler'] = dict(phase=0, status=1, epoch=1)
                return row
            def update_session(self, state):
                return True
            def set_execution_epoch(self, epoch):
                pass
        return {side: DeferredSide(side) for side in ('left', 'right')}

    def test_async_burst_keeps_inflight_pair_and_coalesces_waiting_frames(self):
        clients = self.async_clients()
        backend = self.make(clients)
        frame = self.frame()
        for index in range(3):
            backend.submit_retarget(replace(frame, receiver_frame_sequence=7 + index,
                                            received_timestamp_ns=1000 + index))
        for client in clients.values():
            self.assertEqual(len(client.pending), 1)
            client.ready = True
        result = backend.poll_retarget()
        self.assertEqual(result['callback_sequence'], 8)
        result = backend.poll_retarget()
        self.assertEqual(result['callback_sequence'], 10)
        self.assertIsNone(backend.poll_retarget())

    def test_idle_heartbeat_preserves_pending_async_readiness_result(self):
        clients = self.async_clients()
        backend = self.make(clients)
        state = SessionState(1, 1, 1000, 'idle', 'test', 'coordinator', None, 'coord', 'router')
        backend.update_session(state)
        backend.submit_retarget(self.frame())
        backend.update_session(replace(state, sequence=2, timestamp_ns=1001))
        for client in clients.values():
            client.ready = True
        self.assertIsNotNone(backend.poll_retarget())

    def test_explicit_epoch_change_discards_pending_pair_even_in_idle(self):
        clients = self.async_clients()
        backend = self.make(clients)
        state = SessionState(1, 1, 1000, 'idle', 'test', 'coordinator', None, 'coord', 'router')
        backend.update_session(state)
        backend.submit_retarget(self.frame())
        backend.set_execution_epoch(2)
        backend.update_session(replace(state, sequence=2, timestamp_ns=1001))
        for client in clients.values():
            client.ready = True
        self.assertIsNone(backend.poll_retarget())

    def test_async_pair_rejects_inconsistent_execution_metadata(self):
        clients = self.async_clients()
        backend = self.make(clients)
        backend.submit_retarget(self.frame())
        for client in clients.values():
            client.ready = True
        original = clients['right'].poll_retarget
        def wrong_epoch():
            row = original()
            row['native_scheduler']['epoch'] = 2
            return row
        clients['right'].poll_retarget = wrong_epoch
        with self.assertRaisesRegex(RuntimeError, 'scheduler'):
            backend.poll_retarget()

    def test_stale_side_completes_pair_and_allows_next_frame(self):
        clients = self.async_clients()
        backend = self.make(clients)
        backend.submit_retarget(self.frame())
        backend.submit_retarget(replace(self.frame(), receiver_frame_sequence=8,
                                        received_timestamp_ns=1001))
        for client in clients.values():
            client.ready = True
        original = clients['right'].poll_retarget
        def stale():
            row = original()
            if row and row['callback_sequence'] == 8:
                row['native_scheduler']['status'] = 3
                row['right']['valid'] = False
            return row
        clients['right'].poll_retarget = stale
        row = backend.poll_retarget()
        self.assertFalse(row['left']['valid'])
        self.assertFalse(row['right']['valid'])
        row = backend.poll_retarget()
        self.assertEqual(row['callback_sequence'], 9)
        self.assertTrue(row['left']['valid'])
        self.assertTrue(row['right']['valid'])

    def frame(self):
        return pico_fixture.OfficialPicoInputTest().frame()

    def make(self, clients):
        name = 'tianji_teleop.producers.pico_official_hand'
        self.assertIsNotNone(importlib.util.find_spec(name))
        from tianji_teleop.producers.pico_official_hand import PicoOfficialHandBackend
        backend = PicoOfficialHandBackend(clients, receiver_instance_id='pico', connection_generation=3)
        self.addCleanup(backend.close)
        return backend

    def test_invalid_side_is_not_solved_and_recovery_preserves_sequence_gap(self):
        clients = {side: SideClient(side) for side in ('left', 'right')}
        backend = self.make(clients)
        frame = self.frame()
        first = backend.retarget(frame)
        self.assertEqual(first['frame_association_id'], frame.association_id)
        self.assertTrue(first['left']['valid'])
        lost = replace(frame, receiver_frame_sequence=8, received_timestamp_ns=1001,
                       hands=dict(frame.hands, left=replace(frame.hands['left'], valid=False)))
        row = backend.retarget(lost)
        self.assertFalse(row['left']['valid'])
        self.assertTrue(row['right']['valid'])
        backend.retarget(replace(frame, receiver_frame_sequence=9, received_timestamp_ns=1002))
        self.assertEqual([call[1] for call in clients['left'].calls], [8, 10])
        self.assertEqual([call[1] for call in clients['right'].calls], [8, 9, 10])
        self.assertTrue(all(len(call[0]) == 63 for client in clients.values() for call in client.calls))

    def test_pico_geometry_scale_is_only_applied_at_official_worker_boundary(self):
        clients = {side: SideClient(side) for side in ('left', 'right')}
        backend = self.make(clients)
        frame = self.frame()

        from tianji_teleop.hand_tracking.official_pico import (
            pico_official_hand2_retarget_input,
            pico_official_hand_observations,
        )
        observations = pico_official_hand_observations(frame)
        backend.retarget(frame)

        for side in ('left', 'right'):
            submitted = np.asarray(clients[side].calls[0][0], dtype=np.float64).reshape(21, 3)
            expected = pico_official_hand2_retarget_input(observations[side].keypoints_m)
            np.testing.assert_allclose(submitted, expected, atol=0, rtol=0)

    def test_independent_official_workers_are_called_concurrently(self):
        barrier = Barrier(2)

        class ConcurrentSideClient(SideClient):
            def retarget(self, points, *, sequence, timestamp_ns):
                barrier.wait(timeout=1.0)
                return super().retarget(points, sequence=sequence, timestamp_ns=timestamp_ns)

        clients = {side: ConcurrentSideClient(side) for side in ('left', 'right')}
        backend = self.make(clients)
        result = backend.retarget(self.frame())
        self.assertTrue(result['left']['valid'])
        self.assertTrue(result['right']['valid'])

    def test_reconnection_duplicate_and_foreign_source_never_reuse_filter_state(self):
        clients = {side: SideClient(side) for side in ('left', 'right')}
        backend = self.make(clients)
        frame = self.frame()
        backend.retarget(frame)
        for invalid in (frame, replace(frame, connection_generation=4),
                        replace(frame, receiver_instance_id='foreign')):
            with self.assertRaises(ValueError):
                backend.retarget(invalid)
        self.assertEqual(len(clients['left'].calls), 1)

    def test_wrong_side_worker_binding_latches_failure(self):
        clients = {side: SideClient('right') for side in ('left', 'right')}
        backend = self.make(clients)
        with self.assertRaisesRegex(RuntimeError, 'side'):
            backend.retarget(self.frame())
        with self.assertRaisesRegex(RuntimeError, 'failed'):
            backend.retarget(replace(self.frame(), receiver_frame_sequence=8))
        self.assertEqual(len(clients['left'].calls), 1)

    def test_native_scheduler_epoch_advances_once_after_return_to_idle(self):
        class NativeSideClient(SideClient):
            def __init__(self, side):
                super().__init__(side)
                self.epochs = []
                self.sessions = []

            def set_execution_epoch(self, epoch):
                self.epochs.append(epoch)

            def update_session(self, state):
                self.sessions.append(state.state)
                return True

        clients = {side: NativeSideClient(side) for side in ('left', 'right')}
        backend = self.make(clients)

        def state(mode, sequence):
            return SessionState(1, sequence, 1000 + sequence, mode, 'test',
                                'coordinator', None, 'coord', 'router')

        self.assertTrue(backend.update_session(state('idle', 1)))
        self.assertTrue(backend.update_session(state('teleop', 2)))
        self.assertTrue(backend.update_session(state('returning', 3)))
        self.assertTrue(backend.update_session(state('idle', 4)))
        self.assertTrue(backend.update_session(state('idle', 5)))
        self.assertTrue(backend.update_session(state('teleop', 6)))
        self.assertEqual(clients['left'].epochs, [2])
        self.assertEqual(clients['right'].epochs, [2])
        self.assertEqual(clients['left'].sessions,
                         ['idle', 'teleop', 'returning', 'idle', 'idle', 'teleop'])

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional official Hand2 environment')
    def test_per_side_workers_match_existing_official_bilateral_worker(self):
        from tianji_teleop.producers.hand_retarget import OfficialHandClient
        from tianji_teleop.hand_tracking.official_pico import (
            pico_official_hand_observations, pico_official_hand2_retarget_input)
        root = Path(__file__).resolve().parents[1]
        def client(side):
            value = OfficialHandClient(python=root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
                script=root / 'scripts/wuji_hand_worker.py', single_hand_side=side, startup_handshake=True)
            self.addCleanup(value.close)
            return value
        backend = self.make({side: client(side) for side in ('left', 'right')})
        reference = client('right')
        for index in range(3):
            frame = replace(self.frame(), receiver_frame_sequence=7+index, received_timestamp_ns=1000+index)
            if index == 1:
                frame = replace(frame, hands=dict(frame.hands,
                    left=replace(frame.hands['left'], valid=False)))
            rows = pico_official_hand_observations(frame)
            expected = reference.retarget(np.concatenate([pico_official_hand2_retarget_input(
                rows[side].keypoints_m).ravel()
                for side in ('right', 'left') if rows[side].valid]).tolist(),
                sequence=8+index, timestamp_ns=1000+index)
            actual = backend.retarget(frame)
            for side in ('left', 'right'):
                self.assertEqual(actual[side], expected[side])
