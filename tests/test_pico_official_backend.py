import importlib.util
import os
from dataclasses import replace
from pathlib import Path
from threading import Barrier
import unittest

import numpy as np

from tests import test_official_pico_input as pico_fixture
from tianji_teleop.protocol.messages import HAND_JOINT_NAMES


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

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional official Hand2 environment')
    def test_per_side_workers_match_existing_official_bilateral_worker(self):
        from tianji_teleop.producers.hand_retarget import OfficialHandClient
        from tianji_teleop.hand_tracking.official_pico import pico_official_hand_observations
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
            expected = reference.retarget(np.concatenate([rows[side].keypoints_m.ravel()
                for side in ('right', 'left') if rows[side].valid]).tolist(),
                sequence=8+index, timestamp_ns=1000+index)
            actual = backend.retarget(frame)
            for side in ('left', 'right'):
                self.assertEqual(actual[side], expected[side])
