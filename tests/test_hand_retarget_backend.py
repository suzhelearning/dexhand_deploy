import importlib.util
import os
from pathlib import Path
import unittest
import subprocess
import sys
from unittest.mock import patch

from tianji_teleop.protocol.messages import HAND_JOINT_NAMES


class HandRetargetResultTest(unittest.TestCase):
    def module(self):
        name = 'tianji_teleop.producers.hand_retarget'
        self.assertIsNotNone(importlib.util.find_spec(name))
        from tianji_teleop.producers import hand_retarget
        return hand_retarget

    def row(self):
        row = dict(schema_version=1, kind='wuji_hand_result', algorithm='official_wuji_hand2',
                   callback_sequence=7, timestamp_ns=100)
        for side in ('left', 'right'):
            names = [name.replace('_index_', '_index_finger_').replace('_middle_', '_middle_finger_')
                     .replace('_ring_', '_ring_finger_') for name in HAND_JOINT_NAMES[side]]
            row[side] = dict(valid=True, joint_names=names, position_rad=[i / 100 for i in range(20)])
        return row

    def test_names_only_translation_no_second_retarget_or_clamp(self):
        result = self.module().validate_result(self.row(), sequence=7, timestamp_ns=100)
        for side in ('left', 'right'):
            self.assertEqual(result[side]['joint_names'], list(HAND_JOINT_NAMES[side]))
            self.assertEqual(result[side]['position_rad'], self.row()[side]['position_rad'])

    def test_latest_sampling_preserves_filter_continuity_and_raw_identity(self):
        import json
        module = self.module()
        seen = []
        def exchange(client, request):
            row = json.loads(request)
            seen.append(row['callback_sequence'])
            result = self.row()
            result.update(callback_sequence=row['callback_sequence'], timestamp_ns=row['timestamp_ns'])
            return result
        with patch.object(module.OfficialHandClient, '_exchange', exchange):
            client = module.OfficialHandClient(python=sys.executable, script=__file__,
                                               filter_continuity_ns=200_000_000)
            self.addCleanup(client.close)
            for seq, stamp in [(3, 1), (10, 20_000_001), (20, 300_000_001), (25, 320_000_001)]:
                result = client.retarget([0.] * 126, sequence=seq, timestamp_ns=stamp)
                self.assertEqual(result['callback_sequence'], seq)
            self.assertEqual(seen, [1, 2, 4, 5])
            with self.assertRaises(ValueError):
                client.retarget([0.] * 126, sequence=24, timestamp_ns=330_000_001)

    def test_wrong_sequence_nonfinite_wrong_side_and_partial_frame_rejected(self):
        for field in ('sequence', 'nan', 'side', 'partial'):
            row = self.row()
            if field == 'sequence': row['callback_sequence'] = 8
            if field == 'nan': row['right']['position_rad'][3] = float('nan')
            if field == 'side': row['right']['joint_names'] = row['left']['joint_names']
            if field == 'partial': del row['right']
            with self.assertRaises(ValueError):
                self.module().validate_result(row, sequence=7, timestamp_ns=100)

    def test_hand_worker_does_not_inherit_main_runtime_import_or_library_paths(self):
        module = self.module()
        real_popen = subprocess.Popen
        def spawn(*args, **kwargs):
            environment = kwargs.get('env', os.environ)
            for key in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD'):
                self.assertFalse(key in environment, f'inherited worker override: {key}')
            return real_popen([sys.executable, '-c', 'import sys; sys.stdin.readline()'], **kwargs)
        with patch.dict(os.environ, {key: '/unrelated-test-environment' for key in
                                     ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD')}):
            with patch.object(module.subprocess, 'Popen', side_effect=spawn):
                client = module.OfficialHandClient(python=sys.executable, script=__file__)
                client.close()

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional pinned official hand process')
    def test_sampled_inputs_match_continuous_official_filter_and_gap_reset(self):
        import numpy as np
        root = Path(__file__).resolve().parents[1]
        clients = []
        for options in ({'filter_continuity_ns': 200_000_000}, {}):
            client = self.module().OfficialHandClient(
                python=root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
                script=root / 'scripts/wuji_hand_worker.py', startup_handshake=True, **options)
            self.addCleanup(client.close)
            clients.append(client)
        for raw, internal, stamp, bend in [(3, 1, 1, 0.), (10, 2, 20_000_001, .01),
                                          (20, 4, 300_000_001, .02), (25, 5, 320_000_001, .005)]:
            points = [0., 0., 0.]
            for finger in range(5):
                for joint in range(4):
                    points.extend([.02 * (finger - 2), .02 * (joint + 1), bend * joint])
            actual = clients[0].retarget(points + points, sequence=raw, timestamp_ns=stamp)
            expected = clients[1].retarget(points + points, sequence=internal, timestamp_ns=stamp)
            self.assertEqual(actual['callback_sequence'], raw)
            for side in ('left', 'right'):
                np.testing.assert_allclose(actual[side]['position_rad'], expected[side]['position_rad'], atol=1e-10)

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional pinned official hand process')
    def test_startup_handshake_does_not_consume_a_callback(self):
        module = self.module()
        root = Path(__file__).resolve().parents[1]
        client = module.OfficialHandClient(
            python=root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
            script=root / 'scripts/wuji_hand_worker.py', startup_handshake=True)
        self.addCleanup(client.close)
        self.assertTrue(client.startup_ready)
        points = [0., 0., 0.]
        for finger in range(5):
            for joint in range(4):
                points.extend([.02 * (finger - 2), .02 * (joint + 1), .002 * joint])
        result = client.retarget(points + points, sequence=1, timestamp_ns=100)
        self.assertEqual(result['callback_sequence'], 1)

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional pinned official hand process')
    def test_persistent_client_preserves_callback_sequence_and_rejects_rollback(self):
        module = self.module()
        self.assertTrue(hasattr(module, 'OfficialHandClient'))
        root = Path(__file__).resolve().parents[1]
        client = module.OfficialHandClient(
            python=root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
            script=root / 'scripts/wuji_hand_worker.py')
        self.addCleanup(client.close)
        # A finite synthetic open hand, explicitly not a hardware recording.
        points = [0., 0., 0.]
        for finger in range(5):
            for joint in range(4):
                points.extend([.02 * (finger - 2), .02 * (joint + 1), .002 * joint])
        for seq in (1, 3):
            row = client.retarget(points + points, sequence=seq, timestamp_ns=100 + seq)
            self.assertEqual(row['callback_sequence'], seq)
            self.assertTrue(row['left']['valid'])
            self.assertTrue(row['right']['valid'])
        with self.assertRaises(ValueError):
            client.retarget(points + points, sequence=2, timestamp_ns=105)
