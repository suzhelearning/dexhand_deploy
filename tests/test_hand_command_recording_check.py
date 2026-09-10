from pathlib import Path
import tempfile
import unittest
import subprocess
import sys
import json
from unittest.mock import patch

import h5py

from tests import test_manus_recording_check as fixture
from tianji_teleop.protocol.messages import HandJointCommand, HAND_JOINT_NAMES

ROOT = Path(__file__).resolve().parents[1]


class Backend:
    def __init__(self):
        self.sequences = []
        self.closed = False

    def retarget(self, points, *, sequence, timestamp_ns):
        self.sequences.append(sequence)
        return {side: dict(valid=True, joint_names=list(HAND_JOINT_NAMES[side]),
                           position_rad=[sequence * .01] * 20) for side in ('left', 'right')}

    def close(self):
        self.closed = True


class HandCommandRecordingCheckTest(unittest.TestCase):
    def setUp(self):
        # Pure replay tests must not require the optional native environment.
        assets = patch('tianji_teleop.recording.hand_command_check.hand_replay_asset_hashes',
                       return_value={'/test-only/pinned-hand-asset': 'a' * 64})
        assets.start()
        self.addCleanup(assets.stop)

    def recording(self, path):
        from tianji_teleop.recording.hand_command_check import hand_replay_asset_hashes
        def command(sequence, now, capture):
            if sequence == 1:
                return  # Idle callback must still advance the retarget state.
            capture.hand_output({side: HandJointCommand(1, sequence, now, 'hand', side,
                list(HAND_JOINT_NAMES[side]), [.01 * sequence] * 20, 'producer', 'router')
                for side in ('left', 'right')})
        fixture.ManusRecordingCheckTest().recording(path,
            assets=hand_replay_asset_hashes(ROOT), on_callback=command)

    def test_retargets_every_callback_but_does_not_replay_authority(self):
        from tianji_teleop.recording.hand_command_check import check_manus_hand_commands
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            backend = Backend()
            with patch('socket.socket', side_effect=AssertionError('network access')):
                report = check_manus_hand_commands(path, root=ROOT, backend_factory=lambda **kw: backend)
            self.assertTrue(report['passed'], report)
            self.assertEqual(backend.sequences, [1, 2, 3])
            self.assertTrue(backend.closed)
            self.assertEqual(report['matched_commands'], 4)
            self.assertEqual(report['operator_events_executed'], 0)

    def test_tampered_joint_reports_side_and_callback(self):
        from tianji_teleop.recording.hand_command_check import check_manus_hand_commands
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                file['joint/command/hand/right/position_rad'][0, 3] += .1
            report = check_manus_hand_commands(path, root=ROOT, backend_factory=lambda **kw: Backend())
            self.assertFalse(report['passed'])
            self.assertEqual(report['first_difference']['side'], 'right')
            self.assertEqual(report['first_difference']['callback_sequence'], 2)

    def test_missing_asset_provenance_rejects_before_starting_solver(self):
        from tianji_teleop.recording.hand_command_check import check_manus_hand_commands
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            fixture.ManusRecordingCheckTest().recording(path)
            with patch('tianji_teleop.recording.hand_command_check.OfficialHandClient') as backend:
                with self.assertRaisesRegex(ValueError, 'asset'):
                    check_manus_hand_commands(path, root=ROOT)
                backend.assert_not_called()

    def test_cli_pico_retarget_checks_missing_input_before_solver_start(self):
        script = ROOT / 'scripts/check_dual_recording.py'
        result = subprocess.run([sys.executable, str(script), '--mode', 'pico',
            '--input', 'not-opened.h5', '--retarget-hand-commands'],
            capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2)
        value = json.loads(result.stdout)
        self.assertIn('not-opened.h5', value['error'])
        self.assertEqual(value['operator_events_executed'], 0)

    def test_changed_asset_digest_rejects_before_solver_start(self):
        from tianji_teleop.recording.hand_command_check import check_manus_hand_commands
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                metadata = json.loads(file['meta/hand_tracking'].attrs['metadata_json'])
                hashes = metadata['resolved_configuration']['asset_sha256']
                hashes[next(iter(hashes))] = '0' * 64
                file['meta/hand_tracking'].attrs['metadata_json'] = json.dumps(metadata)
            factory = unittest.mock.Mock()
            with self.assertRaisesRegex(ValueError, 'asset provenance'):
                check_manus_hand_commands(path, root=ROOT, backend_factory=factory)
            factory.assert_not_called()

    def test_solver_failure_always_closes_offline_worker(self):
        from tianji_teleop.recording.hand_command_check import check_manus_hand_commands
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            backend = Backend()
            with patch.object(backend, 'retarget', side_effect=RuntimeError('solver failed')):
                with self.assertRaisesRegex(RuntimeError, 'solver failed'):
                    check_manus_hand_commands(path, root=ROOT, backend_factory=lambda **kw: backend)
            self.assertTrue(backend.closed)


class HandReplayAssetHashTest(unittest.TestCase):
    def assets(self, root):
        paths = [root / 'scripts/wuji_hand_worker.py', root / 'tools/wuji_hand_native/pixi.lock',
                 root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
                 root / 'third_party/wuji_hand_retargeting/source.py']
        for path in paths:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'test asset')
        manifest = root / 'third_party/wuji_hand_retargeting/source_manifest.json'
        manifest.write_text(json.dumps({'files': [{'destination': 'source.py'}]}))
        return manifest

    def test_hashes_complete_known_slice_without_native_dependencies(self):
        from tianji_teleop.recording.hand_command_check import hand_replay_asset_hashes
        import hashlib
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assets(root)
            result = hand_replay_asset_hashes(root)
            self.assertEqual(len(result), 5)
            self.assertEqual(result['scripts/wuji_hand_worker.py'],
                             hashlib.sha256(b'test asset').hexdigest())

    def test_source_manifest_cannot_escape_pinned_directory(self):
        from tianji_teleop.recording.hand_command_check import hand_replay_asset_hashes
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = self.assets(root)
            (root / 'third_party/outside.py').write_bytes(b'not a pinned source')
            manifest.write_text(json.dumps({'files': [{'destination': '../outside.py'}]}))
            with self.assertRaisesRegex(ValueError, 'escapes pinned'):
                hand_replay_asset_hashes(root)
