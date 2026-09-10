from pathlib import Path
import tempfile
import unittest
import json
import subprocess
import sys
import hashlib
from unittest.mock import patch

import h5py

from tianji_teleop.hand_tracking.reference_manus import HandInputAssembler, RawvizHandInputProcessor
from tianji_teleop.hand_tracking.reference_manus_process import ManusCallback
from tianji_teleop.recording.live_capture import LiveCapture
from tianji_teleop.recording.session_h5 import SessionH5Writer
from tests.test_reference_manus_process import rawviz_records


class ManusRecordingCheckTest(unittest.TestCase):
    def recording(self, path, sides=('right', 'left'), *, assets=None, on_callback=None):
        contract = dict(version=1, sides=list(sides), right_glove=None, left_glove=None,
                        callback_order='right_then_left', callback_trigger='each_accepted_pose')
        with SessionH5Writer(path, source_type='vr_manus_sim', robot_model='spark',
                router_zid='router', schema_version='1.2',
                metadata=dict(resolved_configuration=dict(manus_input_contract=contract,
                    asset_sha256=assets or {}))) as writer:
            class Sink:
                def append(self, method, *args, **kwargs):
                    getattr(writer, method)(*args, **kwargs)
            capture = LiveCapture(Sink(), run_id='test')
            now, sequence = 1000, 0
            def publish(frame):
                nonlocal sequence
                sequence += 1
                capture.callback(ManusCallback('manus', sequence, now,
                    tuple(frame.values.tolist()), frame.sequences, frame.source_timestamps_ns))
                if on_callback is not None:
                    on_callback(sequence, now, capture)
            processor = RawvizHandInputProcessor(
                HandInputAssembler('right' in sides, 'left' in sides), publish)
            # Original callbacks: wait for both sides, then publish each POSE;
            # repeated sequence is suppressed, not resampled/coalesced.
            records = ''.join(rawviz_records(side, seq) for side, seq in (
                ('right', 1), ('left', 1), ('right', 2), ('right', 2), ('left', 2)))
            for line in records.splitlines():
                now += 1
                capture.rawviz(line, now)
                processor.process_line(line)

    def test_original_raw_lines_reconstruct_ordered_callbacks_without_network(self):
        from tianji_teleop.recording.manus_check import check_manus_recording
        for sides, count in ((('right', 'left'), 3), (('left',), 2)):
            with self.subTest(sides=sides), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'capture.h5'
                self.recording(path, sides)
                before = hashlib.sha256(path.read_bytes()).hexdigest()
                with patch('socket.socket', side_effect=AssertionError('network access')):
                    report = check_manus_recording(path)
                self.assertTrue(report['passed'], report)
                self.assertEqual(report['matched_callbacks'], count)
                self.assertEqual(report['operator_events_executed'], 0)
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), before)

    def test_tampered_callback_reports_stage_sequence_and_field(self):
        from tianji_teleop.recording.manus_check import check_manus_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                file['raw/manus_callbacks/points'][1, 5] += .1
            report = check_manus_recording(path)
            self.assertFalse(report['passed'])
            self.assertEqual(report['first_difference']['field'], 'points')
            self.assertEqual(report['first_difference']['callback_sequence'], 2)

    def test_missing_parser_contract_does_not_guess_bindings(self):
        from tianji_teleop.recording.manus_check import check_manus_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                file['meta/hand_tracking'].attrs['metadata_json'] = '{}'
            with self.assertRaisesRegex(ValueError, 'cannot infer glove bindings'):
                check_manus_recording(path)

    def test_callback_sequence_duplicates_and_clock_corruption_fail(self):
        from tianji_teleop.recording.manus_check import check_manus_recording
        for field in ('callback_sequence', 'received_timestamp_ns'):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'capture.h5'
                self.recording(path)
                with h5py.File(path, 'r+') as file:
                    file['raw/manus_callbacks/' + field][1] = file['raw/manus_callbacks/' + field][0]
                report = check_manus_recording(path)
                self.assertFalse(report['passed'])
                self.assertEqual(report['first_difference']['field'], field)

    def test_malformed_recorded_configuration_is_a_clear_input_error(self):
        from tianji_teleop.recording.manus_check import check_manus_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                file['meta/hand_tracking'].attrs['metadata_json'] = json.dumps(
                    {'resolved_configuration': []})
            with self.assertRaisesRegex(ValueError, 'configuration'):
                check_manus_recording(path)

    def test_raw_line_run_identity_is_validated_before_indexing(self):
        from tianji_teleop.recording.manus_check import check_manus_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                dataset = file['meta/dual_audit/payload_json']
                value = json.loads(dataset.asstr()[0])
                value['run_id'] = {'invalid': True}
                dataset[0] = json.dumps(value)
            with self.assertRaisesRegex(ValueError, 'run identity'):
                check_manus_recording(path)

    def test_cli_manus_mode_and_missing_input(self):
        script = Path(__file__).resolve().parents[1] / 'scripts/check_dual_recording.py'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            result = subprocess.run([sys.executable, str(script), '--mode', 'manus',
                '--input', str(path)], capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(json.loads(result.stdout)['passed'])
            result = subprocess.run([sys.executable, str(script), '--mode', 'manus',
                '--input', str(path.parent / 'missing.h5')], capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertEqual(json.loads(result.stdout)['operator_events_executed'], 0)
