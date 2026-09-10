import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import h5py

from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.recording.live_capture import LiveCapture
from tianji_teleop.recording.session_h5 import SessionH5Writer
from tests.test_reference_tjvr_receiver import packet

ROOT = Path(__file__).resolve().parents[1]


class TjvrRecordingCheckTest(unittest.TestCase):
    def recording(self, path, *, native_inputs=False):
        with SessionH5Writer(path, source_type='vr_manus_sim', robot_model='spark',
                router_zid='router', schema_version='1.2', metadata={
                    'run_id': 'run', 'resolved_configuration': {'tjvr_stream_contract': {
                        'version': 1, 'initial_state': 'reset',
                        'max_position_jump_m': .15, 'max_orientation_jump_rad': .6}}}) as writer:
            class Recorder:
                def append(self, method, *args, **kwargs):
                    getattr(writer, method)(*args, **kwargs)
            capture = LiveCapture(Recorder(), run_id='run')
            source = ReferenceTjvrReceiver('receiver', .15, .6,
                raw_frame_sink=capture.tjvr, decision_sink=capture.tjvr_decision)
            samples = []
            source.ingest(b'malformed', 99)  # receiver ordinals may contain gaps
            for i, (seq, x) in enumerate(((1, 10.), (1, 10.), (2, 10.3), (3, 10.31), (4, 10.32))):
                source.ingest(packet(seq, x), 100 + i)
                sample = source.try_read_latest()
                if sample is not None:
                    samples.append(sample.to_dict())
            if native_inputs:
                # Null means no new sample this tick, not a synthetic zero pose.
                # A reset starts tick 1 in a new execution epoch, not a new input stream.
                for epoch, tick, stamp, sample in ((1, 1, 110, samples[0]),
                        (1, 2, 120, None), (2, 1, 130, samples[1])):
                    capture.audit('native_cycle', dict(execution_epoch=epoch,
                        native_attempt=dict(tick_id=tick, timestamp_ns=stamp, sample=sample)), stamp + 1)

    def test_native_consumption_checks_original_envelopes_across_execution_reset(self):
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            self.recording(path, native_inputs=True)
            with patch('socket.socket', side_effect=AssertionError('offline only')):
                result = check_tjvr_recording(path)
            self.assertTrue(result['passed'], result)
            self.assertEqual(result.get('native_input_check'), 'checked')
            self.assertEqual(result['native_attempts'], 3)
            self.assertEqual(result['matched_native_inputs'], 2)
            self.assertEqual(result['native_ticks_without_new_input'], 1)

    def test_native_consumption_tampering_is_not_masked_by_correct_gate_records(self):
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        for mutation in ('generation', 'packet', 'receiver', 'future', 'reused_input', 'tick_gap', 'run',
                         'not_accepted', 'boolean_tick', 'missing_sample', 'late_attempt'):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'session.h5'
                self.recording(path, native_inputs=True)
                with h5py.File(path, 'r+') as file:
                    data = file['meta/dual_audit/payload_json']
                    row = json.loads(data[7])
                    sample = row['native_attempt']['sample']
                    if mutation == 'generation':
                        sample['resynchronization_generation'] = 0
                    elif mutation == 'packet':
                        sample['raw_packet_base64'] = json.loads(data[5])['native_attempt']['sample']['raw_packet_base64']
                    elif mutation == 'receiver':
                        sample['receiver_instance_id'] = 'other'
                    elif mutation == 'future':
                        row['native_attempt']['timestamp_ns'] = 103
                    elif mutation == 'reused_input':
                        row['native_attempt']['sample'] = json.loads(data[5])['native_attempt']['sample']
                    elif mutation == 'tick_gap':
                        row['native_attempt']['tick_id'] = 2
                    elif mutation == 'not_accepted':
                        sample['receiver_frame_sequence'] = 3
                    elif mutation == 'boolean_tick':
                        row['native_attempt']['tick_id'] = True
                    elif mutation == 'missing_sample':
                        row['native_attempt'].pop('sample')
                    elif mutation == 'late_attempt':
                        row['native_attempt']['timestamp_ns'] = 140
                    else:
                        row['run_id'] = 'other'
                    data[7] = json.dumps(row)
                result = check_tjvr_recording(path)
                self.assertFalse(result['passed'], result)

    def test_consumption_before_gate_audit_is_not_validated_using_future_rows(self):
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            self.recording(path, native_inputs=True)
            with h5py.File(path, 'r+') as file:
                for dataset in file['meta/dual_audit'].values():
                    original = dataset[:]
                    dataset[0] = original[5]
                    dataset[1:6] = original[:5]
            result = check_tjvr_recording(path)
            self.assertFalse(result['passed'], result)
            self.assertEqual(result['first_difference']['field'], 'sample_gate_decision_after_native_cycle')

    def test_legacy_missing_and_null_attempts_have_explicit_coverage(self):
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        for case in ('legacy', 'mixed', 'null'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'session.h5'
                self.recording(path, native_inputs=True)
                with h5py.File(path, 'r+') as file:
                    data = file['meta/dual_audit/payload_json']
                    for index in (range(5, 8) if case != 'mixed' else (6,)):
                        row = json.loads(data[index])
                        if case == 'null':
                            row['native_attempt'] = None
                        else:
                            row.pop('native_attempt')
                        data[index] = json.dumps(row)
                result = check_tjvr_recording(path)
                if case == 'mixed':
                    self.assertFalse(result['passed'])
                else:
                    self.assertTrue(result['passed'], result)
                    self.assertEqual(result['native_attempts'], 0)
                    self.assertEqual(result['native_input_check'],
                        'unavailable_legacy_audit' if case == 'legacy' else 'no_native_attempts')

    def test_gate_only_capture_does_not_claim_native_consumption_coverage(self):
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            self.recording(path)
            result = check_tjvr_recording(path)
            self.assertTrue(result['passed'])
            self.assertEqual(result.get('native_input_check'), 'not_recorded')

    def test_rebuilds_gate_with_duplicate_packets_and_jump_recovery_without_network(self):
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            self.recording(path)
            before = hashlib.sha256(path.read_bytes()).hexdigest()
            with patch('socket.socket', side_effect=AssertionError('offline only')):
                result = check_tjvr_recording(path)
            self.assertTrue(result['passed'], result)
            self.assertEqual(result['matched_decisions'], 5)
            self.assertEqual(result['accepted'], 2)
            self.assertEqual(result['rejected'], 3)
            self.assertEqual(result['resynchronizations'], 1)
            self.assertEqual(result['operator_events_executed'], 0)
            self.assertFalse(result['complete_plan_acceptance'])
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), before)

    def test_tampered_decision_fails_at_first_field(self):
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                data = file['meta/dual_audit/payload_json']
                row = json.loads(data[1])
                row['accepted'] = True
                data[1] = json.dumps(row)
            result = check_tjvr_recording(path)
            self.assertFalse(result['passed'])
            self.assertEqual(result['first_difference']['field'], 'accepted')

    def test_cli_checks_tjvr_without_hand_solver(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            self.recording(path)
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/check_dual_recording.py'),
                '--mode', 'tjvr', '--input', str(path)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            self.assertTrue(json.loads(result.stdout)['passed'])

    def test_missing_reordered_and_wrong_clock_audits_fail_closed(self):
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        for corruption in ('missing', 'reordered', 'clock', 'integer_boolean', 'run_id'):
            with self.subTest(corruption=corruption), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'session.h5'
                self.recording(path)
                with h5py.File(path, 'r+') as file:
                    group = file['meta/dual_audit']
                    if corruption == 'missing':
                        for dataset in group.values():
                            dataset.resize((4,))
                    elif corruption == 'clock':
                        group['received_timestamp_ns'][0] += 1
                    else:
                        dataset = group['payload_json']
                        if corruption == 'reordered':
                            first, second = dataset[0], dataset[1]
                            dataset[0], dataset[1] = second, first
                        else:
                            row = json.loads(dataset[0])
                            row['accepted' if corruption == 'integer_boolean' else 'run_id'] = (
                                1 if corruption == 'integer_boolean' else 'other-run')
                            dataset[0] = json.dumps(row)
                self.assertFalse(check_tjvr_recording(path)['passed'])

    def test_raw_receiver_ordinal_reuse_is_invalid_not_packet_duplicate(self):
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                file['raw/tjvr_upper_limb/receiver_frame_sequence'][1] = 2
            with self.assertRaisesRegex(ValueError, 'receive ordinals'):
                check_tjvr_recording(path)

    def test_tjvr_cli_rejects_hand_retarget_before_file_access(self):
        result = subprocess.run([sys.executable, str(ROOT / 'scripts/check_dual_recording.py'),
            '--mode', 'tjvr', '--retarget-hand-commands', '--input', '/no-such-file.h5'],
            capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2)
        self.assertIn('contains no hand fingers', result.stderr)

    def test_missing_or_invalid_contract_is_not_guessed(self):
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        for mutation in ('missing', 'boolean_threshold', 'initial_state', 'run_id'):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'session.h5'
                self.recording(path)
                with h5py.File(path, 'r+') as file:
                    group = file['meta/hand_tracking']
                    metadata = json.loads(group.attrs['metadata_json'])
                    contract = metadata['resolved_configuration']['tjvr_stream_contract']
                    if mutation == 'missing':
                        metadata['resolved_configuration'].pop('tjvr_stream_contract')
                    elif mutation == 'run_id':
                        metadata['run_id'] = []
                    elif mutation == 'boolean_threshold':
                        contract['max_position_jump_m'] = True
                    else:
                        contract['initial_state'] = 'unknown'
                    group.attrs['metadata_json'] = json.dumps(metadata)
                with self.assertRaises(ValueError):
                    check_tjvr_recording(path)
