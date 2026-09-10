from copy import deepcopy
import json
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import h5py

from tests import test_tjvr_recording_check as fixtures

ROOT = Path(__file__).resolve().parents[1]


def cycle(epoch, stamp):
    return dict(kind='native_cycle', received_timestamp_ns=stamp + 1,
        payload=dict(run_id='run', execution_epoch=epoch,
            native_attempt=dict(tick_id=1, timestamp_ns=stamp, sample=None)))


def reset(epoch=2):
    return dict(kind='operator_result', received_timestamp_ns=120,
        payload=dict(run_id='run', kind='operator_result', action='rearm', accepted=True,
            execution_epoch=epoch, reset_ack=dict(schema_version=1, kind='spark_reset_ack',
                execution_epoch=epoch, position_rad=[.1] * 14,
                velocity_rad_s=[0.] * 14, acceleration_rad_s2=[0.] * 14)))


class SparkResetRecordingCheckTest(unittest.TestCase):
    def test_reset_ack_is_passive_and_required_before_epoch_changes(self):
        from tianji_teleop.recording.spark_reset_check import check_reset_audits
        rows = [cycle(1, 110), reset(), cycle(2, 130)]
        original = deepcopy(rows)
        report = check_reset_audits(rows, run_id='run')
        self.assertTrue(report['passed'], report)
        self.assertEqual(report['validated_reset_acks'], 1)
        self.assertEqual(report['operator_events_executed'], 0)
        self.assertFalse(report['physical_state_verified'])
        self.assertEqual(rows, original)
        self.assertFalse(check_reset_audits([rows[0], rows[2]], run_id='run')['passed'])

    def test_invalid_ack_and_epoch_or_clock_changes_fail(self):
        from tianji_teleop.recording.spark_reset_check import check_reset_audits
        for mutation in ('velocity', 'acceleration', 'shape', 'boolean', 'overflow', 'epoch', 'run',
                         'late', 'duplicate', 'rejected', 'initial_epoch'):
            with self.subTest(mutation=mutation):
                rows = [cycle(1, 110), reset(), cycle(2, 130)]
                ack = rows[1]['payload']['reset_ack']
                if mutation in ('velocity', 'acceleration'):
                    ack['velocity_rad_s' if mutation == 'velocity' else 'acceleration_rad_s2'][0] = .1
                elif mutation == 'shape':
                    ack['position_rad'].pop()
                elif mutation == 'boolean':
                    ack['position_rad'][0] = True
                elif mutation == 'overflow':
                    ack['position_rad'][0] = 10**400
                elif mutation == 'epoch':
                    ack['execution_epoch'] = 3
                elif mutation == 'run':
                    rows[1]['payload']['run_id'] = 'other'
                elif mutation == 'late':
                    rows[1]['received_timestamp_ns'] = 140
                elif mutation == 'duplicate':
                    rows.insert(2, deepcopy(rows[1]))
                elif mutation == 'rejected':
                    rows[1]['payload'] = dict(run_id='run', action='rearm', accepted=False)
                else:
                    rows[0]['payload']['execution_epoch'] = 2
                self.assertFalse(check_reset_audits(rows, run_id='run')['passed'])

    def test_rejected_rearm_does_not_advance_epoch(self):
        from tianji_teleop.recording.spark_reset_check import check_reset_audits
        denied = reset()
        denied['payload'] = dict(run_id='run', action='rearm', accepted=False)
        report = check_reset_audits([cycle(1, 110), denied, cycle(1, 130)], run_id='run')
        self.assertTrue(report['passed'], report)
        self.assertEqual(report['rejected_rearms'], 1)
        self.assertEqual(report['validated_reset_acks'], 0)

    def test_cli_reset_check_rejects_missing_ack_without_changing_default_check(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            fixtures.TjvrRecordingCheckTest().recording(path, native_inputs=True)
            command = [sys.executable, str(ROOT / 'scripts/check_dual_recording.py'),
                '--mode', 'tjvr', '--input', str(path)]
            ordinary = subprocess.run(command, capture_output=True, text=True, timeout=10)
            self.assertEqual(ordinary.returncode, 0, ordinary.stderr)
            checked = subprocess.run(command + ['--check-native-resets'],
                capture_output=True, text=True, timeout=10)
            self.assertEqual(checked.returncode, 1, checked.stdout + checked.stderr)
            self.assertFalse(json.loads(checked.stdout)['reset_audit']['passed'])

    def test_cli_checks_ack_without_modifying_the_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            fixtures.TjvrRecordingCheckTest().recording(path, native_inputs=True)
            with h5py.File(path, 'r+') as file:
                group = file['meta/dual_audit']
                for dataset in group.values():
                    last = dataset[7]
                    dataset.resize((9,))
                    dataset[8] = last
                group['kind'][7] = 'operator_result'
                group['received_timestamp_ns'][7] = 125
                group['time_ns'][7] = 25
                group['payload_json'][7] = json.dumps(reset()['payload'])
            before = hashlib.sha256(path.read_bytes()).hexdigest()
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/check_dual_recording.py'),
                '--mode', 'tjvr', '--check-native-resets', '--input', str(path)],
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            report = json.loads(result.stdout)
            self.assertEqual(report['reset_audit']['validated_reset_acks'], 1)
            self.assertEqual(report['operator_events_executed'], 0)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), before)

    def test_reset_check_requires_tjvr_and_native_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            fixtures.TjvrRecordingCheckTest().recording(path)
            for mode in ('pico', 'manus', 'tjvr'):
                result = subprocess.run([sys.executable, str(ROOT / 'scripts/check_dual_recording.py'),
                    '--mode', mode, '--check-native-resets', '--input', str(path)],
                    capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)

    def test_foreign_run_ack_is_not_counted_or_applied(self):
        from tianji_teleop.recording.spark_reset_check import check_reset_audits
        foreign = reset()
        foreign['payload']['run_id'] = 'other'
        report = check_reset_audits([cycle(1, 110), foreign, cycle(2, 130)], run_id='run')
        self.assertFalse(report['passed'])
        self.assertEqual(report['validated_reset_acks'], 0)
