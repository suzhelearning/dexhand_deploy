from dataclasses import replace
import hashlib
from pathlib import Path
import tempfile
import subprocess
import sys
import json
import unittest
from unittest.mock import patch

import h5py

from tests.test_pico_hand_service import wire_frame
from tests import test_dual_input_recorder as recorder_fixture
from tianji_teleop.hand_tracking.pico_gestures import PicoGestureRuntime


class DualRecordingCheckTest(unittest.TestCase):
    def recording(self, path):
        node = recorder_fixture.DualInputRecorderTest().node(
            recorder_fixture.Session(), path, 'pico2_hands_sim', 'pico')
        try:
            runtime = PicoGestureRuntime(publish=node.receive, publisher_instance_id='pico', router_zid='router')
            for seq in (7, 8):
                runtime.ingest_pico(replace(wire_frame(), receiver_frame_sequence=seq,
                    received_timestamp_ns=1000 + seq))
            node.writer.append_dual_audit('operator_result', {'action': 'start_request'},
                                         received_timestamp_ns=2000)
        finally:
            node.close()

    def test_reconstructs_raw_without_network_authority_or_modifying_file(self):
        from tianji_teleop.recording.dual_check import check_pico_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            before = hashlib.sha256(path.read_bytes()).hexdigest()
            with patch('socket.socket', side_effect=AssertionError('offline check opened a socket')):
                report = check_pico_recording(path)
            self.assertTrue(report['passed'], report)
            self.assertEqual(report['raw_frames'], 2)
            self.assertEqual(report['matched_observations'], 8)
            self.assertEqual(report['matched_gesture_observations'], 2)
            self.assertEqual(report['operator_events_executed'], 0)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), before)

    def test_reports_first_differing_field_and_frame(self):
        from tianji_teleop.recording.dual_check import check_pico_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                file['observation/hand_tracking/left/keypoints_m'][0, 5, 0] += .1
            report = check_pico_recording(path)
            self.assertFalse(report['passed'])
            self.assertEqual(report['first_difference']['stage'], 'hand_observation')
            self.assertEqual(report['first_difference']['side'], 'left')
            self.assertEqual(report['first_difference']['field'], 'keypoints_m')
            self.assertEqual(report['first_difference']['receiver_frame_sequence'], 7)

    def test_duplicate_associations_are_not_silently_overwritten(self):
        from tianji_teleop.recording.dual_check import check_pico_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                dataset = file['observation/hand_tracking/left/frame_association_id']
                dataset[1] = dataset[0]
            report = check_pico_recording(path)
            self.assertFalse(report['passed'])
            self.assertGreater(report['duplicate_observations'], 0)
            self.assertGreater(report['missing_observations'], 0)

    def test_raw_geometry_and_observation_clock_corruption_are_detected(self):
        from tianji_teleop.recording.dual_check import check_pico_recording
        for dataset, field in (
            ('raw/pico_hand_tracking/head_pose', 'head_pose'),
            ('observation/arm_input/right/received_timestamp_ns', 'received_timestamp_ns')):
            with self.subTest(dataset=dataset), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'capture.h5'
                self.recording(path)
                with h5py.File(path, 'r+') as file:
                    file[dataset][0] += 1
                report = check_pico_recording(path)
                self.assertFalse(report['passed'])
                self.assertEqual(report['first_difference']['field'], field)

    def test_invalid_tolerances_are_rejected(self):
        from tianji_teleop.recording.dual_check import check_pico_recording
        for tolerance in (-1, float('nan'), float('inf'), True):
            with self.subTest(tolerance=tolerance), self.assertRaises(ValueError):
                check_pico_recording('not-opened.h5', atol=tolerance)

    def test_extra_observation_reports_first_difference(self):
        from tianji_teleop.recording.dual_check import check_pico_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                group = file['observation/hand_tracking/left']
                for dataset in group.values():
                    dataset.resize(3, axis=0)
                    dataset[2] = dataset[1]
                group['frame_association_id'][2] = 'orphan'
            report = check_pico_recording(path)
            self.assertFalse(report['passed'])
            self.assertEqual(report['extra_observations'], 1)
            self.assertEqual(report['first_difference']['field'], 'extra_association')

    def test_cli_outputs_json_and_failure_exit_status(self):
        script = Path(__file__).resolve().parents[1] / 'scripts/check_dual_recording.py'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            command = [sys.executable, str(script), '--input', str(path)]
            good = subprocess.run(command, capture_output=True, text=True, timeout=15)
            self.assertEqual(good.returncode, 0, good.stderr)
            self.assertTrue(json.loads(good.stdout)['passed'])
            with h5py.File(path, 'r+') as file:
                file['observation/arm_input/left/pose'][0, 0] += .1
            bad = subprocess.run(command, capture_output=True, text=True, timeout=15)
            self.assertEqual(bad.returncode, 1, bad.stderr)
            self.assertFalse(json.loads(bad.stdout)['passed'])

    def test_passive_gesture_reconstruction_detects_corruption_without_executing_events(self):
        from tianji_teleop.recording.dual_check import check_pico_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            with h5py.File(path, 'r+') as file:
                group = file['meta/dual_audit']
                index = list(group['kind'].asstr()[:]).index('operator_observation')
                value = json.loads(group['payload_json'].asstr()[index])
                value['hands']['left']['gesture'] = 'fist'
                group['payload_json'][index] = json.dumps(value)
            report = check_pico_recording(path)
            self.assertFalse(report['passed'])
            self.assertEqual(report['first_difference']['stage'], 'gesture_observation')
            self.assertEqual(report['operator_events_executed'], 0)
