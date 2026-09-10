from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import h5py

from tests.test_pico_hand_service import wire_frame
from tests import test_dual_recording_check as check_fixture
from tianji_teleop.recording.session_h5 import SessionH5Writer, SessionH5Reader, SessionH5Error


class PicoRawRecordingClockTest(unittest.TestCase):
    def test_schema12_preserves_input_clock_separately_from_recorder_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            with SessionH5Writer(path, source_type='pico2_hands_sim', robot_model='test',
                    router_zid='router', schema_version='1.2') as writer:
                for seq in range(2):
                    writer.append_raw_pico(replace(wire_frame(), received_timestamp_ns=1234 + seq),
                                           received_time_ns=9000 + seq * 10)
            with SessionH5Reader(path) as reader:
                rows = reader.read_raw_pico()
                self.assertEqual([r['received_timestamp_ns'] for r in rows], [1234, 1235])
                self.assertEqual([r['time_ns'] for r in rows], [0, 10])

    def test_original_schema11_layout_is_not_extended(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'old.h5'
            with SessionH5Writer(path, source_type='hand_tracking_sim', robot_model='test',
                    router_zid='router', schema_version='1.1') as writer:
                writer.append_raw_pico(wire_frame())
            with SessionH5Reader(path) as reader:
                self.assertNotIn('received_timestamp_ns', reader.read_raw_pico()[0])

    def test_uniform_observation_clock_corruption_is_detected_against_raw(self):
        from tianji_teleop.recording.dual_check import check_pico_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            check_fixture.DualRecordingCheckTest().recording(path)
            with h5py.File(path, 'r+') as file:
                for stage in ('hand_tracking', 'arm_input'):
                    for side in ('left', 'right'):
                        file[f'observation/{stage}/{side}/received_timestamp_ns'][:] += 1
            report = check_pico_recording(path)
            self.assertFalse(report['passed'])
            self.assertEqual(report['raw_receiver_clock_frames'], 2)
            self.assertEqual(report['first_difference']['field'], 'received_timestamp_ns')

    def test_prior_schema12_without_optional_clock_is_readable_but_reports_limit(self):
        from tianji_teleop.recording.dual_check import check_pico_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'old12.h5'
            check_fixture.DualRecordingCheckTest().recording(path)
            with h5py.File(path, 'r+') as file:
                if 'received_timestamp_ns' in file['raw/pico_hand_tracking']:
                    del file['raw/pico_hand_tracking/received_timestamp_ns']
            report = check_pico_recording(path)
            self.assertTrue(report['passed'], report)
            self.assertEqual(report['raw_receiver_clock_frames'], 0)
            self.assertTrue(any('clock unavailable' in limit for limit in report['limitations']))
