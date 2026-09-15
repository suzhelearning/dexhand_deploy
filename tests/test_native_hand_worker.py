import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


class NativeHandWorkerSelectionTest(unittest.TestCase):
    def test_native_worker_selection_is_explicit_and_fails_before_spawn_when_missing(self):
        from tianji_teleop.producers.hand_retarget import OfficialHandClient

        with patch.dict(os.environ, {'TIANJI_HAND_WORKER_BACKEND': 'cpp'}, clear=False):
            with self.assertRaisesRegex(RuntimeError, 'native hand worker'):
                OfficialHandClient(python=sys.executable, script=__file__,
                                   native_worker=ROOT / 'build/hand-native/does-not-exist')

    def test_unknown_worker_backend_is_rejected_before_spawn(self):
        from tianji_teleop.producers.hand_retarget import OfficialHandClient

        with patch.dict(os.environ, {'TIANJI_HAND_WORKER_BACKEND': 'not-a-backend'}, clear=False):
            with self.assertRaisesRegex(ValueError, 'TIANJI_HAND_WORKER_BACKEND'):
                OfficialHandClient(python=sys.executable, script=__file__)


@unittest.skipUnless(
    (ROOT / 'build/hand-native/tianji_hand_native_worker').is_file()
    and (ROOT / 'build/hand-native/libtianji_hand_optimizer.so').is_file(),
    'requires the native Hand2 worker build',
)
class NativeHandWorkerMetadataTest(unittest.TestCase):
    def test_metadata_matches_wire_and_has_no_machine_paths(self):
        import json

        from tianji_teleop.producers.native_hand_worker import recording_metadata

        with patch.dict(os.environ, {'TIANJI_HAND_WORKER_BACKEND': 'cpp'}, clear=False):
            metadata = recording_metadata(os.environ)
        self.assertEqual(metadata['request_size_bytes'], 1040)
        self.assertEqual(metadata['response_size_bytes'], 344)
        self.assertNotIn('/home/', json.dumps(metadata, sort_keys=True))


@unittest.skipUnless(
    os.environ.get('WUJI_REFERENCE_TEST') == '1'
    and (ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python').is_file()
    and (ROOT / 'build/hand-native/tianji_hand_native_worker').is_file(),
    'requires the pinned Hand2 environment and the native worker build',
)
class NativeHandWorkerReferenceTest(unittest.TestCase):
    def test_manus_scheduler_continuity_matches_python_on_skipped_frames(self):
        import subprocess
        import time
        import numpy as np
        from tianji_teleop.producers.hand_retarget import OfficialHandClient
        from tianji_teleop.producers.native_hand_scheduler import _INPUT, _OUTPUT
        python = ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python'
        reference = OfficialHandClient(python=python, script=ROOT/'scripts/wuji_hand_worker.py',
            single_hand_side='right', startup_handshake=True, filter_continuity_ns=200_000_000)
        self.addCleanup(reference.close)
        process = subprocess.Popen([str(python), str(ROOT/'scripts/wuji_hand_native_scheduler_launcher.py'),
            '--native-scheduler', str(ROOT/'build/hand-native/tianji_hand_native_scheduler'),
            '--filter-continuity-ns', '200000000'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        try:
            stamp = time.monotonic_ns() - 2_000_000_000
            for index, (seq, offset) in enumerate(((1,0),(5,5),(9,15),(10,25),(19,226),(25,231))):
                points = self._points()
                points[-1] += .005 * index
                padded = points + points
                ts = stamp + offset * 1_000_000
                process.stdin.write(_INPUT.pack(b'TJHI',1,3,_INPUT.size,seq,ts,1,126,0,*padded))
                process.stdin.flush()
                import select
                self.assertTrue(select.select([process.stdout],[],[],10)[0])
                row = _OUTPUT.unpack(process.stdout.read(_OUTPUT.size))
                self.assertEqual(row[5], seq)
                expected = reference.retarget(padded, sequence=seq, timestamp_ns=ts)
                np.testing.assert_allclose(row[-40:-20],expected['left']['position_rad'],atol=2e-5,rtol=0)
                np.testing.assert_allclose(row[-20:],expected['right']['position_rad'],atol=2e-5,rtol=0)
        finally:
            process.terminate()
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill();process.wait()
            process.stdin.close();process.stdout.close()

    @staticmethod
    def _points():
        points = [0.0, 0.0, 0.0]
        for finger in range(5):
            for joint in range(4):
                points.extend([0.02 * (finger - 2), 0.02 * (joint + 1), 0.002 * joint])
        return points

    def _clients(self, *, single_hand_side):
        from tianji_teleop.producers.hand_retarget import OfficialHandClient

        python = ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python'
        common = dict(
            python=python,
            script=ROOT / 'scripts/wuji_hand_worker.py',
            single_hand_side=single_hand_side,
            startup_handshake=True,
        )
        with patch.dict(os.environ, {'TIANJI_HAND_WORKER_BACKEND': 'cpp'}, clear=False):
            native = OfficialHandClient(**common)
        reference = OfficialHandClient(**common)
        self.addCleanup(native.close)
        self.addCleanup(reference.close)
        return native, reference

    def test_native_worker_matches_python_worker_and_resets_on_gap(self):
        import numpy as np
        native, reference = self._clients(single_hand_side='right')
        points = self._points()

        for sequence, timestamp in ((1, 1_000), (2, 2_000), (8, 8_000)):
            actual = native.retarget(points, sequence=sequence, timestamp_ns=timestamp)
            expected = reference.retarget(points, sequence=sequence, timestamp_ns=timestamp)
            self.assertEqual(actual['callback_sequence'], sequence)
            self.assertEqual(actual['timestamp_ns'], timestamp)
            for side in ('left', 'right'):
                self.assertEqual(actual[side]['valid'], expected[side]['valid'])
                np.testing.assert_allclose(
                    actual[side]['position_rad'],
                    expected[side]['position_rad'],
                    rtol=0,
                    atol=2e-5,
                )

    def test_native_worker_matches_python_worker_for_bilateral_frame(self):
        import numpy as np

        native, reference = self._clients(single_hand_side='right')
        points = self._points()
        for sequence, timestamp in ((1, 1_000), (2, 2_000), (8, 8_000)):
            actual = native.retarget(points + points, sequence=sequence, timestamp_ns=timestamp)
            expected = reference.retarget(points + points, sequence=sequence, timestamp_ns=timestamp)
            self.assertEqual(actual['callback_sequence'], sequence)
            self.assertEqual(actual['timestamp_ns'], timestamp)
            for side in ('left', 'right'):
                self.assertEqual(actual[side]['valid'], expected[side]['valid'])
                np.testing.assert_allclose(
                    actual[side]['position_rad'],
                    expected[side]['position_rad'],
                    rtol=0,
                    atol=2e-5,
                )


if __name__ == '__main__':
    unittest.main()
