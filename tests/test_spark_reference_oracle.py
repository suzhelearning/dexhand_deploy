import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('TJVR_REFERENCE_ROOT'), 'optional original Viewer source')
class SparkOracleSourceTest(unittest.TestCase):
    def test_original_control_block_is_verbatim_not_the_migrated_wrapper(self):
        ref = Path(os.environ['TJVR_REFERENCE_ROOT'])
        commit = subprocess.check_output(['git', '-C', str(ref), 'rev-parse', 'HEAD'], text=True).strip()
        self.assertEqual(commit, 'c022b17789e9b81141915662b3bb802f4c20a396')
        source = (ref / 'apps/run_qp_ik_viewer.cpp').read_text()
        start = source.index('    PicoTeleopFrame pico_frame;', source.index('void controlLoop('))
        end = source.index('    if (!paused && !pico_paused && hand_frames != nullptr)', start)
        oracle = (ROOT / 'tests/cpp/spark/reference_viewer_cycle.hpp').read_text()
        extracted = oracle.split('// BEGIN VERBATIM REFERENCE VIEWER CONTROL BLOCK\n')[1].split(
            '// END VERBATIM REFERENCE VIEWER CONTROL BLOCK')[0]
        self.assertEqual(extracted, source[start:end])
        self.assertNotIn('tianji_spark/bilateral_cycle.hpp', oracle)
