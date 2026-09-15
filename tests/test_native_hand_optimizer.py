"""Pinned-environment differential tests, never open devices or publish commands."""
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
PYTHON = ROOT/'tools/wuji_hand_native/.pixi/envs/default/bin/python'


@unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST') == '1' and PYTHON.is_file(),
                     'set WUJI_REFERENCE_TEST=1 with the pinned Hand2 environment')
class NativeHandOptimizerTest(unittest.TestCase):
    def test_pinned_reference(self):
        env = {k:v for k,v in os.environ.items() if k not in
               ('PYTHONPATH','PYTHONHOME','LD_LIBRARY_PATH','LD_PRELOAD')}
        result = subprocess.run([str(PYTHON), str(ROOT/'tests/native_hand_optimizer_reference.py')],
                                env=env, capture_output=True, text=True, timeout=90)
        self.assertEqual(result.returncode, 0, result.stdout+'\n'+result.stderr)


class HandOptimizerSelectionTest(unittest.TestCase):
    def test_default_metadata_does_not_load_native_dependencies(self):
        from tianji_teleop.producers import native_hand_optimizer as module
        self.assertTrue(hasattr(module, 'recording_metadata'))
        self.assertIsNone(module.recording_metadata({}))
        with self.assertRaises(ValueError):
            module.recording_metadata({'TIANJI_HAND_OPTIMIZER_BACKEND':'invalid'})

    def test_pico_provenance(self):
        import json
        from unittest.mock import patch
        from tianji_teleop.producers import native_hand_optimizer as module
        from tianji_teleop.recording.session_recorder import _session_metadata
        if not module.library_path().is_file(): self.skipTest('build-native-hand-optimizer required')
        env = dict(TIANJI_RUN_ID='offline', TIANJI_HAND_OPTIMIZER_BACKEND='cpp',
            TIANJI_RESOLVED_DUAL_SESSION=json.dumps(dict(profile='pico2_hands_sim',
                config=dict(input_mode='pico2_hands', active_hand_sides=['left','right']))))
        with patch('tianji_teleop.recording.hand_command_check.pico_hand_replay_asset_hashes', return_value={}):
            metadata = _session_metadata('pico2_hands_sim', env)
        self.assertEqual(metadata['hand_optimizer']['backend'], 'cpp')
        self.assertEqual(len(metadata['hand_optimizer']['library_sha256']), 64)


if __name__ == '__main__': unittest.main()
