import os
from pathlib import Path
import tempfile
import unittest


class ManusSdkEnvironmentTest(unittest.TestCase):
    def test_standard_sdk_layout_is_scoped_to_child_environment(self):
        from tianji_teleop.hand_tracking.manus_environment import manus_environment
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            library = root / 'ManusSDK/lib/libManusSDK_Integrated.so'
            library.parent.mkdir(parents=True)
            library.touch()
            base = dict(PATH='/bin', LD_LIBRARY_PATH='/existing')
            env, asset = manus_environment(root / 'rawviz.out', base_env=base)
            self.assertEqual(asset, library)
            self.assertEqual(env['LD_LIBRARY_PATH'], str(library.parent) + ':/existing')
            self.assertEqual(base, dict(PATH='/bin', LD_LIBRARY_PATH='/existing'))

    def test_explicit_missing_sdk_is_rejected(self):
        from tianji_teleop.hand_tracking.manus_environment import manus_environment
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'Manus SDK'):
                manus_environment(Path(directory) / 'rawviz', library_dir=directory)

    def test_no_standard_library_preserves_inherited_environment(self):
        from tianji_teleop.hand_tracking.manus_environment import manus_environment
        with tempfile.TemporaryDirectory() as directory:
            env, asset = manus_environment(Path(directory) / 'custom', base_env={'PATH': '/bin'})
            self.assertEqual(env, {'PATH': '/bin'})
            self.assertIsNone(asset)
