"""Offline calibration finalization tests; never acquire hardware."""
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1] / 'vendor/pico_tracker/src/pico_bridge/scripts'


class FinalizeTest(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(SCRIPTS))
        self.addCleanup(lambda: sys.path.remove(str(SCRIPTS)))
        import pico_calibration_finalize
        self.module = pico_calibration_finalize
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.source = Path(self.temp.name) / 'calibration'
        self.source.mkdir()

    def populate(self):
        for name in self.module.FILES:
            (self.source / name).write_text('original measurement')

    def test_original_and_pending_do_not_write(self):
        self.assertEqual(self.module.finalize(self.source, 'original')['state'], 'original')
        self.assertEqual(self.module.finalize(self.source, 'symmetric_max')['state'], 'pending')
        self.assertEqual(list(self.source.iterdir()), [])

    def test_invalid_calibration_preserves_old_selection(self):
        self.populate()
        with patch.object(self.module, 'create_profile', side_effect=ValueError('gate failed')):
            with self.assertRaisesRegex(ValueError, 'gate failed'):
                self.module.finalize(self.source, 'symmetric_max')
        self.assertFalse((self.source / 'runtime_symmetric').exists())

    def test_versions_are_preserved_and_link_updates(self):
        self.populate()
        def create(source, output):
            output.mkdir(parents=True)
            (output / 'pico_geometry_policy.json').write_text('{}')
            return {'effective_lengths_m': {'upper_arm': .2763, 'forearm': .2475}}
        with patch.object(self.module, 'create_profile', side_effect=create):
            first = self.module.finalize(self.source, 'symmetric_max')
            previous = (self.source / 'runtime_symmetric').resolve()
            second = self.module.finalize(self.source, 'symmetric_max')
            self.assertTrue(previous.exists())
            self.assertNotEqual(previous, (self.source / 'runtime_symmetric').resolve())
            self.assertEqual(first['state'], 'ready')
            self.assertEqual(second['state'], 'ready')
            with patch.object(self.module, 'create_profile', side_effect=ValueError('gate')):
                current = (self.source / 'runtime_symmetric').resolve()
                with self.assertRaises(ValueError):self.module.finalize(self.source, 'symmetric_max')
                self.assertEqual(current, (self.source / 'runtime_symmetric').resolve())
        for name in self.module.FILES:
            self.assertEqual((self.source / name).read_text(), 'original measurement')

    def test_refuses_foreign_paths_and_derived_input(self):
        self.populate()
        link = self.source / 'runtime_symmetric'
        link.mkdir()
        with self.assertRaises(ValueError):self.module.finalize(self.source, 'symmetric_max')
        link.rmdir()
        link.symlink_to(self.source.parent)
        with self.assertRaises(ValueError):self.module.finalize(self.source, 'symmetric_max')
        (self.source / 'pico_geometry_policy.json').write_text('{}')
        with self.assertRaises(ValueError):self.module.finalize(self.source, 'original')


if __name__ == '__main__':unittest.main()
