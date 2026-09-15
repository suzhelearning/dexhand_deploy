import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/run_pico_vr_arms_cpp.sh'


class ShortcutTest(unittest.TestCase):
    def run_script(self, *args, env=None):
        return subprocess.run(['bash', str(SCRIPT), *args], cwd='/tmp',
                              env=env, capture_output=True, text=True, timeout=10)

    def test_preview_has_exact_tested_arm_only_route(self):
        result = self.run_script('--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = shlex.split(result.stdout.splitlines()[-1])
        expected = {'--profile': 'pico_vr_manus_sim', '--ik-backend': 'pico_ee_mapped_corrected_palm_velocity_qp',
                    '--scheduler-backend': 'cpp', '--publication-backend': 'cpp',
                    '--recording-adapter': 'cpp', '--viewer-backend': 'cpp',
                    '--pico-world-x-offset-m': '0.20', '--joint-limit-source': 'urdf'}
        for key, value in expected.items():self.assertEqual(args[args.index(key)+1], value)
        for flag in ['--viewer', '--disable-hands', '--mapped-palm-xz-calibration']:
            self.assertIn(flag, args)
        for flag in ['--manus-rawviz', '--pico-calibration-dir', '--mapped-palm-common-x-reference']:
            self.assertNotIn(flag, args)

    def test_preview_is_read_only_and_recording_names_are_unique(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'space in path'
            first = self.run_script('--dry-run', '--record-dir', str(output))
            second = self.run_script('--dry-run', '--record-dir', str(output))
            self.assertEqual(first.returncode, 0, first.stderr)
            self.assertNotEqual(first.stdout, second.stdout)
            self.assertFalse(output.exists())
            args = shlex.split(first.stdout.splitlines()[-1])
            self.assertEqual(Path(args[args.index('--record')+1]).parent, output)

    def test_help_and_invalid_options_do_not_start_session(self):
        self.assertEqual(self.run_script('--help').returncode, 0)
        self.assertEqual(self.run_script('--record-dir').returncode, 2)
        self.assertEqual(self.run_script('--profile', 'real').returncode, 2)

    def test_execution_delegates_to_existing_session_and_preserves_exit_status(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake = root / 'pixi'
            fake.write_text('#!/bin/sh\nprintf "%s\\n" "$TIANJI_ROUTER_ENDPOINT" "$PWD" "$@"\nexit 17\n')
            fake.chmod(0o700)
            env = dict(os.environ, PATH=str(root)+':'+os.environ['PATH'],
                       TIANJI_ROUTER_ENDPOINT='tcp/127.0.0.1:7447')
            result = self.run_script('--record-dir', str(root/'records'), env=env)
            self.assertEqual(result.returncode, 17, result.stderr)
            self.assertIn(str(ROOT), result.stdout)
            self.assertIn('scripts/run_session.sh', result.stdout)
            self.assertTrue((root/'records').is_dir())


if __name__ == '__main__':unittest.main()
