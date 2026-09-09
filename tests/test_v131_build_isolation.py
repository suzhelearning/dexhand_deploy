"""Build/deploy contract plus an optional actual legacy ELF verification."""
import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class V131BuildIsolationTest(unittest.TestCase):
    def test_portable_build_explicitly_excludes_v131(self):
        self.assertIn('-DTIANJI_ENABLE_V131=OFF', (ROOT / 'scripts/build_ik.sh').read_text())

    def test_simulation_build_explicitly_enables_v131(self):
        self.assertIn('-DTIANJI_ENABLE_V131=ON', (ROOT / 'scripts/build_ik_sim.sh').read_text())

    def test_portable_deploy_rejects_local_v131_before_writes(self):
        source = (ROOT / 'scripts/deploy_ik_runtime.sh').read_text()
        self.assertIn('libmujoco|libqpOASES', source)
        self.assertLess(source.index('libmujoco|libqpOASES'), source.index('shopt -s nullglob'))

    def test_deploy_guard_executes_before_missing_sdk_or_cleanup(self):
        binary = ROOT / 'build/ik-sim/arm_ik_producer'
        if not binary.is_file():
            self.skipTest('build the v131-enabled simulation producer first')
        # Isolated bundle: never execute the deploy script against the workspace.
        with tempfile.TemporaryDirectory(prefix='v131-deploy-guard-') as directory:
            bundle = Path(directory)
            script = bundle / 'scripts/deploy_ik_runtime.sh'
            script.parent.mkdir()
            shutil.copyfile(ROOT / 'scripts/deploy_ik_runtime.sh', script)
            staged = bundle / 'staging/ik/lib/tianji_teleop/arm_ik_producer'
            staged.parent.mkdir(parents=True)
            shutil.copyfile(binary, staged)
            sentinel = staged.parent / 'must-not-be-cleaned'
            sentinel.write_text('preserve')
            result = subprocess.run(['bash', str(script)], text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('staging 含本机 v131 依赖', result.stderr)
            self.assertEqual(sentinel.read_text(), 'preserve')
            self.assertFalse((bundle / 'runtime').exists())
            self.assertFalse((bundle / 'staging/runtime-backup').exists())

    @unittest.skipUnless(os.environ.get('TIANJI_LEGACY_IK_BINARY'), 'set path to v131-disabled build')
    def test_legacy_elf_has_no_v131_dependencies_or_factory(self):
        binary = os.environ['TIANJI_LEGACY_IK_BINARY']
        dynamic = subprocess.check_output(['readelf', '-d', binary], text=True)
        self.assertNotIn('libmujoco', dynamic)
        self.assertNotIn('libqpOASES', dynamic)
        symbols = subprocess.check_output(['nm', '-C', binary], text=True)
        self.assertNotIn('DexhandQpArmIk', symbols)
        self.assertIn('PinocchioQpArmIk', symbols)
