"""Simulation activation must not require the physical hand SDK bundle."""
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class SimulationRuntimeActivationTest(unittest.TestCase):
    def activate(self, capability):
        with tempfile.TemporaryDirectory() as directory:
            return subprocess.run(
                ['bash', '-c', 'source scripts/common.sh\n'
                 'BUNDLE_ROOT="$1"\nZENOH_LIBRARY_ROOT="$1/missing-zenoh"\n'
                 'TIANJI_REQUIRED_CAPABILITY="$2"\nactivate_bundle_runtime',
                 'test', directory, capability], cwd=ROOT, capture_output=True, text=True,
                timeout=10)

    def test_simulation_uses_environment_without_vendor_sdk(self):
        result = self.activate('simulation')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_real_still_requires_vendor_bundle(self):
        result = self.activate('real')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('vendor/python', result.stderr)

    def test_launcher_shell_syntax(self):
        result = subprocess.run(['bash', '-n', 'scripts/run_session.sh'], cwd=ROOT)
        self.assertEqual(result.returncode, 0)
