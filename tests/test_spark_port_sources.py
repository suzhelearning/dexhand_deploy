"""Source closure/isolation gates before enabling the native SPARK backend."""
import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PORT = ROOT / 'src/tianji_teleop/src/ik/spark_headroom'


class SparkPortSourcesTest(unittest.TestCase):
    def test_pinned_source_closure_and_namespace(self):
        manifest_path = PORT / 'source_manifest.json'
        self.assertTrue(manifest_path.is_file(), 'SPARK source closure has not been migrated')
        manifest = json.loads(manifest_path.read_text())
        self.assertEqual(manifest['source_commit'], 'c022b17789e9b81141915662b3bb802f4c20a396')
        names = {entry['source'] for entry in manifest['files']}
        for source in ('spark_guidance', 'spark_constraint_headroom', 'spark_upper_qpoases_ik',
                       'controller', 'velocity_ik', 'pico_teleop_protocol', 'pico_teleop_session'):
            self.assertIn('src/' + source + '.cpp', names)
        for entry in manifest['files']:
            path = ROOT / entry['destination']
            with self.subTest(path=path):
                self.assertTrue(path.is_file())
                data = path.read_bytes()
                self.assertEqual(hashlib.sha256(data).hexdigest(), entry['destination_sha256'])
                if path.suffix in ('.cpp', '.hpp'):
                    self.assertNotIn('tianji_qp_ik', data.decode())
                    restored = data.decode().replace('tianji_spark', 'tianji_qp_ik').encode()
                    self.assertEqual(hashlib.sha256(restored).hexdigest(), entry['source_sha256'],
                                     'unexpected algorithm edits beyond namespace isolation')

    def test_separate_dependency_lock_does_not_replace_v131(self):
        manifest = ROOT / 'tools/spark_native/pixi.toml'
        self.assertTrue(manifest.is_file(), 'SPARK needs a separate native dependency environment')
        lock = (manifest.parent / 'pixi.lock').read_text()
        self.assertIn('libpinocchio-3.9.0-', lock)
        self.assertIn('libmujoco-3.10.0-', lock)
        self.assertIn('qpoases-3.2.2-', lock)
        self.assertIn('pin = "==4.0.0"', (ROOT / 'pixi.toml').read_text())
