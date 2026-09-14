import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
PORT = ROOT / 'src/tianji_teleop/src/ik/mapped_palm'


class MappedPalmPortTest(unittest.TestCase):
    def test_pinned_source_closure(self):
        manifest = json.loads((PORT / 'source_manifest.json').read_text())
        self.assertEqual(manifest['commit'], '2bcfe09e2c78a48c7ba63943deef83d04a139ff8')
        self.assertGreater(len(manifest['files']), 50)
        for row in manifest['files']:
            self.assertEqual(hashlib.sha256((ROOT / row['destination']).read_bytes()).hexdigest(),
                             row['sha256'], row['destination'])

    def test_bandwidth_configuration_is_preserved(self):
        import yaml
        config = yaml.safe_load((PORT / 'config/bandwidth.yaml').read_text())
        self.assertEqual(config['spark_feedforward_velocity_qp']['position_feedforward_gain'], 1.0)
        self.assertEqual(config['spark_feedforward_velocity_qp']['orientation_feedforward_gain'], .8)
        self.assertEqual(config['hierarchical_qp']['smoothness_mode'], 'normalized_split')
        self.assertFalse(config['cartesian_otg']['enabled'])
