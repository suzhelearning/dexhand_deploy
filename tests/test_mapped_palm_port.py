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

    def test_only_approved_position_penalty_differs_from_original_config(self):
        import yaml
        text = (PORT / 'config/bandwidth.yaml').read_text()
        config = yaml.safe_load(text)
        self.assertEqual(float(config['hierarchical_qp']['slack_weight_position']), 90000.)
        restored = text.replace(
            '  # Local tuning: tripled position tracking penalty; imported baseline was 3.0e4.\n'
            '  slack_weight_position: 9.0e4\n', '  slack_weight_position: 3.0e4\n')
        self.assertEqual(hashlib.sha256(restored.encode()).hexdigest(),
                         'c6a428da9d9d1e8381726355ad6aa1b6a0de63e0c7943cddd7df8473d45b084c')
