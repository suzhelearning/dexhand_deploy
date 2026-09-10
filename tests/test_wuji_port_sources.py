import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class WujiPortSourceTest(unittest.TestCase):
    def test_reference_sources_and_assets_are_byte_identical(self):
        directory = ROOT / 'third_party/wuji_hand_retargeting'
        manifest = directory / 'source_manifest.json'
        self.assertTrue(manifest.is_file(), 'official hand source closure is missing')
        entries = json.loads(manifest.read_text())
        self.assertEqual(entries['source_commit'], '5875d0b9d1557a4eca118387cf79e99f3c14cb1d')
        required = {'wuji_retargeting/opt/adaptive_analytical.py',
                    'example/tj_wuji2_hand_bridge.py', 'LICENSE'}
        self.assertTrue(required.issubset({e['destination'] for e in entries['files']}))
        for entry in entries['files']:
            actual = hashlib.sha256((directory / entry['destination']).read_bytes()).hexdigest()
            self.assertEqual(actual, entry['sha256'], entry['destination'])
