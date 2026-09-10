from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]


class WujiNativeEnvironmentTest(unittest.TestCase):
    def test_official_hand_dependencies_are_isolated_and_pinned(self):
        directory = ROOT / 'tools/wuji_hand_native'
        self.assertTrue((directory / 'pixi.toml').is_file(), 'missing hand-only environment')
        lock = yaml.safe_load((directory / 'pixi.lock').read_text())
        urls = [p['conda'] for p in lock['environments']['default']['packages']['linux-64']]
        for filename in ('nlopt-2.10.1-np2py312h0f77346_2.conda',
                         'numpy-2.4.4-py312h33ff503_0.conda',
                         'scipy-1.17.1-py312h54fa4ab_0.conda',
                         'pinocchio-python-4.0.0-py312h0deca05_1.conda',
                         'pyyaml-6.0.3-py312h8a5da7c_1.conda'):
            self.assertTrue(any(url.endswith('/' + filename) for url in urls), filename)
        self.assertFalse(any('/ros-' in url for url in urls))
