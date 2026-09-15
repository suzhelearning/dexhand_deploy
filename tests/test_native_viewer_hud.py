from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]

class ViewerHudTest(unittest.TestCase):
    def test_calibration_states_and_fault(self):
        with tempfile.TemporaryDirectory() as folder:
            binary=Path(folder)/'hud'
            subprocess.run(['c++','-std=c++17','-O1','-pthread','-Wall','-Wextra','-Werror',
                str(ROOT/'tests/cpp/viewer_hud_fixture.cpp'),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)
