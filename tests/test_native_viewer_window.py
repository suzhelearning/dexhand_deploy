import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]

class ViewerWindowTest(unittest.TestCase):
    def test_keys_and_explicit_no_display_failure(self):
        self.assertTrue((ROOT/'native/control/viewer_window.hpp').is_file())
        with tempfile.TemporaryDirectory() as directory:
            binary=Path(directory)/'window'
            prefix=Path(sys.prefix)
            imported=ROOT/'src/tianji_teleop/src/ik/mapped_palm'
            subprocess.run(['c++','-std=c++17','-O2','-pthread','-Wall','-Wextra','-Werror',
                '-I'+str(prefix/'include'),'-I/usr/include/eigen3','-I'+str(imported/'include'),
                str(ROOT/'tests/cpp/viewer_window_fixture.cpp'),
                str(imported/'src/pico_teleop_protocol.cpp'),str(imported/'src/so3.cpp'),
                '-L'+str(prefix/'lib'),'-Wl,-rpath,'+str(prefix/'lib'),'-lmujoco','-lglfw','-o',str(binary)],check=True)
            env=dict(os.environ,DISPLAY='',WAYLAND_DISPLAY='')
            result=subprocess.run([str(binary),str(ROOT/'src/tianji_teleop/assets/mapped_palm/marvin_m6_wuji2.xml')],
                                  env=env,text=True,capture_output=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
