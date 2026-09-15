from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]

@unittest.skipUnless(shutil.which('c++') and (Path(sys.prefix)/'include/mujoco/mujoco.h').is_file(),
                     'C++ and MuJoCo development headers required')
class NativeRuntimeSimulationTest(unittest.TestCase):
    def test_owned_thread_execution_home_and_failure_cleanup(self):
        with tempfile.TemporaryDirectory(prefix='native-runtime-sim-') as folder:
            binary=Path(folder)/'driver'
            flags=['-O1','-g','-fsanitize=undefined','-fno-omit-frame-pointer'] if os.environ.get('NATIVE_RUNTIME_SIM_UBSAN')=='1' else ['-O2']
            subprocess.run(['c++','-std=c++17','-pthread','-Wall','-Wextra','-Werror',*flags,
                '-I'+str(Path(sys.prefix)/'include'),str(ROOT/'tests/cpp/native_runtime_sim_driver.cpp'),
                '-L'+str(Path(sys.prefix)/'lib'),'-Wl,-rpath,'+str(Path(sys.prefix)/'lib'),'-lmujoco','-o',str(binary)],check=True)
            for model in ('spark','mapped_palm'):
                for mode in ('home','interrupt','stop_loading','load_fail','null_fail','apply_fail','feedback_fail'):
                    with self.subTest(model=model,mode=mode):
                        result=subprocess.run([str(binary),str(ROOT/f'src/tianji_teleop/assets/{model}/marvin_m6_wuji2.xml'),mode],
                            text=True,capture_output=True,timeout=10)
                        self.assertEqual(result.returncode,0,result.stderr)
                        self.assertIn('fault_cleaned' if mode.endswith('fail') else 'native_thread_simulation_complete',result.stdout)
