import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('c++'),'C++ compiler required')
class NativeSessionRearmTest(unittest.TestCase):
    def compile(self,source,output):
        flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('NATIVE_RESET_SANITIZERS')=='1' else []
        subprocess.run(['c++','-std=c++17','-pthread','-O2','-Wall','-Wextra','-Werror',*flags,
            str(ROOT/'tests/cpp'/source),'-o',str(output)],check=True)

    def test_transaction_guards_and_cancellation(self):
        with tempfile.TemporaryDirectory(prefix='native-rearm-') as directory:
            binary=Path(directory)/'test'
            self.compile('test_session_rearm.cpp',binary)
            subprocess.run([str(binary)],check=True,timeout=10)

    def test_actual_worker_session_transaction(self):
        check=subprocess.run(['c++','-x','c++','-E','-'],input='#include <nlohmann/json.hpp>\n',text=True,capture_output=True)
        if check.returncode: self.skipTest('nlohmann-json headers required')
        from tianji_teleop.producers.spark.backend_assets import bilateral_assets
        from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND,MAPPED_PALM_BACKEND
        with tempfile.TemporaryDirectory(prefix='native-rearm-worker-') as directory:
            binary=Path(directory)/'driver'
            self.compile('native_session_rearm_driver.cpp',binary)
            for prefix,backend in (('spark',SPARK_BACKEND),('mapped_palm',MAPPED_PALM_BACKEND)):
                assets=bilateral_assets(ROOT,backend)
                if not assets['worker'].is_file(): self.skipTest('build both native workers')
                result=subprocess.run([str(binary),prefix,backend,'20000',
                    *[str(assets[key]) for key in ('worker','config','model','urdf')],'--startup-handshake'],
                    text=True,capture_output=True,timeout=60)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertIn('session_epoch=2; new_input_required',result.stdout)
