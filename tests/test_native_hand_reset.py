from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import sys

ROOT=Path(__file__).resolve().parents[1]

@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class NativeHandResetTest(unittest.TestCase):
    def test_cpp_child_wrong_ack_timeout_truncation_and_cancel_fail_closed(self):
        with tempfile.TemporaryDirectory(prefix='native-hand-faults-') as folder:
            binary=Path(folder)/'faults'
            result=subprocess.run(['c++','-std=c++17','-pthread','-O2','-Wall','-Wextra','-Werror',
                str(ROOT/'tests/cpp/native_hand_client_faults.cpp'),'-o',str(binary)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            for mode in ('epoch','sequence','future','requested','backward','flags','truncated','timeout','cancel'):
                with self.subTest(mode=mode):
                    result=subprocess.run([str(binary),sys.executable,str(ROOT/'tests/native_hand_reset_fake_worker.py'),mode],
                        capture_output=True,text=True,timeout=10)
                    self.assertEqual(result.returncode,0,result.stderr)

    @unittest.skipUnless((ROOT/'tools/wuji_hand_native/.pixi/envs/default/bin/python').is_file() and
                         (ROOT/'build/hand-native/tianji_hand_native_scheduler').is_file(),
                         'pinned native hand runtime required')
    def test_actual_child_cpp_transport_reset_and_first_result_parity(self):
        from tianji_teleop.worker_environment import isolated_worker_environment
        with tempfile.TemporaryDirectory(prefix='native-hand-client-') as folder:
            binary=Path(folder)/'client'
            result=subprocess.run(['c++','-std=c++17','-pthread','-O2','-Wall','-Wextra','-Werror',
                str(ROOT/'tests/cpp/native_hand_client_driver.cpp'),'-o',str(binary)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(binary),str(ROOT/'tools/wuji_hand_native/.pixi/envs/default/bin/python'),
                str(ROOT/'scripts/wuji_hand_native_scheduler_launcher.py'),
                '--native-scheduler',str(ROOT/'build/hand-native/tianji_hand_native_scheduler'),
                '--freshness-ns','5000000000','--startup-handshake'],capture_output=True,text=True,
                env=isolated_worker_environment(),timeout=20)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_reset_ack_waits_for_both_callbacks_and_has_strict_wire(self):
        with tempfile.TemporaryDirectory(prefix='hand-reset-ack-') as folder:
            binary=Path(folder)/'ack'
            result=subprocess.run(['c++','-std=c++17','-pthread','-O2','-Wall','-Wextra','-Werror',
                str(ROOT/'tests/cpp/test_hand_reset_ack.cpp'),'-o',str(binary)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(binary)],capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
