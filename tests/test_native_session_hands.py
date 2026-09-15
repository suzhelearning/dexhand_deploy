from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import sys
import os

ROOT=Path(__file__).resolve().parents[1]

@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class NativeSessionHandsTest(unittest.TestCase):
    @unittest.skipUnless((Path(sys.prefix)/'include/mujoco/mujoco.h').is_file() and
                         (ROOT/'tools/wuji_hand_native/.pixi/envs/default/bin/python').is_file() and
                         (ROOT/'build/hand-native/tianji_hand_native_scheduler').is_file(),
                         'MuJoCo headers and pinned native Hand2 runtime required')
    def test_joint_reset_waits_for_actual_hand_child_and_fails_closed(self):
        from tianji_teleop.worker_environment import isolated_worker_environment
        with tempfile.TemporaryDirectory(prefix='native-joint-hand-reset-') as folder:
            binary=Path(folder)/'reset'
            flags=['-O1','-g','-fsanitize=undefined','-fno-sanitize-recover=undefined','-fno-omit-frame-pointer'] if os.environ.get('NATIVE_HAND_DOMAIN_UBSAN')=='1' else ['-O2']
            result=subprocess.run(['c++','-std=c++17','-pthread',*flags,'-Wall','-Wextra','-Werror',
                '-I'+str(Path(sys.prefix)/'include'),str(ROOT/'tests/cpp/native_session_hand_reset.cpp'),
                '-L'+str(Path(sys.prefix)/'lib'),'-Wl,-rpath,'+str(Path(sys.prefix)/'lib'),'-lmujoco',
                '-o',str(binary)],text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            launcher=[str(ROOT/'tools/wuji_hand_native/.pixi/envs/default/bin/python'),
                str(ROOT/'scripts/wuji_hand_native_scheduler_launcher.py'),
                '--native-scheduler',str(ROOT/'build/hand-native/tianji_hand_native_scheduler'),
                '--startup-handshake']
            for mode in ('ok','arm_fail','hand_fail','cancel','missing','source_loss'):
                command=launcher if mode in ('ok','arm_fail','missing') else [sys.executable,
                    str(ROOT/'tests/native_hand_reset_fake_worker.py'),'epoch' if mode=='hand_fail' else 'cancel']
                result=subprocess.run([str(binary),str(ROOT/'src/tianji_teleop/assets/spark/marvin_m6_wuji2.xml'),mode,*command],
                    env=isolated_worker_environment(),text=True,capture_output=True,timeout=15)
                self.assertEqual(result.returncode,0,result.stderr)

            from tianji_teleop.producers.spark.backend_assets import bilateral_assets
            from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND, MAPPED_PALM_BACKEND
            for prefix,backend in (('spark',SPARK_BACKEND),('mapped_palm',MAPPED_PALM_BACKEND)):
                assets=bilateral_assets(ROOT,backend)
                result=subprocess.run([str(binary),str(assets['model']),'ok',*launcher,
                    '--arm-worker',prefix,backend,*[str(assets[key]) for key in ('worker','config','model','urdf')],
                    '--startup-handshake'],env=isolated_worker_environment(),text=True,capture_output=True,timeout=35)
                self.assertEqual(result.returncode,0,result.stderr)

    @unittest.skipUnless((Path(sys.prefix)/'include/mujoco/mujoco.h').is_file(), 'MuJoCo headers required')
    def test_runtime_owns_hand_application_and_home_fault_fences(self):
        with tempfile.TemporaryDirectory(prefix='native-session-hand-runtime-') as folder:
            binary=Path(folder)/'runtime'
            flags=['-O1','-g','-fsanitize=undefined','-fno-sanitize-recover=undefined','-fno-omit-frame-pointer'] if os.environ.get('NATIVE_HAND_DOMAIN_UBSAN')=='1' else ['-O2']
            result=subprocess.run(['c++','-std=c++17','-pthread',*flags,'-Wall','-Wextra','-Werror',
                '-I'+str(Path(sys.prefix)/'include'),str(ROOT/'tests/cpp/native_session_hands_runtime.cpp'),
                '-L'+str(Path(sys.prefix)/'lib'),'-Wl,-rpath,'+str(Path(sys.prefix)/'lib'),'-lmujoco',
                '-o',str(binary)],text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            for model in ('spark','mapped_palm'):
                for mode in ('home','fault','stale','scheduler'):
                    with self.subTest(model=model,mode=mode):
                        result=subprocess.run([str(binary),str(ROOT/f'src/tianji_teleop/assets/{model}/marvin_m6_wuji2.xml'),mode],
                            text=True,capture_output=True,timeout=10)
                        self.assertEqual(result.returncode,0,result.stderr)

    def test_input_association_phase_epoch_freshness_and_fail_closed(self):
        with tempfile.TemporaryDirectory(prefix='native-session-hands-') as folder:
            binary=Path(folder)/'gates'
            flags=['-O1','-g','-fsanitize=undefined','-fno-sanitize-recover=undefined','-fno-omit-frame-pointer'] if os.environ.get('NATIVE_HAND_DOMAIN_UBSAN')=='1' else ['-O2']
            result=subprocess.run(['c++','-std=c++17','-pthread',*flags,'-Wall','-Wextra','-Werror',
                str(ROOT/'tests/cpp/test_session_hand_domain.cpp'),'-o',str(binary)],text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(binary)],text=True,capture_output=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
