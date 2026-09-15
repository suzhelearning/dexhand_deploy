from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]

@unittest.skipUnless(shutil.which('c++') and (Path(sys.prefix)/'include/mujoco/mujoco.h').is_file() and
                     (ROOT/'tools/wuji_hand_native/.pixi/envs/default/bin/python').is_file() and
                     (ROOT/'build/hand-native/tianji_hand_native_scheduler').is_file(),
                     'compiler, MuJoCo headers and pinned native Hand2 runtime required')
class NativeHandPipelineTest(unittest.TestCase):
    def test_cancel_reaps_owned_child_without_waiting_for_endpoint_destruction(self):
        with tempfile.TemporaryDirectory(prefix='native-hand-pipeline-close-') as folder:
            binary=Path(folder)/'close'
            flags=['-O1','-g','-fsanitize=undefined','-fno-sanitize-recover=undefined'] if os.environ.get('NATIVE_HAND_DOMAIN_UBSAN')=='1' else ['-O2']
            result=subprocess.run(['c++','-std=c++17','-pthread',*flags,'-Wall','-Wextra','-Werror',
                str(ROOT/'tests/cpp/native_hand_pipeline_lifecycle.cpp'),'-o',str(binary)],text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result=subprocess.run([str(binary),sys.executable,str(ROOT/'tests/native_hand_reset_fake_worker.py'),'cancel'],
                text=True,capture_output=True,timeout=8)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_real_child_input_phase_reset_and_owned_simulation(self):
        from tianji_teleop.worker_environment import isolated_worker_environment
        with tempfile.TemporaryDirectory(prefix='native-hand-pipeline-') as folder:
            binary=Path(folder)/'pipeline'
            flags=['-O1','-g','-fsanitize=undefined','-fno-sanitize-recover=undefined'] if os.environ.get('NATIVE_HAND_DOMAIN_UBSAN')=='1' else ['-O2']
            result=subprocess.run(['c++','-std=c++17','-pthread',*flags,'-Wall','-Wextra','-Werror',
                '-I'+str(Path(sys.prefix)/'include'),str(ROOT/'tests/cpp/native_session_hand_pipeline.cpp'),
                str(ROOT/'native/hand/manus_input.cpp'),
                '-L'+str(Path(sys.prefix)/'lib'),'-Wl,-rpath,'+str(Path(sys.prefix)/'lib'),'-lmujoco','-o',str(binary)],
                text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            launcher=[str(ROOT/'tools/wuji_hand_native/.pixi/envs/default/bin/python'),
                str(ROOT/'scripts/wuji_hand_native_scheduler_launcher.py'),
                '--native-scheduler',str(ROOT/'build/hand-native/tianji_hand_native_scheduler'),'--startup-handshake']
            for model in ('spark','mapped_palm'):
                for mode in ('home','rawviz','generation','nonfinite','stall','overflow','reset_cancel','capture_overflow','auto_home'):
                    with self.subTest(model=model,mode=mode):
                        command=launcher if mode in ('home','rawviz','generation','nonfinite','auto_home') else [
                            sys.executable,str(ROOT/'tests/native_hand_reset_fake_worker.py'),'cancel']
                        from tests.test_reference_manus_process import rawviz_records
                        from tests.test_gesture_recognition import hand_points
                        data='\f'.join(rawviz_records('right',i,canonical_points=hand_points())+
                                       rawviz_records('left',i,canonical_points=hand_points())
                                       for i in range(1,5)) if mode=='rawviz' else ''
                        result=subprocess.run([str(binary),str(ROOT/f'src/tianji_teleop/assets/{model}/marvin_m6_wuji2.xml'),
                            mode,*command],input=data,env=isolated_worker_environment(),text=True,capture_output=True,timeout=20)
                        self.assertEqual(result.returncode,0,result.stderr)
