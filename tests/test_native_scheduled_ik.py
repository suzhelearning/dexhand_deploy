from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from tests.test_reference_tjvr_receiver import packet

ROOT=Path(__file__).resolve().parents[1]
IMPORTED=ROOT/'src/tianji_teleop/src/ik/mapped_palm'

@unittest.skipUnless(shutil.which('c++') and (Path(sys.prefix)/'include/mujoco/mujoco.h').is_file()
                     and Path('/usr/include/eigen3/Eigen/Core').is_file(),'C++, MuJoCo and Eigen headers required')
class NativeScheduledIkTest(unittest.TestCase):
    def test_actual_worker_runs_only_in_teleop_and_returns_home(self):
        from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND,MAPPED_PALM_BACKEND
        from tianji_teleop.producers.spark.backend_assets import bilateral_assets
        with tempfile.TemporaryDirectory(prefix='native-scheduled-ik-') as folder:
            binary=Path(folder)/'driver'
            flags=['-O1','-g','-fsanitize=undefined','-fno-omit-frame-pointer'] if os.environ.get('NATIVE_SCHEDULED_IK_UBSAN')=='1' else ['-O2']
            subprocess.run(['c++','-std=c++17','-pthread','-Wall','-Wextra','-Werror',*flags,
                '-I/usr/include/eigen3','-I'+str(IMPORTED/'include'),'-I'+str(Path(sys.prefix)/'include'),
                str(ROOT/'tests/cpp/native_scheduled_ik_driver.cpp'),
                *[str(IMPORTED/'src'/f) for f in ('pico_teleop_protocol.cpp','so3.cpp','pico_mapped_corrected_palm.cpp')],
                '-L'+str(Path(sys.prefix)/'lib'),'-Wl,-rpath,'+str(Path(sys.prefix)/'lib'),'-lmujoco','-o',str(binary)],check=True)
            for prefix,backend in [('spark',SPARK_BACKEND),('mapped_palm',MAPPED_PALM_BACKEND)]:
                with self.subTest(backend=backend):
                    assets=bilateral_assets(ROOT,backend)
                    if not assets['worker'].is_file(): self.skipTest('build native IK workers')
                    result=subprocess.run([str(binary),prefix,backend,'20000',
                        *[str(assets[k]) for k in ('worker','config','model','urdf')],
                        '--startup-handshake','--binary-results','--deterministic-test'],
                        input='\n'.join(packet(i).hex() for i in range(1,101))+'\n',
                        text=True,capture_output=True,timeout=30)
                    self.assertEqual(result.returncode,0,result.stderr)
                    self.assertIn('scheduled_ik_and_simulation_complete',result.stdout)
                    automatic=subprocess.run([str(binary),prefix,backend,'20000',
                        *[str(assets[k]) for k in ('worker','config','model','urdf')],
                        '--startup-handshake','--binary-results','--deterministic-test'],
                        input='\n'.join(packet(i).hex() for i in range(1,101))+'\n',
                        env=dict(os.environ,NATIVE_AUTO_HOME='1'),
                        text=True,capture_output=True,timeout=30)
                    self.assertEqual(automatic.returncode,0,automatic.stderr)
                    self.assertIn('scheduled_ik_and_simulation_complete',automatic.stdout)
            assets=bilateral_assets(ROOT,MAPPED_PALM_BACKEND)
            from tests.test_mapped_palm_height import sample
            result=subprocess.run([str(binary),'mapped_palm',MAPPED_PALM_BACKEND,'20000',
                *[str(assets[k]) for k in ('worker','config','model','urdf')],
                '--startup-handshake','--binary-results','--deterministic-test'],
                input='\n'.join(sample(i,1000000000+i*5000000).observation.frame.raw_packet.hex() for i in range(1,1001))+'\n',
                env=dict(os.environ,NATIVE_IK_HEIGHT='1'),text=True,capture_output=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('scheduled_height_complete',result.stdout)
            spark_assets=bilateral_assets(ROOT,SPARK_BACKEND)
            rejected=subprocess.run([str(binary),'spark',SPARK_BACKEND,'20000',
                *[str(spark_assets[k]) for k in ('worker','config','model','urdf')],
                '--startup-handshake','--binary-results','--deterministic-test'],input=packet(1).hex()+'\n',
                env=dict(os.environ,NATIVE_IK_HEIGHT='1'),text=True,capture_output=True,timeout=20)
            self.assertNotEqual(rejected.returncode,0)
            self.assertIn('height calibration requires mapped-palm reset service',rejected.stderr)
            height_packets='\n'.join(sample(i,1000000000+i*5000000).observation.frame.raw_packet.hex() for i in range(1,1001))+'\n'
            for mode in ('height_gap','height_abort','height_fail','height_cancel'):
                with self.subTest(height=mode):
                    result=subprocess.run([str(binary),'mapped_palm',MAPPED_PALM_BACKEND,'20000',
                        *[str(assets[k]) for k in ('worker','config','model','urdf')]],input=height_packets,
                        env=dict(os.environ,NATIVE_IK_HEIGHT='1',NATIVE_IK_TEST_MODE=mode),text=True,capture_output=True,timeout=20)
                    self.assertEqual(result.returncode,0,result.stderr)
                    self.assertIn('scheduled_height_failure_held',result.stdout)
            for mode in ('cancel','late_return','overflow','wrong_tick','nonfinite','limit','stale','io_failure'):
                with self.subTest(failure=mode):
                    result=subprocess.run([str(binary),'mapped_palm',MAPPED_PALM_BACKEND,'20000',
                        *[str(assets[k]) for k in ('worker','config','model','urdf')]],
                        input=packet(1).hex()+'\n',env=dict(os.environ,NATIVE_IK_TEST_MODE=mode),
                        text=True,capture_output=True,timeout=15)
                    self.assertEqual(result.returncode,0,result.stderr)
                    self.assertIn('scheduled_ik_failure_held',result.stdout)
            for mode in ('reset_ok','reset_cancel','reset_epoch','reset_wrong_epoch','reset_fail','reset_stale','reset_overflow','reset_io_failure'):
                with self.subTest(reset=mode):
                    result=subprocess.run([str(binary),'mapped_palm',MAPPED_PALM_BACKEND,'20000',
                        *[str(assets[k]) for k in ('worker','config','model','urdf')]],
                        input=packet(1).hex()+'\n'+packet(2,epoch=10).hex()+'\n'+packet(2).hex()+'\n',
                        env=dict(os.environ,NATIVE_IK_TEST_MODE=mode),text=True,capture_output=True,timeout=15)
                    self.assertEqual(result.returncode,0,result.stderr)
                    self.assertIn('scheduled_ik_reset_checked',result.stdout)
