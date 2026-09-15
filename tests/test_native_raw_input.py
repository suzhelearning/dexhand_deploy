from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest
from tests.test_reference_tjvr_receiver import packet
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver

ROOT=Path(__file__).resolve().parents[1]
IMPORTED=ROOT/'src/tianji_teleop/src/ik/mapped_palm'

def compile_driver(source,binary):
    flags=['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie'] if os.environ.get('NATIVE_RAW_SANITIZERS')=='1' else ['-O2']
    subprocess.run(['c++','-std=c++17','-pthread',*flags,'-Wall','-Wextra','-Werror',
        '-I/usr/include/eigen3','-I'+str(IMPORTED/'include'),str(ROOT/'tests/cpp'/source),
        *[str(IMPORTED/'src'/name) for name in ('pico_teleop_protocol.cpp','so3.cpp','pico_mapped_corrected_palm.cpp')],
        '-o',str(binary)],check=True)

@unittest.skipUnless(shutil.which('c++') and Path('/usr/include/eigen3/Eigen/Core').is_file(),'C++ compiler and Eigen headers required')
class NativeRawInputTest(unittest.TestCase):
    def test_successful_and_rejected_packets_keep_recording_ordinals(self):
        packets = [packet(1), packet(1), b'bad', packet(2)]
        with tempfile.TemporaryDirectory(prefix='native-ingress-audit-') as directory:
            binary = Path(directory) / 'driver'
            compile_driver('native_raw_input_driver.cpp', binary)
            for mode in ('packet', 'mapped'):
                result = subprocess.run([str(binary), mode, 'metadata'],
                    input='\n'.join(p.hex() for p in packets) + '\n',
                    capture_output=True, text=True, check=True, timeout=10)
                self.assertEqual([list(map(int, line.split())) for line in result.stdout.splitlines()],
                                 [[1, 1, 1], [0, 1, 2], [0, 0, 3], [1, 1, 4]])

    def test_real_worker_reset_and_new_raw_frame(self):
        check=subprocess.run(['c++','-x','c++','-E','-'],input='#include <nlohmann/json.hpp>\n',text=True,capture_output=True)
        if check.returncode: self.skipTest('nlohmann-json headers required')
        from tianji_teleop.producers.spark.backend_assets import bilateral_assets
        from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND,MAPPED_PALM_BACKEND
        with tempfile.TemporaryDirectory(prefix='native-raw-rearm-') as directory:
            binary=Path(directory)/'driver'
            compile_driver('native_raw_rearm_driver.cpp',binary)
            for prefix,backend in (('spark',SPARK_BACKEND),('mapped_palm',MAPPED_PALM_BACKEND)):
                assets=bilateral_assets(ROOT,backend)
                if not assets['worker'].is_file(): self.skipTest('build both native workers')
                for packets in ([packet(1),packet(2)],
                                [packet(1),packet(2,epoch=10),packet(3,epoch=10)]):
                    with self.subTest(backend=backend,epoch_change=len(packets)==3):
                        result=subprocess.run([str(binary),prefix,backend,'20000',
                            *[str(assets[key]) for key in ('worker','config','model','urdf')],'--startup-handshake'],
                            input='\n'.join(p.hex() for p in packets)+'\n',text=True,capture_output=True,timeout=60)
                        self.assertEqual(result.returncode,0,result.stderr)
                        self.assertIn('raw_barrier_released_by_new_frame',result.stdout)

    def test_reference_stream_decisions(self):
        packets=[packet(1),packet(1),b'bad',packet(2,10.3),packet(3,10.31),packet(4,10.32),
                 packet(5,epoch=10),packet(6,epoch=9),packet(7,epoch=10)]
        with tempfile.TemporaryDirectory(prefix='native-tjvr-') as directory:
            binary=Path(directory)/'driver'
            compile_driver('native_raw_input_driver.cpp',binary)
            for mapped in (False,True):
                reference=ReferenceTjvrReceiver('source',.15,.6,target_source='mapped_corrected_palm' if mapped else 'packet')
                expected=[]
                epoch=generation=0
                for i,p in enumerate(packets):
                    decision=reference.ingest(p,100+i)
                    accepted=bool(decision and decision.accepted)
                    if accepted:
                        latest=reference.try_read_latest()
                        epoch=latest.observation.frame.tracking_epoch
                        generation=latest.resynchronization_generation
                    expected.append([int(accepted),epoch,generation])
                result=subprocess.run([str(binary),'mapped' if mapped else 'packet'],input='\n'.join(p.hex() for p in packets)+'\n',
                    capture_output=True,text=True,check=True,timeout=10)
                self.assertEqual([list(map(int,line.split())) for line in result.stdout.splitlines()],expected)
