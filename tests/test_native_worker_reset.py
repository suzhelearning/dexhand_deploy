from pathlib import Path
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class NativeWorkerResetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert (ROOT/'native/control/worker_reset_client.hpp').is_file(), 'native worker reset transport missing'
        check=subprocess.run(['c++','-std=c++17','-x','c++','-E','-'],input='#include <nlohmann/json.hpp>\n',
                             text=True,capture_output=True)
        if check.returncode: raise unittest.SkipTest('nlohmann-json C++ headers required')
        cls.directory=tempfile.TemporaryDirectory(prefix='native-worker-reset-')
        cls.addClassCleanup(cls.directory.cleanup)
        cls.driver=Path(cls.directory.name)/'driver'
        flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('NATIVE_RESET_SANITIZERS')=='1' else []
        cls.flags=flags
        subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror',*flags,
            str(ROOT/'tests/cpp/native_worker_reset_driver.cpp'),'-o',str(cls.driver)],check=True)

    def fake(self,mode):
        code='''
import json, sys, time, os
print('child_pid='+str(os.getpid()),file=sys.stderr,flush=True)
mode=sys.argv[1]
print(json.dumps(dict(schema_version=1,kind='mapped_palm_worker_ready',algorithm='test',native_ticks=0)),flush=True)
for line in sys.stdin:
    fields=line.split()
    if mode=='timeout': time.sleep(10)
    if mode=='eof': sys.exit(0)
    q=list(map(float,fields[2:]))
    if mode=='wrong_position': q[0]+=0.1
    if mode=='boolean': q[4]=False
    if mode=='oversize': print('x'*5000,flush=True); continue
    ack=dict(schema_version=1,kind='mapped_palm_reset_ack',execution_epoch=int(fields[1]),
             position_rad=q,velocity_rad_s=[0]*14,acceleration_rad_s2=[0]*14)
    if mode=='moving': ack['velocity_rad_s'][0]=0.1
    text=json.dumps(ack)
    if mode=='duplicate': text=text.replace('"schema_version": 1','"schema_version": 1, "schema_version": 1')
    print(text,flush=True)
'''
        return subprocess.run([str(self.driver),'mapped_palm','test','300',sys.executable,'-u','-c',code,mode],
                              capture_output=True,text=True,timeout=5)

    def test_reset_ack_and_transport_failures(self):
        for mode in ('ok','wrong_position','moving','boolean','duplicate','oversize','timeout','eof'):
            with self.subTest(mode=mode):
                result=self.fake(mode)
                self.assertEqual(result.returncode,0 if mode=='ok' else 1,result.stderr)
                if mode=='ok': self.assertIn('reset_epoch=3',result.stdout)
                else: self.assertIn('failure_epoch=1; transport_closed',result.stderr)
                pid=re.search(r'child_pid=(\d+)',result.stderr)
                self.assertIsNotNone(pid,result.stderr)
                self.assertFalse(Path('/proc',pid.group(1)).exists(),'owned worker was not reaped')

    def test_actual_native_workers(self):
        from tianji_teleop.producers.spark.backend_assets import bilateral_assets
        from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND,MAPPED_PALM_BACKEND
        for prefix,backend in (('spark',SPARK_BACKEND),('mapped_palm',MAPPED_PALM_BACKEND)):
            assets=bilateral_assets(ROOT,backend)
            if not assets['worker'].is_file(): self.skipTest('build both native IK workers')
            result=subprocess.run([str(self.driver),prefix,backend,'20000',
                *[str(assets[key]) for key in ('worker','config','model','urdf')],'--startup-handshake'],
                capture_output=True,text=True,timeout=60)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('reset_epoch=3',result.stdout)

    def test_spawn_failure_and_wrong_handshake(self):
        for command in ([str(Path(self.directory.name)/'missing-worker')],
                        [sys.executable,'-u','-c',"print('{}', flush=True)"]):
            result=subprocess.run([str(self.driver),'mapped_palm','test','300',*command],
                                  capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,1,result.stderr)
            self.assertNotIn('reset_epoch=',result.stdout)

    def test_cancel_interrupts_real_ipc_wait(self):
        binary=Path(self.directory.name)/'cancel'
        subprocess.run(['c++','-std=c++17','-pthread','-O2','-Wall','-Wextra','-Werror',*self.flags,
            str(ROOT/'tests/cpp/test_worker_reset_cancel.cpp'),'-o',str(binary)],check=True)
        code="import json,time,sys; print(json.dumps(dict(schema_version=1,kind='mapped_palm_worker_ready',algorithm='test',native_ticks=0)),flush=True); sys.stdin.readline(); time.sleep(10)"
        result=subprocess.run([str(binary),sys.executable,'-u','-c',code],text=True,capture_output=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('cancelled_without_epoch_commit',result.stdout)
