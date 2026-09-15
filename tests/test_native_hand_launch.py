import json
from pathlib import Path
import subprocess
import os
import sys
from types import SimpleNamespace
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]

def manifest():
    identity=lambda name:dict(logical=name,instance=name+'-instance',router='router')
    return dict(run_id='run',router_zid='router',publication_backend='cpp',viewer_backend='cpp',
        diagnostic_transport='summary',manus_source_authority=identity('manus'),
        hand_authorities=dict(producer=identity('hand'),left=identity('left'),right=identity('right')),
        hand_runtime=dict(stdout_fd=10,generation=1,worker_command=['/fixture/worker','--startup-handshake'],
            timeout_ms=2000,capacity=256,age_ns=200000000,right_glove='',left_glove='',
            lower=[[-1.]*20]*2,upper=[[1.]*20]*2,zero=[[0.]*20]*2,zero_tolerance=[[.01]*20]*2))

class NativeHandLaunchTest(unittest.TestCase):
    def test_python_adapter_requires_hand_capability(self):
        from tianji_teleop.producers.spark.native_gateway import NativeGatewayProcess
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'manifest.json';path.write_text('{}')
            raw_read,raw_write=os.pipe()
            try:
                for bad_fd in (raw_write, True):
                    with self.assertRaises(ValueError):
                        NativeGatewayProcess(Path(sys.executable),path,prefix='spark',publication_backend='cpp',
                            viewer_backend='cpp',diagnostic_transport='summary',manus_fd=bad_fd)
                with self.assertRaises(ValueError):
                    NativeGatewayProcess(Path(sys.executable),path,prefix='spark',manus_fd=raw_read)
                with self.assertRaises(ValueError):
                    NativeGatewayProcess(Path(sys.executable),path,prefix='spark',publication_backend='cpp',
                        viewer_backend='cpp',diagnostic_transport='summary',manus_fd=raw_read,recording_fd=raw_read)
                for enabled in (False,True):
                    process=NativeGatewayProcess(Path(sys.executable),path,prefix='spark',
                        publication_backend='cpp',viewer_backend='cpp',diagnostic_transport='summary',manus_fd=raw_read)
                    read,write=os.pipe()
                    with os.fdopen(read,'rb',buffering=0) as reader:
                        os.write(write,b'native_session_gateway_ready publication=cpp viewer=cpp diagnostics=summary'+
                            (b' hands=cpp' if enabled else b'')+b'\n');os.close(write)
                        process._process=SimpleNamespace(stdout=reader,poll=lambda:None)
                        if enabled:process._wait_ready()
                        else:
                            with self.assertRaisesRegex(RuntimeError,'handshake'):process._wait_ready()
                        process._process=None
            finally:os.close(raw_read);os.close(raw_write)

    def test_gateway_rejects_bad_hand_config_before_starting_workers(self):
        binary=ROOT/'build/control-native/tianji_native_session_gateway'
        if not binary.is_file():self.skipTest('build-native-session-gateway required')
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'manifest.json'
            row=manifest();row['hand_runtime']['stdout_fd']=9
            path.write_text(json.dumps(row))
            p=subprocess.run([str(binary),'--manifest',str(path),'--control-fd','9'],capture_output=True,text=True,timeout=5)
            self.assertNotEqual(p.returncode,0)
            self.assertIn('invalid native hand launch configuration',p.stderr)

    def test_config_and_reject_ambiguous_launch(self):
        self.assertTrue((ROOT/'native/control/hand_launch_config.hpp').is_file())
        with tempfile.TemporaryDirectory() as folder:
            binary=Path(folder)/'config'
            subprocess.run(['c++','-std=c++17','-pthread','-O2','-Wall','-Wextra','-Werror',
                str(ROOT/'tests/cpp/native_hand_launch.cpp'),'-o',str(binary)],check=True)
            def run(value):return subprocess.run([str(binary)],input=json.dumps(value),text=True,capture_output=True)
            self.assertEqual(run({}).stdout,'disabled')
            self.assertEqual(run(manifest()).stdout,'10 run manus-instance 2')
            for key,value in [('stdout_fd',9),('stdout_fd',2),('stdout_fd',True),('stdout_fd',2**64-1),
                ('generation',0),('worker_command',['relative']),('capacity',0),('timeout_ms',60001),
                ('age_ns',0),('lower',[[0]*19]*2),('zero',[[2]*20]*2),('zero_tolerance',[[-1]*20]*2)]:
                row=manifest();row['hand_runtime'][key]=value
                self.assertNotEqual(run(row).returncode,0,(key,value))
            for field in ('publication_backend','viewer_backend','diagnostic_transport'):
                row=manifest();row[field]='python';self.assertNotEqual(run(row).returncode,0)
            row=manifest();row['recording_fd']=10;self.assertNotEqual(run(row).returncode,0)
            row=manifest();row['hand_authorities']['left']['router']='other';self.assertNotEqual(run(row).returncode,0)
            row=manifest();row['hand_runtime'].update(right_glove='same',left_glove='same');self.assertNotEqual(run(row).returncode,0)
