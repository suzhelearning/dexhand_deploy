from pathlib import Path
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

from tests.test_reference_tjvr_receiver import packet
from tianji_teleop.hand_tracking.native_binary_results import decode_result, frame_size
from tianji_teleop.hand_tracking.spark_replay import ReplayTick, encode_tick
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver

ROOT=Path(__file__).resolve().parents[1]

@unittest.skipUnless(shutil.which('c++'),'C++ compiler required')
class NativeWorkerStepTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        check=subprocess.run(['c++','-x','c++','-E','-'],input='#include <nlohmann/json.hpp>\n',text=True,capture_output=True)
        if check.returncode: raise unittest.SkipTest('nlohmann-json headers required')
        cls.tmp=tempfile.TemporaryDirectory(prefix='native-worker-step-')
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.driver=Path(cls.tmp.name)/'driver'
        flags=['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie'] if os.environ.get('NATIVE_STEP_SANITIZERS')=='1' else ['-O2']
        cls.flags=flags
        subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror',*flags,
            str(ROOT/'tests/cpp/native_worker_step_driver.cpp'),'-o',str(cls.driver)],check=True)

    def test_actual_workers_match_python_client_through_reset(self):
        from tianji_teleop.producers.spark.backend_assets import bilateral_assets
        from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND,MAPPED_PALM_BACKEND
        for prefix,backend in [('spark',SPARK_BACKEND),('mapped_palm',MAPPED_PALM_BACKEND)]:
            with self.subTest(backend=backend):
                assets=bilateral_assets(ROOT,backend)
                if not assets['worker'].is_file(): self.skipTest('build native workers')
                receiver=ReferenceTjvrReceiver('test',.15,.6,target_source='mapped_corrected_palm' if prefix=='mapped_palm' else 'packet')
                lines=['invalid\n']; expected=[]
                with assets['client'](**{k:assets[k] for k in ('worker','config','model','urdf')},
                    deterministic_test=True,startup_handshake=True,result_format='binary') as client:
                    for i in range(1,81):
                        now=1_000_000_000+i*5_000_000
                        sample=None
                        if i in (1,2,24):
                            receiver.ingest(packet(i,epoch=10 if i==24 else 9),now)
                            sample=receiver.try_read_latest()
                        tick=ReplayTick(i,now,sample)
                        lines.append(encode_tick(tick)); expected.append(client.step(tick))
                    q=expected[-1]['left']['q']+expected[-1]['right']['q']
                    client.reset_at_rest(q,execution_epoch=2); lines.extend(['reset\n','invalid\n'])
                    tick=ReplayTick(1,2_000_000_000,None)
                    lines.append(encode_tick(tick)); expected.append(client.step(tick))
                self.assertTrue(any(r['control_executed'] for r in expected))
                self.assertTrue(any(r['input_live'] for r in expected))
                self.assertTrue(any(not r['input_live'] for r in expected))
                self.assertNotEqual(expected[0]['left']['q'],expected[30]['left']['q'])
                result=subprocess.run([str(self.driver),prefix,backend,'20000',
                    *[str(assets[k]) for k in ('worker','config','model','urdf')],
                    '--startup-handshake','--deterministic-test','--binary-results'],
                    input=''.join(lines),text=True,capture_output=True,timeout=60)
                self.assertEqual(result.returncode,0,result.stderr)
                rows=[json.loads(line) for line in result.stdout.splitlines()]
                self.assertEqual(len(rows),len(expected))
                for row,want in zip(rows,expected):
                    self.assertEqual(decode_result(bytes.fromhex(row['wire']),prefix),want)
                    for key,field in [('tick','tick_id'),('now','timestamp_ns'),('epoch','applied_epoch'),
                                      ('sequence','applied_sequence'),('live','input_live'),('control','control_executed'),('deterministic','deterministic_test')]:
                        self.assertEqual(row[key],want[field])
                    for side in ('left','right'):
                        for key,value in row[side].items(): self.assertEqual(value,want[side][key])

    def test_bounded_transport_and_invalid_results_poison(self):
        frame=bytearray(frame_size('mapped_palm'))
        struct.pack_into('<4sBBH',frame,0,b'TJBR',1,2,len(frame)-8)
        frame[8]=1
        struct.pack_into('<QQ',frame,9,1,1_000_000_000)
        for mode in ('ok','split','int64max','truncated','extra','backend','size','bool','nan','tick','time','deterministic','timeout'):
            with self.subTest(mode=mode):
                data=bytearray(frame)
                now=2**63-1 if mode=='int64max' else 1_000_000_000
                struct.pack_into('<Q',data,17,now)
                if mode=='truncated': data=data[:10]
                if mode=='extra': data+=b'bad'
                if mode=='backend': data[5]=1
                if mode=='size': data[6]=0
                if mode=='bool': data[8]=2
                if mode=='deterministic': data[8]=0
                if mode=='nan': struct.pack_into('<d',data,65,float('nan'))
                if mode=='tick': struct.pack_into('<Q',data,9,99)
                if mode=='time': struct.pack_into('<Q',data,17,2)
                code="import sys,os,time,json\nprint(json.dumps(dict(schema_version=1,kind='mapped_palm_worker_ready',algorithm='test',native_ticks=0)),flush=True)\nsys.stdin.readline()\n"
                if mode=='timeout': code+='time.sleep(10)\n'
                elif mode=='split': code+='for b in '+repr(bytes(data))+':\n os.write(1,bytes([b])); time.sleep(.00001)\n'
                else: code+='os.write(1,'+repr(bytes(data))+')\n'
                code+='sys.stdin.readline()\n'
                result=subprocess.run([str(self.driver),'mapped_palm','test','300',sys.executable,'-u','-c',code],
                    input=f'TJSC1 1 {now} 0 0 0 -\n',text=True,capture_output=True,timeout=5)
                self.assertEqual(result.returncode,0 if mode in ('ok','split','int64max') else 1,result.stderr)
                if mode not in ('ok','split','int64max'): self.assertIn('closed; no implicit restart',result.stderr)

    def test_cancel_step_wait(self):
        binary=Path(self.tmp.name)/'cancel'
        subprocess.run(['c++','-std=c++17','-pthread','-Wall','-Wextra','-Werror',*self.flags,
            str(ROOT/'tests/cpp/test_worker_reset_cancel.cpp'),'-o',str(binary)],check=True)
        code="import json,time,sys; print(json.dumps(dict(schema_version=1,kind='mapped_palm_worker_ready',algorithm='test',native_ticks=0)),flush=True); sys.stdin.readline(); time.sleep(10)"
        result=subprocess.run([str(binary),'--step',sys.executable,'-u','-c',code],text=True,capture_output=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('cancelled_without_epoch_commit',result.stdout)
