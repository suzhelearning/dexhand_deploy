import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
import sys
import numpy as np
from tianji_teleop.hand_tracking.mapped_palm_height import MappedPalmHeight

ROOT = Path(__file__).resolve().parents[1]

@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class NativeHeightTest(unittest.TestCase):
    def test_height_transport_rejects_invalid_ack_and_closes(self):
        code='''
import json,sys,time
print(json.dumps(dict(schema_version=1,kind='mapped_palm_worker_ready',algorithm='test',native_ticks=0)),flush=True)
for line in sys.stdin:
    mode=sys.argv[1]
    if mode=='timeout': time.sleep(10)
    if mode=='eof': sys.exit(0)
    if mode=='oversize': print('x'*5000,flush=True); continue
    offsets=list(map(float,line.split()[1:]))
    if mode=='wrong': offsets[1]+=.01
    if mode=='boolean': offsets[0]=False
    ack=json.dumps(dict(kind='mapped_palm_height_ack',target_height_offsets_m=offsets))
    if mode=='duplicate': ack=ack.replace('"kind":','"kind":"mapped_palm_height_ack","kind":')
    print(ack,flush=True)
'''
        with tempfile.TemporaryDirectory(prefix='native-height-ack-') as folder:
            binary=Path(folder)/'ack'
            subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
                str(ROOT/'tests/cpp/native_height_ack_driver.cpp'),'-o',str(binary)],check=True)
            for mode in ('ok','wrong','boolean','duplicate','oversize','timeout','eof'):
                with self.subTest(mode=mode):
                    result=subprocess.run([str(binary),sys.executable,'-u','-c',code,mode],text=True,capture_output=True,timeout=5)
                    self.assertEqual(result.returncode,0 if mode=='ok' else 1,result.stderr)
                    if mode!='ok': self.assertIn('height_transport_closed',result.stderr)

    @unittest.skipUnless((Path(sys.prefix)/'include/mujoco/mujoco.h').is_file(), 'MuJoCo headers required')
    def test_actual_worker_height_ack_and_readonly_model_reference(self):
        from tianji_teleop.producers.spark.backend_assets import bilateral_assets
        from tianji_teleop.hand_tracking.input_modes import MAPPED_PALM_BACKEND
        from tianji_teleop.hand_tracking.mapped_palm_height import horizontal_tcp_heights
        from tests.test_mapped_palm_height import sample
        import mujoco
        assets=bilateral_assets(ROOT,MAPPED_PALM_BACKEND)
        if not assets['worker'].is_file(): self.skipTest('build mapped worker')
        with tempfile.TemporaryDirectory(prefix='native-height-worker-') as folder:
            binary=Path(folder)/'worker'
            subprocess.run(['c++','-std=c++17','-O2','-pthread','-Wall','-Wextra','-Werror',
                '-I'+str(Path(sys.prefix)/'include'),str(ROOT/'tests/cpp/native_height_worker_driver.cpp'),
                '-L'+str(Path(sys.prefix)/'lib'),'-Wl,-rpath,'+str(Path(sys.prefix)/'lib'),'-lmujoco','-o',str(binary)],check=True)
            result=subprocess.run([str(binary),*[str(assets[k]) for k in ('worker','config','model','urdf')],
                '--startup-handshake','--binary-results','--deterministic-test'],
                input=sample(1,1000000000).observation.frame.raw_packet.hex()+'\n',text=True,capture_output=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stderr)
            got=json.loads(result.stdout)
            reference=horizontal_tcp_heights(mujoco.MjModel.from_xml_path(str(assets['model'])))
            np.testing.assert_allclose(got['reference'],[reference[s] for s in ('left','right')],atol=1e-14,rtol=0)
            self.assertEqual(got['offsets'],[-.1,-.12])

    def test_reference_sampling_and_failed_retry_parity(self):
        self.assertTrue((ROOT/'native/control/height_calibration.hpp').is_file(), 'native sampler missing')
        with tempfile.TemporaryDirectory(prefix='native-height-') as folder:
            binary=Path(folder)/'height'
            subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
                            str(ROOT/'tests/cpp/native_height_driver.cpp'),'-o',str(binary)],check=True)
            for mode in ('steady','movement','duplicates','stale','epoch','invalid','large','gap','retry','few','same_stamp','pre_begin'):
                with self.subTest(mode=mode):
                    rows=[dict(op='begin',now=1_000_000_000)]
                    for i in range(101):
                        now=1_000_000_000+i*20_000_000
                        rows.append(dict(op='update',now=now,seq=1 if mode=='duplicates' else i+1,
                            epoch=10 if mode=='epoch' and i>20 else 9,
                            valid=not(mode=='invalid' and i>20),
                            stamp=now-300_000_000 if mode=='stale' else now,
                            z=2.5 if mode=='large' else 1.2+(i*.002 if mode=='movement' else 0)))
                    if mode=='gap': rows=rows[:3]+[dict(op='update',now=2_000_000_000)]
                    if mode=='few': rows=[rows[0]]+[r for i,r in enumerate(rows[1:]) if i%5==0]
                    if mode=='same_stamp':
                        for row in rows[1:]: row['stamp']=1_000_000_000
                    if mode=='pre_begin':
                        rows.insert(1,dict(op='update',now=1_000_000_000,seq=1,epoch=9,valid=True,stamp=990_000_000,z=1.2))
                    if mode=='retry': rows += [dict(op='begin',now=4_000_000_000),dict(op='update',now=4_300_000_000)]
                    result=subprocess.run([str(binary)],input=''.join(json.dumps(r)+'\n' for r in rows),
                                          text=True,capture_output=True,check=True)
                    actual=[json.loads(line) for line in result.stdout.splitlines()]
                    reference=MappedPalmHeight({'left':1.1,'right':1.1})
                    self.assertEqual(len(actual),len(rows))
                    for row,got in zip(rows,actual):
                        if row['op']=='begin': reference.begin(row['now'])
                        else:
                            sample=None
                            if 'seq' in row:
                                points=np.zeros((8,3)); points[3]=[.3,.35,row['z']]; points[7]=[.3,-.35,row['z']+.02]
                                frame=SimpleNamespace(receiver_instance_id='fixed',tracking_epoch=row['epoch'],sequence=row['seq'],
                                    upper_limb_skeleton_valid=row['valid'],upper_limb_rotations_valid=row['valid'],
                                    upper_limb_points=points,received_timestamp_ns=row['stamp'])
                                sample=SimpleNamespace(observation=SimpleNamespace(frame=frame))
                            reference.update(sample,row['now'])
                        self.assertEqual(got['state'],reference.state)
                        self.assertEqual(got['count'],[len(reference.sampler.samples[s]) for s in ('left','right')])
                        if reference.offsets is None: self.assertIsNone(got['offsets'])
                        else: np.testing.assert_allclose(got['offsets'],reference.offsets,atol=1e-12,rtol=0)
