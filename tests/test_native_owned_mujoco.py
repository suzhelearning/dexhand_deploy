import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]

@unittest.skipUnless(shutil.which('c++') and (Path(sys.prefix)/'include/mujoco/mujoco.h').is_file(),
                     'C++ and active MuJoCo development headers required')
class NativeOwnedMujocoTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import mujoco
        cls.mujoco=mujoco
        cls.tmp=tempfile.TemporaryDirectory(prefix='native-owned-mujoco-')
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.driver=Path(cls.tmp.name)/'driver'
        sanitizers=os.environ.get('NATIVE_OWNED_MUJOCO_SANITIZERS','')
        if sanitizers not in ('','1','undefined'): raise ValueError('sanitizers must be 1 or undefined')
        flags=['-O1','-g','-fsanitize='+('address,undefined' if sanitizers=='1' else 'undefined'),
               '-fno-omit-frame-pointer','-fno-pie','-no-pie'] if sanitizers else ['-O2']
        cls.flags=flags
        subprocess.run(['c++','-std=c++17','-pthread','-Wall','-Wextra','-Werror',*flags,
            '-I'+str(Path(sys.prefix)/'include'),str(ROOT/'tests/cpp/native_owned_mujoco_driver.cpp'),
            '-L'+str(Path(sys.prefix)/'lib'),'-Wl,-rpath,'+str(Path(sys.prefix)/'lib'),'-lmujoco','-o',str(cls.driver)],check=True)

    def run_driver(self,model,groups,bodies,commands,aliases=None):
        return subprocess.run([str(self.driver),str(model)],input='\n'.join(json.dumps(x) for x in
            [dict(groups=groups,bodies=bodies,aliases=aliases or {}),*commands])+'\n',text=True,capture_output=True,timeout=30)

    def test_aliases_prefer_canonical_and_reject_duplicate_and_invalid_names(self):
        path=ROOT/'src/tianji_teleop/assets/mapped_palm/marvin_m6_wuji2.xml'
        result=self.run_driver(path,[['l_pinky_mcp_flex']],[],[[[.3]]],
                               {'l_pinky_mcp_flex':'missing'})
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(json.loads(result.stdout.splitlines()[-1])['groups'],[[.3]])
        for groups,aliases in [([['renamed','Joint1_L']], {'renamed':'Joint1_L'}),
                               ([['']], {'':'Joint1_L'}),
                               ([['renamed']], {'renamed':'Joint1_L\x00bad'}),
                               ([['renamed']], {'renamed':'missing'})]:
            with self.subTest(groups=groups,aliases=aliases):
                result=self.run_driver(path,groups,[],[],aliases)
                self.assertEqual(result.returncode,1,result.stderr)

    def test_fixed_feedback_has_no_heap_allocation_and_matches_snapshot(self):
        binary=Path(self.tmp.name)/'feedback'
        subprocess.run(['c++','-std=c++17','-pthread','-Wall','-Wextra','-Werror',*self.flags,
            '-I'+str(Path(sys.prefix)/'include'),str(ROOT/'tests/cpp/native_feedback_allocations.cpp'),
            '-L'+str(Path(sys.prefix)/'lib'),'-Wl,-rpath,'+str(Path(sys.prefix)/'lib'),'-lmujoco','-o',str(binary)],check=True)
        for name in ('spark','mapped_palm'):
            with self.subTest(model=name):
                result=subprocess.run([str(binary),str(ROOT/f'src/tianji_teleop/assets/{name}/marvin_m6_wuji2.xml')],
                    text=True,capture_output=True,timeout=10)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertIn('fixed_allocations=0',result.stdout)
                self.assertIn('mixed_allocations=0',result.stdout)

    def test_real_models_match_python_fk_and_atomic_failure(self):
        import numpy as np
        for name in ('spark','mapped_palm'):
            with self.subTest(model=name):
                path=ROOT/f'src/tianji_teleop/assets/{name}/marvin_m6_wuji2.xml'
                model=self.mujoco.MjModel.from_xml_path(str(path)); data=self.mujoco.MjData(model)
                groups=[[f'Joint{i}_{side}' for i in range(1,8)] for side in ('L','R')]
                bodies=['hand_tcp_mount_L','hand_tcp_mount_R']
                addresses=[[int(model.jnt_qposadr[model.joint(j).id]) for j in group] for group in groups]
                rng=np.random.default_rng(1412)
                commands=[rng.uniform(-.4,.4,(2,7)).tolist() for _ in range(30)]
                commands += [[None,[.1]*7], 'nan', [[0.]*7,[0.]*6], [[0.]*7], 'wrong_thread']
                result=self.run_driver(path,groups,bodies,commands)
                self.assertEqual(result.returncode,0,result.stderr)
                rows=[json.loads(line) for line in result.stdout.splitlines()]
                self.assertEqual(len(rows),len(commands)+1)
                self.mujoco.mj_forward(model,data)
                for index,row in enumerate(rows):
                    if index:
                        command=commands[index-1]
                        valid=isinstance(command,list) and len(command)==2 and all(g is None or len(g)==7 for g in command)
                        if valid:
                            for addr,g in zip(addresses,command):
                                if g is not None: data.qpos[addr]=g
                            self.mujoco.mj_forward(model,data)
                        self.assertEqual(row['accepted'],valid or command=='wrong_thread')
                    np.testing.assert_array_equal(row['qpos'],data.qpos)
                    for addr,g in zip(addresses,row['groups']): np.testing.assert_array_equal(g,data.qpos[addr])
                    for body,pose in zip(bodies,row['poses']):
                        np.testing.assert_allclose(pose['position'],data.body(body).xpos,atol=1e-14,rtol=0)
                        np.testing.assert_allclose(pose['quaternion_wxyz'],data.body(body).xquat,atol=1e-14,rtol=0)

    def test_missing_or_duplicate_bindings_rejected(self):
        path=ROOT/'src/tianji_teleop/assets/mapped_palm/marvin_m6_wuji2.xml'
        for groups,bodies in [([['missing']],[]),([['Joint1_L'],['Joint1_L']],[]),
                              ([['Joint1_L']],['missing']),([[]]*5,[])]:
            with self.subTest(groups=groups,bodies=bodies):
                result=self.run_driver(path,groups,bodies,[])
                self.assertEqual(result.returncode,1,result.stderr)

    def test_four_groups_and_missing_hand_update_hold(self):
        path=ROOT/'src/tianji_teleop/assets/mapped_palm/marvin_m6_wuji2.xml'
        groups=[['Joint1_L'],['Joint1_R'],['l_thumb_cmc_flex'],['r_thumb_cmc_flex']]
        result=self.run_driver(path,groups,[],[[[.1],[.2],[.3],[.4]],[[.5],[.6],None,None]])
        self.assertEqual(result.returncode,0,result.stderr)
        rows=[json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(rows[-1]['groups'],[[.5],[.6],[.3],[.4]])

    def test_fixed_mixed_matches_all_qpos_and_fk_of_dynamic_batch(self):
        from tianji_teleop.protocol.messages import HAND_JOINT_NAMES, ARM_JOINT_NAMES
        import numpy as np
        groups=[list(ARM_JOINT_NAMES[s]) for s in ('left','right')]
        for side in ('left','right'):
            groups.append([name if 'thumb_' in name or 'pinky_' in name else name.replace('_mcp_', '_finger_mcp_')
                           .replace('_pip', '_finger_pip').replace('_dip', '_finger_dip')
                           for name in HAND_JOINT_NAMES[side]])
        rng=np.random.default_rng(9127)
        commands=[[rng.uniform(-.4,.4,len(g)).tolist() for g in groups] for _ in range(20)]
        commands[7][2]=None
        commands[11][3]=None
        bodies=['hand_tcp_mount_L','hand_tcp_mount_R']
        for name in ('spark','mapped_palm'):
            with self.subTest(model=name):
                path=ROOT/f'src/tianji_teleop/assets/{name}/marvin_m6_wuji2.xml'
                dynamic=self.run_driver(path,groups,bodies,commands)
                fixed=self.run_driver(path,groups,bodies,[{'mixed':v} for v in commands])
                self.assertEqual(dynamic.returncode,0,dynamic.stderr)
                self.assertEqual(fixed.returncode,0,fixed.stderr)
                self.assertEqual(fixed.stdout,dynamic.stdout)

    def test_worker_coordinator_guard_simulation_and_home(self):
        from tests.test_reference_tjvr_receiver import packet
        from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
        from tianji_teleop.hand_tracking.spark_replay import ReplayTick,encode_tick
        from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND,MAPPED_PALM_BACKEND
        from tianji_teleop.producers.spark.backend_assets import bilateral_assets
        driver=Path(self.tmp.name)/'cycle'
        subprocess.run(['c++','-std=c++17','-pthread','-Wall','-Wextra','-Werror',*self.flags,
            '-I'+str(Path(sys.prefix)/'include'),str(ROOT/'tests/cpp/native_owned_cycle_driver.cpp'),
            '-L'+str(Path(sys.prefix)/'lib'),'-Wl,-rpath,'+str(Path(sys.prefix)/'lib'),'-lmujoco','-o',str(driver)],check=True)
        for prefix,backend in [('spark',SPARK_BACKEND),('mapped_palm',MAPPED_PALM_BACKEND)]:
            with self.subTest(backend=backend):
                assets=bilateral_assets(ROOT,backend)
                if not assets['worker'].is_file(): self.skipTest('build native IK workers')
                model=self.mujoco.MjModel.from_xml_path(str(assets['model']))
                groups=[[f'Joint{i}_{side}' for i in range(1,8)] for side in ('L','R')]
                config=dict(groups=groups,home=[[1.1,-1.52,-1.52,-1.1,0,0,0],[-1.1,-1.52,1.52,-1.1,0,0,0]],
                    lower=[[float(model.jnt_range[model.joint(j).id,0]) for j in g] for g in groups],
                    upper=[[float(model.jnt_range[model.joint(j).id,1]) for j in g] for g in groups])
                receiver=ReferenceTjvrReceiver('offline',.15,.6,target_source='mapped_corrected_palm' if prefix=='mapped_palm' else 'packet')
                lines=[json.dumps(config)+'\n']; expected=[]
                with assets['client'](**{k:assets[k] for k in ('worker','config','model','urdf')},
                    deterministic_test=True,startup_handshake=True,result_format='binary') as client:
                    for i in range(1,41):
                        now=1000000000+i*5000000
                        receiver.ingest(packet(i),now)
                        tick=ReplayTick(i,now,receiver.try_read_latest()); lines.append(encode_tick(tick))
                        row=client.step(tick); expected.append([row[s]['q'] for s in ('left','right')])
                result=subprocess.run([str(driver),prefix,backend,'20000',
                    *[str(assets[k]) for k in ('worker','config','model','urdf')],
                    '--startup-handshake','--binary-results','--deterministic-test'],
                    input=''.join(lines),text=True,capture_output=True,timeout=60)
                self.assertEqual(result.returncode,0,result.stderr)
                rows=[json.loads(line) for line in result.stdout.splitlines()]
                self.assertEqual([r['q'] for r in rows[:-1]],expected)
                self.assertEqual(rows[-1],dict(home=True,state='idle'))
