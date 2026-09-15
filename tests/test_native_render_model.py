import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import mujoco
import numpy as np

from tianji_teleop.executors.mujoco.mapped_palm_overlay import MappedPalmOverlay
from tianji_teleop.protocol.messages import HAND_JOINT_NAMES

ROOT = Path(__file__).resolve().parents[1]


class NativeRenderModelTest(unittest.TestCase):
    def test_independent_fk_and_target_timeout_match_python(self):
        self._check_model('mapped_palm')

    def test_spark_model_render_target_geometry(self):
        self._check_model('spark')

    def test_hand_feedback_both_models(self):
        for name in ('spark', 'mapped_palm'):
            self._check_model(name, hands=True)

    def _check_model(self, name, hands=False):
        self.assertTrue((ROOT / 'native/control/render_model.hpp').is_file(), 'native render model missing')
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / 'render'
            prefix = Path(sys.prefix)
            flags = ['-O1','-g','-fsanitize=undefined','-fno-omit-frame-pointer'] if os.environ.get('NATIVE_RENDER_UBSAN') == '1' else ['-O2']
            subprocess.run(['c++','-std=c++17',*flags,'-pthread','-Wall','-Wextra','-Werror',
                '-I'+str(prefix/'include'),str(ROOT/'tests/cpp/render_model_fixture.cpp'),
                '-L'+str(prefix/'lib'),'-Wl,-rpath,'+str(prefix/'lib'),'-lmujoco','-o',str(binary)],check=True)
            path = ROOT / f'src/tianji_teleop/assets/{name}/marvin_m6_wuji2.xml'
            model = mujoco.MjModel.from_xml_path(str(path))
            data = mujoco.MjData(model)
            rows, expected = [], []
            rng = np.random.default_rng(1426)
            native = dict(timestamp_ns=1000,
                # Accepted by the overlay's |norm-1| <= 1e-3 contract.
                left=dict(target_position=[.2,.3,1.1],target_quaternion_xyzw=[0.,0.,.60045,.8006]),
                right=dict(target_position=[.3,-.4,.9],target_quaternion_xyzw=[.6,0.,0.,.8]))
            for active, stamp in ((True,1001),(True,200001000),(True,200001001),(True,999),(False,1001)):
                q = rng.uniform(-.3,.3,(2,7))
                hand_q = rng.uniform(-.2,.2,(2,20)) if hands and stamp != 999 else None
                if hand_q is not None:
                    for s, side in enumerate(('left', 'right')):
                        for j, joint in enumerate(HAND_JOINT_NAMES[side]):
                            ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
                            if ident < 0:
                                joint = joint[:joint.index('_', 2)] + '_finger' + joint[joint.index('_', 2):]
                                ident = model.joint(joint).id
                            data.qpos[model.jnt_qposadr[ident]] = hand_q[s,j]
                for s, suffix in enumerate(('L','R')):
                    for j in range(7):
                        data.qpos[model.jnt_qposadr[model.joint(f'Joint{j+1}_{suffix}').id]] = q[s,j]
                overlay = MappedPalmOverlay('source')
                # This tests the renderer boundary; packet/epoch acceptance is
                # separately covered by the existing overlay tests.
                overlay._active = active
                overlay._native = native
                overlay.update_render_targets(model,data,mujoco,stamp)
                ids = [model.body('target_'+suffix).mocapid[0] for suffix in ('L','R')]
                expected.append(dict(qpos=data.qpos.copy(),positions=data.mocap_pos[ids].copy(),
                                     quaternions=data.mocap_quat[ids].copy()))
                rows.append(dict(q=q.tolist(),native=native,active=active,now_ns=stamp))
                if hand_q is not None: rows[-1]['hands'] = hand_q.tolist()
            result=subprocess.run([str(binary),str(path)],input='\n'.join(map(json.dumps,rows)),
                                  text=True,capture_output=True,timeout=15)
            self.assertEqual(result.returncode,0,result.stderr)
            actual=list(map(json.loads,result.stdout.splitlines()))
            self.assertEqual(len(actual),len(expected))
            for a,b in zip(actual,expected):
                for key in b:
                    np.testing.assert_allclose(a[key],b[key],atol=1e-14,rtol=0,err_msg=key)
