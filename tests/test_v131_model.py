from pathlib import Path
import unittest
import numpy as np
import mujoco
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src/tianji_teleop/assets/v131/marvin_m6_qp_pico_fast_kinematics.xml'
_original_model = os.environ.get('V131_REFERENCE_MODEL')
ORIGINAL = Path(_original_model).expanduser() if _original_model else None


class OriginalModelTest(unittest.TestCase):
    @unittest.skipUnless((ROOT / 'build/ik-sim/v131_model_trace').exists(), 'native build unavailable')
    def test_rejects_mismatched_pinocchio_gradient_model(self):
        source = ROOT / 'src/tianji_teleop/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf'
        tree = ET.parse(source)
        for joint in tree.getroot().findall('joint'):
            if joint.find('child').get('link') == 'TCP_Link_L':
                joint.find('origin').set('xyz', '0.2 -0.095 0')
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'mismatched.urdf'
            tree.write(path)
            result = subprocess.run([str(ROOT / 'build/ik-sim/v131_model_trace'), str(path), str(MODEL)],
                cwd=temporary, capture_output=True, text=True,
                env={**os.environ, 'TIANJI_V131_MODEL': str(MODEL)})
        self.assertNotEqual(result.returncode, 0, 'mismatched kinematics were silently accepted')
        self.assertIn('kinematics do not match MuJoCo', result.stderr)

    def test_original_model_is_bundled_without_external_mesh_dependency(self):
        self.assertTrue(MODEL.is_file(), 'missing original v131 kinematic model')
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.assertEqual((model.nq, model.nv), (14, 14))
        self.assertEqual(model.nmesh, 0)
        np.testing.assert_array_equal(model.jnt_user[:, 0], np.full(14, 4.0))
        np.testing.assert_array_equal(model.jnt_range[0], [-1.5708, 3.1067])
        np.testing.assert_array_equal(model.jnt_range[7], [-3.1067, 1.5708])

    @unittest.skipUnless(ORIGINAL is not None and ORIGINAL.is_file(), 'optional reference checkout unavailable')
    def test_world_fk_and_jacobian_match_original_model(self):
        self.assertTrue(MODEL.is_file(), 'missing original v131 kinematic model')
        models = [mujoco.MjModel.from_xml_path(str(path)) for path in (ORIGINAL, MODEL)]
        data = [mujoco.MjData(m) for m in models]
        rng = np.random.default_rng(131)
        for _ in range(30):
            q = rng.uniform(models[0].jnt_range[:, 0] + .05, models[0].jnt_range[:, 1] - .05)
            samples = []
            for m, d in zip(models, data):
                d.qpos[:] = q
                mujoco.mj_forward(m, d)
                sample = []
                for site in ('tcp_L', 'tcp_R'):
                    idx = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, site)
                    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
                    mujoco.mj_jacSite(m, d, jp, jr, idx)
                    sample.extend([d.site_xpos[idx].copy(), d.site_xmat[idx].copy(), jp, jr])
                samples.append(sample)
            for a, b in zip(*samples):
                np.testing.assert_allclose(a, b, atol=1e-12, rtol=0)


if __name__ == '__main__':
    unittest.main()
