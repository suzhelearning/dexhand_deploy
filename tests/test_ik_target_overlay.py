from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np

from tianji_teleop.executors.mujoco.node import MujocoExecutor, _parse_args
from tianji_teleop.protocol.messages import ArmTargetCommand, ProtocolEnvelope
from tianji_teleop.protocol import topics
from test_h5_interaction_overlay import _FakeModel, _FakeData, _FakeSession

URDF = Path(__file__).resolve().parents[1] / 'src/tianji_teleop/assets/tianji_wuji2/tianji_wuji2.urdf'


class IkTargetOverlayTest(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000

    def executor(self, enabled=True):
        self.session = _FakeSession()
        value = MujocoExecutor(model=_FakeModel(), data=_FakeData(), session=self.session,
            publisher_instance_id='sim', router_zid='router', coordinator_instance_id='coord',
            source_instance_id='source', hand_sides=(), ik_target_overlay=enabled,
            overlay_urdf=URDF, clock=lambda: self.now)
        self.addCleanup(value.close)
        return value

    def target(self, side='left', sequence=1):
        return ArmTargetCommand(ProtocolEnvelope(1, 'source', 'router', sequence, self.now),
            None, 'hand_tracking_target', side, 'Base_L' if side == 'left' else 'Base_R',
            [0.3, 0.2, 0.1], [0, 0, 0, 1], [1, 0, 0]).to_dict()

    def viewer(self, count=32):
        return SimpleNamespace(user_scn=mujoco.MjvScene(mujoco.MjModel.from_xml_string('<mujoco/>'), count), lock=nullcontext)

    def test_cli_and_opt_in_subscriptions(self):
        self.assertTrue(_parse_args(['--ik-target-overlay']).ik_target_overlay)
        self.executor(False)
        self.assertNotIn(topics.arm_target('left'), dict(self.session.subscribers))
        self.executor()
        self.assertIn(topics.arm_target('left'), dict(self.session.subscribers))
        self.assertIn(topics.arm_target('right'), dict(self.session.subscribers))

    def test_base_transform_matches_real_mujoco_forward_kinematics(self):
        import xml.etree.ElementTree as ET
        from tianji_teleop.mujoco_urdf import portable_mujoco_urdf
        from tianji_teleop.executors.mujoco.ik_target_overlay import fixed_base_transforms
        xml, assets = portable_mujoco_urdf(URDF)
        root = ET.fromstring(xml)
        root.find('./mujoco/compiler').set('fusestatic', 'false')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'), assets)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        for side, name in (('left', 'Base_L'), ('right', 'Base_R')):
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            self.assertGreater(body, 0)
            transform = fixed_base_transforms(URDF)[side]
            np.testing.assert_allclose(transform[:3, 3], data.xpos[body], atol=1e-9)
            np.testing.assert_allclose(transform[:3, :3], data.xmat[body].reshape(3, 3), atol=1e-9)

    def test_status_reports_last_target_sequence(self):
        executor = self.executor()
        executor.on_ik_target(self.target(sequence=5))
        status = executor._make_status(ready=True, healthy=True, phase='ready')
        self.assertEqual(status.diagnostics['ik_target_overlay']['left']['sequence'], 5)

    def test_passive_bilateral_world_targets_with_rotation(self):
        from tianji_teleop.executors.mujoco.ik_target_overlay import fixed_base_transforms
        executor = self.executor()
        before = executor.data.qpos.copy()
        publications = list(self.session.published)
        bases = fixed_base_transforms(URDF)
        for side in ('left', 'right'):
            self.assertTrue(executor.on_ik_target(self.target(side)))
        viewer = self.viewer()
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        self.assertEqual(viewer.user_scn.ngeom, 8)
        for offset, side in ((0, 'left'), (4, 'right')):
            expected = bases[side][:3, :3] @ [0.3, 0.2, 0.1] + bases[side][:3, 3]
            np.testing.assert_allclose(viewer.user_scn.geoms[offset].pos, expected)
            # Red local X connector center: world position + rotated half-axis.
            np.testing.assert_allclose(viewer.user_scn.geoms[offset+1].pos,
                expected + bases[side][:3, 0] * 0.06, atol=1e-6)
        np.testing.assert_array_equal(executor.data.qpos, before)
        self.assertEqual(publications, self.session.published)

    def test_stale_target_preserved_and_labelled_not_live(self):
        executor = self.executor()
        executor.on_ik_target(self.target())
        self.now += 600_000_000
        viewer = self.viewer()
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        self.assertEqual(viewer.user_scn.ngeom, 4)
        self.assertIn('stale', viewer.user_scn.geoms[0].label)
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        self.assertEqual(viewer.user_scn.ngeom, 4)

    def test_reject_wrong_authority_frame_and_rollback(self):
        executor = self.executor()
        target = self.target()
        self.assertFalse(executor.on_ik_target(dict(target, router_zid='other')))
        self.assertFalse(executor.on_ik_target(dict(target, publisher_instance_id='other')))
        self.assertFalse(executor.on_ik_target(dict(target, frame_id='world')))
        self.assertTrue(executor.on_ik_target(target))
        self.assertFalse(executor.on_ik_target(target))
        viewer = self.viewer(2)
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        self.assertEqual(viewer.user_scn.ngeom, 2)
