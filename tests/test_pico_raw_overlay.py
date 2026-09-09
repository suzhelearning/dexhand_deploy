"""Passive raw PICO visualization, using a real MuJoCo user scene."""
from contextlib import nullcontext
from types import SimpleNamespace
import unittest
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from tianji_teleop.executors.mujoco.node import MujocoExecutor, _parse_args
from tianji_teleop.hand_tracking.pico import parse_pico_packet
from tianji_teleop.protocol import topics
from test_h5_interaction_overlay import _FakeModel, _FakeData, _FakeSession, _frame0_message
from test_pico_mujoco_pipeline import packet


class PicoRawOverlayTest(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000
        self.session = _FakeSession()
        self.data = _FakeData()

    def executor(self, enabled=True):
        value = MujocoExecutor(
            session=self.session, model=_FakeModel(), data=self.data,
            publisher_instance_id='sim', router_zid='router-zid',
            coordinator_instance_id='coord', source_instance_id='h5-source-instance',
            hand_sides=(), pico_overlay=enabled, clock=lambda: self.now,
        )
        self.addCleanup(value.close)
        return value

    def payload(self, sequence=1, generation=1):
        value = parse_pico_packet(
            packet(0, 0), receiver_instance_id='pico',
            connection_generation=generation, receiver_frame_sequence=sequence,
            received_timestamp_ns=self.now,
        ).to_dict()
        return dict(value, router_zid='router-zid')

    def viewer(self, capacity=200):
        model = mujoco.MjModel.from_xml_string('<mujoco/>')
        return SimpleNamespace(user_scn=mujoco.MjvScene(model, capacity), lock=nullcontext)

    def test_cli_opt_in(self):
        self.assertTrue(_parse_args(['--pico-overlay']).pico_overlay)
        self.assertFalse(_parse_args([]).pico_overlay)

    def test_launcher_rejects_overlay_outside_pico_simulation(self):
        result = subprocess.run(['bash', 'scripts/run_session.sh', '--profile',
                                 'hand_tracking_sim_manus', '--pico-overlay'],
                                cwd=Path(__file__).resolve().parents[1],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2)
        self.assertIn('--pico-overlay', result.stderr)

    def test_diagnostics_are_independent_of_executor_health(self):
        executor = self.executor()
        self.assertEqual(executor._make_status(ready=True, healthy=True, phase='ready').diagnostics[
            'pico_overlay']['state'], 'waiting')
        executor.on_pico_raw(self.payload())
        status = executor._make_status(ready=True, healthy=True, phase='ready')
        self.assertEqual(status.diagnostics['pico_overlay']['frames_received'], 1)
        self.assertTrue(status.healthy)

    def test_invalid_joint_never_draws_at_its_invalid_position(self):
        executor = self.executor()
        payload = self.payload()
        joint = payload['hands']['left']['joints'][8]
        joint['valid'] = False
        joint['pose'][:3] = [123, 456, 789]
        self.assertTrue(executor.on_pico_raw(payload))
        viewer = self.viewer()
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        for geom in viewer.user_scn.geoms[:viewer.user_scn.ngeom]:
            self.assertLess(np.linalg.norm(geom.pos), 10)

    def test_rotation_is_xyzw_and_not_recentered_on_head(self):
        executor = self.executor()
        payload = self.payload()
        payload['head_pose'] = [1, 2, 3, 0, 0, np.sqrt(0.5), np.sqrt(0.5)]
        self.assertTrue(executor.on_pico_raw(payload))
        viewer = self.viewer()
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        geoms = viewer.user_scn.geoms
        head_index = next(i for i in range(viewer.user_scn.ngeom) if geoms[i].label == 'PICO head')
        np.testing.assert_allclose(geoms[head_index].pos, [1, 3.2, 3.8])
        np.testing.assert_allclose(geoms[head_index + 1].pos, [1, 3.24, 3.8], atol=1e-6)

    def test_disabled_does_not_subscribe(self):
        self.executor(False)
        self.assertNotIn(topics.RAW_PICO_HAND_TRACKING, dict(self.session.subscribers))

    def test_enabled_without_hands_is_passive_and_preserves_raw_head_pose(self):
        executor = self.executor()
        before = self.data.qpos.copy()
        published = list(self.session.published)
        self.assertIn(topics.RAW_PICO_HAND_TRACKING, dict(self.session.subscribers))
        payload = self.payload()
        self.assertTrue(executor.on_pico_raw(payload))
        viewer = self.viewer()
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        scene = viewer.user_scn
        self.assertGreater(scene.ngeom, 90)
        head = next(g for g in scene.geoms[:scene.ngeom] if g.label == 'PICO head')
        np.testing.assert_allclose(head.pos, np.array(payload['head_pose'][:3]) + [0, 1.2, 0.8])
        np.testing.assert_array_equal(self.data.qpos, before)
        self.assertEqual(self.session.published, published)

    def test_stale_geometry_clears_and_scene_does_not_accumulate(self):
        executor = self.executor()
        executor.on_pico_raw(self.payload())
        viewer = self.viewer()
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        count = viewer.user_scn.ngeom
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        self.assertEqual(viewer.user_scn.ngeom, count)
        self.now += 600_000_000
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        self.assertEqual(viewer.user_scn.ngeom, 1)
        self.assertIn('stale', viewer.user_scn.geoms[0].label)

    def test_wrong_router_stale_and_reordered_frames_rejected(self):
        executor = self.executor()
        payload = self.payload(3)
        self.assertFalse(executor.on_pico_raw(dict(payload, router_zid='other')))
        self.assertTrue(executor.on_pico_raw(payload))
        self.assertFalse(executor.on_pico_raw(self.payload(2)))
        self.assertTrue(executor.on_pico_raw(self.payload(0, 2)))
        self.assertFalse(executor.on_pico_raw(self.payload(4, 1)))
        self.now += 600_000_000
        self.assertFalse(executor.on_pico_raw(payload))

    def test_invalid_hands_hide_skeleton_independently_of_head(self):
        executor = self.executor()
        payload = self.payload()
        for hand in payload['hands'].values():
            hand['valid'] = False
            hand['wrist_valid'] = False
        self.assertTrue(executor.on_pico_raw(payload))
        viewer = self.viewer()
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        self.assertEqual(viewer.user_scn.ngeom, 8)  # origin+axes, head+axes

    def test_small_scene_and_frame0_coexist(self):
        executor = self.executor()
        self.assertTrue(executor.on_frame0_hand_skeleton(_frame0_message()))
        executor.on_pico_raw(self.payload())
        viewer = self.viewer()
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        labels = [g.label for g in viewer.user_scn.geoms[:viewer.user_scn.ngeom]]
        self.assertIn('PICO head', labels)
        self.assertGreater(viewer.user_scn.ngeom, 120)
        viewer = self.viewer(5)
        executor.update_viewer_overlays(viewer, mujoco_module=mujoco)
        self.assertEqual(viewer.user_scn.ngeom, 5)
