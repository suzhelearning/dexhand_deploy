import unittest
from pathlib import Path

import mujoco
import numpy as np

from tests import test_spark_overlay as spark_tests
from tianji_teleop.hand_tracking.input_modes import MAPPED_PALM_BACKEND
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReceivedTjvrFrame


class MappedPalmOverlayTest(unittest.TestCase):
    def make(self):
        from tianji_teleop.executors.mujoco import mapped_palm_overlay
        return mapped_palm_overlay.MappedPalmOverlay('receiver')

    def sample(self, seq=1):
        frame = spark_tests.SparkOverlayTest().frame(seq)
        native = spark_tests.SparkOverlayTest().native()
        native.update(algorithm=MAPPED_PALM_BACKEND, tick_id=seq,
                      applied_sequence=frame.frame.sequence,
                      applied_epoch=frame.frame.tracking_epoch, input_live=True)
        attempt = dict(tick_id=seq, timestamp_ns=1000,
                       sample=ReceivedTjvrFrame(frame).to_dict())
        return frame, native, attempt

    def test_only_applied_skeleton_is_shown_without_packet_targets_or_raw_axes(self):
        overlay = self.make()
        frame, native, attempt = self.sample()
        overlay.ingest_raw(frame)
        self.assertEqual(overlay.geometry(1001), ([], []))
        overlay.ingest_cycle(native, attempt, execution_epoch=1, active=True)
        markers, bones = overlay.geometry(1001)
        self.assertEqual(len(bones), 7)
        self.assertFalse(any('packet target' in m['label'] for m in markers))
        palm = next(m for m in markers if m['label'] == 'Applied corrected palm left')
        np.testing.assert_array_equal(palm['position'], frame.frame.upper_limb_points[3])
        self.assertIsNone(palm['rotation'])
        overlay.ingest_raw(self.sample(2)[0])
        self.assertEqual(len(overlay.geometry(1001)[1]), 7)

    def test_rejected_input_idle_timeout_and_epoch_do_not_show_wrong_skeleton(self):
        overlay = self.make()
        frame, native, attempt = self.sample()
        overlay.ingest_cycle(native, attempt, execution_epoch=1, active=True)
        _, newer, wrong = self.sample(2)
        newer['applied_sequence'] = native['applied_sequence']
        overlay.ingest_cycle(newer, wrong, execution_epoch=1, active=True)
        palm = next(m for m in overlay.geometry(1001)[0] if m['label'] == 'Applied corrected palm left')
        np.testing.assert_array_equal(palm['position'], frame.frame.upper_limb_points[3])
        self.assertEqual(overlay.geometry(50_001_001)[1], [])
        overlay.ingest_cycle(None, None, execution_epoch=1, active=False)
        self.assertEqual(overlay.geometry(1001), ([], []))
        newer['tick_id'] = 1
        overlay.ingest_cycle(newer, None, execution_epoch=2, active=True)
        self.assertEqual(overlay.geometry(1001)[1], [])

    def test_render_targets_update_only_independent_render_data(self):
        overlay = self.make()
        root = Path(__file__).resolve().parents[1]
        model = mujoco.MjModel.from_xml_path(str(root / 'src/tianji_teleop/assets/mapped_palm/marvin_m6_wuji2.xml'))
        control, render = mujoco.MjData(model), mujoco.MjData(model)
        original = control.mocap_pos.copy()
        rgba = model.geom_rgba.copy()
        frame, native, attempt = self.sample()
        overlay.ingest_cycle(native, attempt, execution_epoch=1, active=True)
        overlay.update_render_targets(model, render, mujoco, 1001)
        for side, suffix in [('left', 'L'), ('right', 'R')]:
            index = int(model.body('target_' + suffix).mocapid[0])
            np.testing.assert_allclose(render.mocap_pos[index], native[side]['target_position'])
            np.testing.assert_allclose(render.mocap_quat[index], [1, 0, 0, 0])
        np.testing.assert_array_equal(control.mocap_pos, original)
        np.testing.assert_array_equal(model.geom_rgba, rgba)
        expected_pos, expected_quat = render.mocap_pos.copy(), render.mocap_quat.copy()
        overlay.update_render_targets(model, render, mujoco, 1001, fk_current=True)
        np.testing.assert_array_equal(render.mocap_pos, expected_pos)
        np.testing.assert_array_equal(render.mocap_quat, expected_quat)
        overlay.ingest_cycle(None, None, execution_epoch=1, active=False)
        overlay.update_render_targets(model, render, mujoco, 1001)
        for suffix in ['L', 'R']:
            index = int(model.body('target_' + suffix).mocapid[0])
            np.testing.assert_allclose(render.mocap_pos[index], render.site('hand_tcp_frame_' + suffix).xpos)

    def test_rearm_clears_old_skeleton_even_when_source_sequence_is_repeated(self):
        overlay = self.make()
        _, native, attempt = self.sample()
        overlay.ingest_cycle(native, attempt, execution_epoch=1, active=True)
        self.assertEqual(len(overlay.geometry(1001)[1]), 7)
        overlay.ingest_cycle(native, None, execution_epoch=2, active=True)
        self.assertEqual(overlay.geometry(1001)[1], [])

    def test_foreign_or_malformed_attempt_cannot_supply_skeleton(self):
        for mode in ('foreign', 'malformed', 'wrong_tick'):
            with self.subTest(mode=mode):
                overlay = self.make()
                _, native, attempt = self.sample()
                if mode == 'foreign':
                    attempt['sample']['receiver_instance_id'] = 'foreign'
                elif mode == 'malformed':
                    attempt['sample']['raw_packet_base64'] = '!'
                else:
                    attempt['tick_id'] += 1
                overlay.ingest_cycle(native, attempt, execution_epoch=1, active=True)
                self.assertEqual(overlay.geometry(1001)[1], [])

    def test_calibration_preserves_skeleton_and_adds_separate_palm_positions(self):
        overlay=self.make()
        frame,native,attempt=self.sample()
        points=frame.frame.upper_limb_points.copy()
        native['target_height_offsets_m']=[-.1,.2]
        overlay.ingest_cycle(native,attempt,execution_epoch=1,active=True)
        markers,bones=overlay.geometry(1001)
        for side,i,dz in [('left',3,-.1),('right',7,.2)]:
            palm=next(m for m in markers if m['label']==f'Z calibrated palm position {side}')
            np.testing.assert_allclose(palm['position'],points[i]+[0,0,dz])
            self.assertIsNone(palm['rotation'])
            target=next(m for m in markers if m['label']==f'Mapped-palm IK target {side}')
            np.testing.assert_allclose(target['position'],native[side]['target_position'])
        for i,point in enumerate(points):
            side='left' if i<4 else 'right'
            name=('shoulder','elbow','wrist','palm')[i%4]
            marker=next(m for m in markers if m['label']==f'Applied corrected {name} {side}')
            np.testing.assert_array_equal(marker['position'],point)
        for (a,b),(start,end) in zip(((0,4),(0,1),(1,2),(2,3),(4,5),(5,6),(6,7)),bones):
            np.testing.assert_array_equal(start,points[a])
            np.testing.assert_array_equal(end,points[b])
        np.testing.assert_array_equal(frame.frame.upper_limb_points,points)
        self.assertEqual(len(bones),7)
        again,_=overlay.geometry(1002)
        for a,b in zip(markers,again):
            np.testing.assert_array_equal(a['position'],b['position'])
        self.assertFalse(any(m['label'].startswith('Z calibrated') for m in overlay.geometry(50_001_001)[0]))
