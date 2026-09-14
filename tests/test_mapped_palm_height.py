import importlib
import os
from pathlib import Path
import struct
import subprocess
import sys
import unittest
import zlib
import tempfile

import numpy as np

from tests.test_reference_tjvr_receiver import packet
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReceivedTjvrFrame
from tianji_teleop.hand_tracking.reference_tjvr import parse_reference_tjvr_packet

ROOT = Path(__file__).resolve().parents[1]
NAME = 'pico_ee_mapped_corrected_palm_velocity_qp'


def sample(seq, now, z=1.2, epoch=9):
    raw = bytearray(packet(seq, epoch=epoch))
    struct.pack_into('<qq', raw, 24, now, now)
    points = np.array([[0,.2,1.1],[.1,.25,1.1],[.2,.3,1.1],[.3,.35,z],
                       [0,-.2,1.1],[.1,-.25,1.1],[.2,-.3,1.1],[.3,-.35,z+.02]])
    struct.pack_into('<24d', raw, 204, *points.ravel())
    struct.pack_into('<I', raw, len(raw)-4, zlib.crc32(raw[:-4]))
    return ReceivedTjvrFrame(parse_reference_tjvr_packet(bytes(raw), receiver_instance_id='height-source',
        receiver_frame_sequence=seq, received_timestamp_ns=now))


class HeightTest(unittest.TestCase):
    def make(self):
        module = 'tianji_teleop.hand_tracking.mapped_palm_height'
        self.assertIsNotNone(importlib.util.find_spec(module), 'isolated mapped height sampler missing')
        return importlib.import_module(module).MappedPalmHeight({'left':1.1, 'right':1.1})

    def test_stable_pair_only_computes_z_and_failed_retry_retains_previous_offsets(self):
        height = self.make()
        height.begin(1_000_000_000)
        for i in range(101):
            now=1_000_000_000+i*20_000_000
            height.update(sample(i+1,now),now)
        self.assertTrue(height.ready)
        np.testing.assert_allclose(height.offsets, [-.1,-.12])
        height.begin(4_000_000_000)
        height.update(None,4_300_000_000)
        self.assertEqual(height.state,'failed')
        self.assertTrue(height.ready)
        np.testing.assert_allclose(height.offsets, [-.1,-.12])

    def test_duplicate_frames_and_epoch_switch_cannot_complete_calibration(self):
        height=self.make(); height.begin(1_000_000_000)
        height.update(sample(1,1_000_000_000),1_000_000_000)
        height.update(sample(2,1_020_000_000,epoch=10),1_020_000_000)
        self.assertEqual(height.state,'failed')
        self.assertFalse(height.ready)

    def test_cli_opt_in_rejects_spark_and_accepts_mapped(self):
        command=[sys.executable,str(ROOT/'scripts/vr_manus_live.py'),'--check','--disable-hands',
                 '--mapped-palm-height-calibration']
        rejected=subprocess.run(command,capture_output=True,text=True)
        self.assertNotEqual(rejected.returncode,0)
        accepted=subprocess.run(command+['--ik-backend',NAME],capture_output=True,text=True)
        self.assertEqual(accepted.returncode,0,accepted.stderr)
        self.assertIn('"mapped_palm_height_calibration": true',accepted.stdout)

    def test_shell_rejects_other_routes_before_device_start(self):
        for profile in ('pico2_hands_sim','hand_tracking_sim','vr_manus_xr_sim'):
            row=subprocess.run(['bash',str(ROOT/'scripts/run_session.sh'),'--profile',profile,
                '--mapped-palm-height-calibration'],capture_output=True,text=True)
            self.assertEqual(row.returncode,2,row.stderr)
            self.assertIn('仅支持',row.stderr)
        row=subprocess.run(['bash',str(ROOT/'scripts/run_session.sh'),'--profile','pico_vr_manus_sim',
            '--mapped-palm-height-calibration'],capture_output=True,text=True)
        self.assertEqual(row.returncode,2)
        self.assertIn('必须显式选择 mapped-palm IK',row.stderr)

    def test_movement_duplicates_and_stale_frames_fail_without_offsets(self):
        for mode in ('movement','duplicates','stale'):
            height=self.make(); height.begin(1_000_000_000)
            for i in range(101):
                now=1_000_000_000+i*20_000_000
                source_now=now-300_000_000 if mode=='stale' else now
                frame=sample(1 if mode=='duplicates' else i+1,source_now,
                             z=1.2+(i*.002 if mode=='movement' else 0))
                height.update(frame,now)
            self.assertEqual(height.state,'failed',mode)
            self.assertFalse(height.ready,mode)


@unittest.skipUnless(os.environ.get('MAPPED_PALM_NATIVE_TEST'),'requires native mapped worker')
class NativeHeightTest(unittest.TestCase):
    def test_new_session_requires_c_and_keeps_offsets_across_home_reset(self):
        from tianji_teleop.producers.spark.mapped_height_simulation import MappedHeightSimulation
        now=[1_000_000_000]
        core=MappedHeightSimulation(ROOT,run_id='height',router_zid='router',instance_id='height',
                                   clock=lambda:now[0],backend=NAME)
        self.addCleanup(core.close)
        core.step(sample(1,now[0]))
        self.assertFalse(core.request('start').accepted)
        self.assertTrue(core.request('calibrate').accepted)
        self.assertFalse(core.request('calibrate').accepted)
        for i in range(1,402):
            now[0]+=5_000_000
            core.step(sample(i+1,now[0]))
        self.assertTrue(core.height_pending)
        self.assertFalse(core.request('start').accepted)
        self.assertTrue(core.finish_height_calibration())
        self.assertEqual(core.height_events[-1]['action'],'rearm')
        self.assertEqual(core.height_events[-1]['execution_epoch'],2)
        calibration_event=dict(core.height_events[-1],run_id='height')
        # The existing journal/schema round-trips calibration evidence and
        # original source bytes without re-encoding a shifted TJVR packet.
        from tianji_teleop.recording.session_h5 import SessionH5Writer,SessionH5Reader
        from tianji_teleop.recording.spark_reset_check import check_reset_audits
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'height.h5'
            recorded=sample(402,now[0])
            with SessionH5Writer(path,source_type='vr_manus_sim',robot_model='mapped_palm',
                                 router_zid='router',schema_version='1.2') as writer:
                writer.append_raw_reference_tjvr(recorded.observation)
                writer.append_dual_audit('operator_result',calibration_event,received_timestamp_ns=now[0])
                writer.append_dual_audit('native_cycle',dict(run_id='height',execution_epoch=2,
                    native_attempt=None),received_timestamp_ns=now[0]+1)
            with SessionH5Reader(path) as reader:
                audits=reader.read_dual_audit()
                self.assertEqual(audits[0]['payload']['target_height_offsets_m'],core.height.offsets)
                self.assertTrue(check_reset_audits(audits,run_id='height',ack_kind='mapped_palm_reset_ack')['passed'])
            import h5py
            with h5py.File(path,'r') as f:
                self.assertEqual(bytes(f['raw/tjvr_upper_limb/raw_packet'][0]),recorded.observation.frame.raw_packet)
        now[0]+=5_000_000; core.step(sample(403,now[0]))
        self.assertTrue(core.request('start').accepted)
        now[0]+=5_000_000; row=core.step(sample(404,now[0]))
        self.assertEqual(row.native_result['target_height_offsets_m'],core.height.offsets)
        self.assertFalse(core.request('calibrate').accepted)

    def test_native_z_offsets_leave_xy_rotation_and_raw_unchanged(self):
        from tianji_teleop.producers.spark.backend_assets import bilateral_assets
        from tianji_teleop.hand_tracking.spark_replay import ReplayTick
        assets=bilateral_assets(ROOT,NAME)
        results=[]
        for offsets in (None,[-.1,-.12]):
            with assets['client'](**{k:assets[k] for k in ('worker','config','model','urdf')},
                                  deterministic_test=True,startup_handshake=True) as worker:
                if offsets is not None:
                    self.assertTrue(hasattr(worker,'configure_height'),'native calibration IPC missing')
                    worker.configure_height(offsets)
                for i in range(100):
                    now=1_000_000_000+i*5_000_000
                    frame=sample(i+1,now)
                    original=frame.observation.frame.raw_packet
                    row=worker.step(ReplayTick(i+1,now,frame))
                    self.assertEqual(original,frame.observation.frame.raw_packet)
                results.append(row)
                if offsets is not None:
                    with self.assertRaises(ValueError):worker.configure_height(offsets)
                    worker.reset_at_rest(row['left']['q']+row['right']['q'],execution_epoch=2)
                    reset=worker.step(ReplayTick(1,now+5_000_000,None))
                    self.assertEqual(reset['target_height_offsets_m'],offsets)
        for side,dz in [('left',-.1),('right',-.12)]:
            np.testing.assert_allclose(np.array(results[1][side]['target_position'])-
                                       results[0][side]['target_position'],[0,0,dz],atol=1e-10)
            np.testing.assert_allclose(results[0][side]['target_quaternion_xyzw'],
                                       results[1][side]['target_quaternion_xyzw'],atol=1e-10)
        self.assertNotIn('target_height_offsets_m',results[0])
        self.assertEqual(results[1]['target_height_offsets_m'],[-.1,-.12])
