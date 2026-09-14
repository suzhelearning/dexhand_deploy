import os
from pathlib import Path
import unittest
import struct
import zlib
import tempfile
import yaml
import numpy as np
from tianji_teleop.hand_tracking.input_modes import MAPPED_PALM_BACKEND
from tianji_teleop.hand_tracking.spark_replay import ReplayTick
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.producers.spark.backend_assets import bilateral_assets
from tests.test_reference_tjvr_receiver import packet

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('MAPPED_PALM_NATIVE_TEST'), 'requires compiled mapped-palm worker')
class MappedPalmNativeTest(unittest.TestCase):
    def test_epoch_reset_uses_fk_of_latest_command_not_previous_tick(self):
        import mujoco
        selected = bilateral_assets(ROOT, MAPPED_PALM_BACKEND)
        config = yaml.safe_load(selected['config'].read_text())
        # Make the reset anchor observable: the first target after an epoch
        # switch must be one limited step from FK(q_last_command).
        step = .01
        config['safety']['max_target_position_step'] = step
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.yaml'
            path.write_text(yaml.safe_dump(config))
            selected['config'] = path
            receiver = ReferenceTjvrReceiver('reset-fk', .15, .6,
                                            target_source='mapped_corrected_palm')
            with selected['client'](**{k: selected[k] for k in ('worker', 'config', 'model', 'urdf')},
                                    deterministic_test=True, startup_handshake=True) as worker:
                previous = None
                for tick in range(1, 202):
                    now = 1_000_000_000 + (tick - 1) * 5_000_000
                    raw = bytearray(packet(tick, epoch=10 if tick == 201 else 9))
                    struct.pack_into('<qq', raw, 24, now, now)
                    struct.pack_into('<I', raw, len(raw)-4, zlib.crc32(raw[:-4]))
                    receiver.ingest(bytes(raw), now)
                    result = worker.step(ReplayTick(tick, now, receiver.try_read_latest()))
                    if tick == 200:
                        previous = result
                self.assertTrue(result['epoch_reset'])
                model = mujoco.MjModel.from_xml_path(str(selected['model']))
                data = mujoco.MjData(model)
                for side, suffix in (('left', 'L'), ('right', 'R')):
                    for index, q in enumerate(previous[side]['q'], 1):
                        data.qpos[model.joint(f'Joint{index}_{suffix}').qposadr[0]] = q
                mujoco.mj_forward(model, data)
                for side, suffix, goal in (('left', 'L', [3., 4., 5.]),
                                            ('right', 'R', [7., 8., 9.])):
                    current = data.site(f'hand_tcp_frame_{suffix}').xpos.copy()
                    delta = np.asarray(goal) - current
                    expected = current + delta * min(1., step / np.linalg.norm(delta))
                    np.testing.assert_allclose(result[side]['target_position'], expected,
                                               atol=1e-10, rtol=0)

    def test_stale_hold_and_epoch_change_preserve_continuous_model(self):
        selected = bilateral_assets(ROOT, MAPPED_PALM_BACKEND)
        receiver = ReferenceTjvrReceiver('test', .15, .6, target_source='mapped_corrected_palm')
        def sample(sequence, epoch, now):
            raw = bytearray(packet(sequence, epoch=epoch))
            struct.pack_into('<qq', raw, 24, now, now)
            struct.pack_into('<I', raw, len(raw)-4, zlib.crc32(raw[:-4]))
            receiver.ingest(bytes(raw), now)
            return receiver.try_read_latest()
        with selected['client'](**{k: selected[k] for k in ('worker','config','model','urdf')},
                                deterministic_test=True, startup_handshake=True) as worker:
            first = worker.step(ReplayTick(1, 1_000_000_000, sample(1, 9, 1_000_000_000)))
            self.assertTrue(first['input_live'])
            for tick in range(2, 14):
                held = worker.step(ReplayTick(tick, 1_000_000_000 + (tick-1)*5_000_000, None))
            self.assertFalse(held['input_live'])
            next_row = worker.step(ReplayTick(14, 1_065_000_000, sample(2, 10, 1_065_000_000)))
            self.assertTrue(next_row['epoch_reset'])
            self.assertTrue(next_row['input_live'])
            for side in ('left', 'right'):
                self.assertLess(np.max(np.abs(np.asarray(next_row[side]['q'])-held[side]['q'])), .1)

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'requires official Hand2 worker')
    def test_official_manus_hand2_and_cpp_hdf5_in_same_simulation(self):
        from tests.test_spark_live_simulation import SparkLiveSimulationTest
        SparkLiveSimulationTest._record_and_reconstruct_actual_hands(
            self, ('left', 'right'), backend=MAPPED_PALM_BACKEND)

    def test_native_identity_state_and_explicit_reset(self):
        selected = bilateral_assets(ROOT, MAPPED_PALM_BACKEND)
        with selected['client'](**{k: selected[k] for k in ('worker', 'config', 'model', 'urdf')},
                                deterministic_test=True, startup_handshake=True) as worker:
            row = worker.step(ReplayTick(1, 1_000_000_000, None))
            self.assertEqual(row['algorithm'], MAPPED_PALM_BACKEND)
            self.assertFalse(row['input_live'])
            q = row['left']['q'] + row['right']['q']
            ack = worker.reset_at_rest(q, execution_epoch=2)
            self.assertEqual(ack['position_rad'], q)
            self.assertEqual(ack['velocity_rad_s'], [0.] * 14)
            self.assertEqual(worker.step(ReplayTick(1, 1_005_000_000, None))['tick_id'], 1)

    def test_managed_simulation_emits_mapped_backend_commands(self):
        from tianji_teleop.producers.spark.live_simulation import SparkLiveSimulation
        now = [1_000_000_000]
        core = SparkLiveSimulation(ROOT, run_id='mapped-test', router_zid='test-router',
            instance_id='mapped', clock=lambda: now[0], backend=MAPPED_PALM_BACKEND)
        self.addCleanup(core.close)
        receiver = ReferenceTjvrReceiver('mapped-source', .15, .6, target_source='mapped_corrected_palm')
        receiver.ingest(packet(1), now[0])
        core.step(receiver.try_read_latest())
        self.assertTrue(core.request('start').accepted)
        now[0] += 5_000_000
        row = core.step()
        self.assertIsNotNone(row.native_result)
        self.assertEqual(row.native_result['algorithm'], MAPPED_PALM_BACKEND)
        self.assertTrue(row.receipt_accepted)
        self.assertEqual(set(core.last_timing), {
            'input_and_feedback', 'coordinator_cycle', 'command_and_simulation',
            'request_encode', 'ipc_roundtrip_decode', 'result_validate'})
        self.assertTrue(all(value >= 0 for value in core.last_timing.values()))
        self.assertNotIn('timing', row.native_result)
        self.assertEqual(core.authorities['producer_arm']['logical_id'], 'ik_mapped_palm')
        np.testing.assert_allclose(core.sim.arm_state.position_rad,
            row.native_result['left']['q'] + row.native_result['right']['q'])
