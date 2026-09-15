"""Cross-language result equivalence and fail-closed framing tests."""
from contextlib import ExitStack
from pathlib import Path
import struct
import subprocess
import sys
import unittest
from unittest.mock import patch

from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND, MAPPED_PALM_BACKEND
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.hand_tracking.spark_replay import ReplayTick
from tianji_teleop.producers.spark.backend_assets import bilateral_assets
from tests.test_reference_tjvr_receiver import packet

ROOT = Path(__file__).resolve().parents[1]


class NativeBinaryResultsTest(unittest.TestCase):
    def test_pico2_rejects_vr_only_result_format_before_runtime(self):
        result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
            '--profile', 'pico2_hands_sim', '--native-result-format', 'binary'],
            text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 2)
        self.assertIn('仅支持 VR/TJVR', result.stderr)

    def test_binary_backend_runs_through_coordinator_and_mujoco(self):
        from tianji_teleop.producers.spark.live_simulation import SparkLiveSimulation
        for backend in (SPARK_BACKEND, MAPPED_PALM_BACKEND):
            with self.subTest(backend=backend):
                if not bilateral_assets(ROOT, backend)['worker'].is_file():
                    self.skipTest('native worker must be built')
                now = [1_000_000_000]
                core = SparkLiveSimulation(ROOT, run_id='binary-sim', router_zid='test-router',
                    instance_id='binary', clock=lambda: now[0], backend=backend,
                    native_result_format='binary')
                try:
                    receiver = ReferenceTjvrReceiver('binary-source', .15, .6,
                        **({'target_source': 'mapped_corrected_palm'}
                           if backend == MAPPED_PALM_BACKEND else {}))
                    receiver.ingest(packet(1), now[0])
                    core.step(receiver.try_read_latest())
                    self.assertTrue(core.request('start').accepted)
                    now[0] += 5_000_000
                    result = core.step()
                    self.assertTrue(result.receipt_accepted)
                    self.assertEqual(list(core.sim.arm_state.position_rad),
                        result.native_result['left']['q'] + result.native_result['right']['q'])
                    self.assertIn('native_step', core.last_timing)
                    self.assertNotIn('_native_step_ns', result.native_result)
                finally:
                    core.close()

    def test_transport_failure_poisoning_and_short_reads(self):
        from tianji_teleop.hand_tracking.native_binary_results import frame_size
        from tianji_teleop.hand_tracking.spark_worker_client import SparkWorkerClient
        frame = bytearray(frame_size('spark'))
        struct.pack_into('<4sBBH', frame, 0, b'TJBR', 1, 1, len(frame)-8)
        struct.pack_into('<QQ', frame, 9, 1, 1_000_000_000)
        real_popen = subprocess.Popen
        for variant in ('short_reads', 'partial', 'extra', 'wrong_identity', 'nonfinite'):
            with self.subTest(variant=variant):
                value = bytearray(frame)
                if variant == 'partial':
                    value = value[:12]
                elif variant == 'extra':
                    value += b'extra'
                elif variant == 'wrong_identity':
                    struct.pack_into('<Q', value, 9, 99)
                elif variant == 'nonfinite':
                    struct.pack_into('<d', value, 66, float('inf'))
                code = 'import sys,os,time; sys.stdin.readline(); data=' + repr(bytes(value)) + '\n'
                if variant == 'short_reads':
                    code += 'for b in data:\n os.write(1, bytes([b])); time.sleep(.00001)\n'
                else:
                    code += 'os.write(1,data)\n'
                code += 'sys.stdin.readline()\n'
                def spawn(*args, **kwargs):
                    return real_popen([sys.executable, '-c', code], **kwargs)
                with patch('tianji_teleop.hand_tracking.spark_worker_client.subprocess.Popen',
                           side_effect=spawn):
                    with SparkWorkerClient(**{k: sys.executable for k in
                        ('worker', 'config', 'model', 'urdf')}, result_format='binary',
                        timeout_seconds=.5) as client:
                        if variant == 'short_reads':
                            self.assertEqual(client.step(ReplayTick(1, 1_000_000_000, None))['tick_id'], 1)
                        else:
                            with self.assertRaises((TimeoutError, RuntimeError, ValueError)):
                                client.step(ReplayTick(1, 1_000_000_000, None))
                            with self.assertRaisesRegex(RuntimeError, 'closed'):
                                client.step(ReplayTick(1, 1_000_000_000, None))

    def test_binary_and_json_results_match_through_reset_and_height(self):
        for backend in (SPARK_BACKEND, MAPPED_PALM_BACKEND):
            with self.subTest(backend=backend), ExitStack() as stack:
                selected = bilateral_assets(ROOT, backend)
                if not selected['worker'].is_file():
                    self.skipTest('native worker must be built')
                options = {k: selected[k] for k in ('worker', 'config', 'model', 'urdf')}
                clients = [stack.enter_context(selected['client'](**options,
                    deterministic_test=True, startup_handshake=True, result_format=fmt))
                    for fmt in ('json', 'binary')]
                receiver = ReferenceTjvrReceiver('binary-test', .15, .6,
                    **({'target_source': 'mapped_corrected_palm'}
                       if backend == MAPPED_PALM_BACKEND else {}))
                if backend == MAPPED_PALM_BACKEND:
                    self.assertEqual(*[c.configure_height([-.2, -.3]) for c in clients])
                for tick in range(1, 26):
                    now = 1_000_000_000 + tick * 5_000_000
                    sample = None
                    if tick in (1, 2, 24):
                        receiver.ingest(packet(tick, epoch=10 if tick == 24 else 9), now)
                        sample = receiver.try_read_latest()
                    results = [c.step(ReplayTick(tick, now, sample)) for c in clients]
                    self.assertEqual(*results)
                q = results[0]['left']['q'] + results[0]['right']['q']
                self.assertEqual(*[c.reset_at_rest(q, execution_epoch=2) for c in clients])
                self.assertEqual(*[c.step(ReplayTick(1, 2_000_000_000, None)) for c in clients])

    def test_reject_corrupt_binary_header(self):
        from tianji_teleop.hand_tracking.native_binary_results import payload_size
        for header in (b'BAD!\x01\0\0\0', b'TJBR\x02\x01\0\0',
                       b'TJBR\x01\xff\0\0', b'TJBR\x01\x01\xff\xff'):
            with self.subTest(header=header), self.assertRaises(ValueError):
                payload_size(header, 'spark')

    def test_reject_nonfinite_and_invalid_boolean(self):
        from tianji_teleop.hand_tracking.native_binary_results import decode_result, frame_size
        data = bytearray(frame_size('mapped_palm'))
        struct.pack_into('<4sBBH', data, 0, b'TJBR', 1, 2, len(data)-8)
        data[8] = 2  # strict bool, not a truthy arbitrary byte
        with self.assertRaises(ValueError):
            decode_result(bytes(data), 'mapped_palm')
        data[8] = 0
        struct.pack_into('<d', data, 65, float('nan'))  # first left q after common/height fields
        with self.assertRaises(ValueError):
            decode_result(bytes(data), 'mapped_palm')
