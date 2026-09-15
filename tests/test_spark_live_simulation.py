import importlib.util
import os
from pathlib import Path
import unittest
import time
import sys
import tempfile
from unittest.mock import patch

from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tests.test_reference_tjvr_receiver import packet
from tests.test_hand_retarget_loop import Source
from tests.test_hand_producer_authority import Backend
from tianji_teleop.hand_tracking.reference_manus_process import ManusCallback

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional actual live simulation core')
class SparkLiveSimulationTest(unittest.TestCase):
    def make(self):
        name = 'tianji_teleop.producers.spark.live_simulation'
        self.assertIsNotNone(importlib.util.find_spec(name))
        from tianji_teleop.producers.spark.live_simulation import SparkLiveSimulation
        self.now = 1_000_000_000
        core = SparkLiveSimulation(ROOT, run_id='test', router_zid='router', instance_id='owner',
                                   clock=lambda: self.now)
        self.addCleanup(core.close)
        receiver = ReferenceTjvrReceiver('owner-source', .15, .6)
        receiver.ingest(packet(1), self.now)
        return core, receiver.try_read_latest()

    def test_live_core_requires_explicit_intent_and_uses_original_budgets(self):
        core, sample = self.make()
        import yaml
        home=yaml.safe_load((ROOT/'src/tianji_teleop/config/robot/arm.yaml').read_text())
        self.assertEqual(list(core.sim.robot.left_home_rad), home['left_home_rad'])
        self.assertEqual(list(core.sim.robot.right_home_rad), home['right_home_rad'])
        core.step(sample)
        self.assertEqual(core.coordinator.state.state, 'idle')
        self.assertEqual(core.hand_telemetry, {})
        self.assertTrue(core.request('start').accepted)
        self.now += 5_000_000
        with patch.object(core.coordinator, 'tick', wraps=core.coordinator.tick) as tick:
            result = core.step()
        self.assertEqual(tick.call_count, 1)
        self.assertTrue(result.receipt_accepted)
        self.assertFalse(result.native_result['deterministic_test'])
        self.assertEqual(result.native_result['guidance_updates'], 1)
        self.assertEqual(core.sim.arm_state.position_rad,
                         result.commands['left'].position_rad + result.commands['right'].position_rad)

    def test_slow_viewer_startup_does_not_latch_executor_unhealthy(self):
        core, _ = self.make()
        self.now += 2_000_000_000
        receiver = ReferenceTjvrReceiver('owner-source', .15, .6)
        receiver.ingest(packet(1), self.now)
        core.step(receiver.try_read_latest())
        self.assertTrue(core.sim.status.healthy, core.sim.status.error)
        self.assertTrue(core.request('start').accepted)

    def test_worker_failure_keeps_both_arms_and_latches_fault(self):
        core, sample = self.make()
        core.step(sample)
        core.request('start')
        self.now += 5_000_000
        core.step()
        previous = list(core.sim.arm_state.position_rad)
        self.now += 5_000_000
        with patch.object(core.producer.backend, 'step', side_effect=RuntimeError('lost')):
            result = core.step()
        self.assertEqual(core.coordinator.state.state, 'fault')
        self.assertEqual(core.sim.arm_state.position_rad, previous)
        self.assertFalse(core.request('start').accepted)
        self.assertFalse(result.receipt_accepted)

    def test_home_rearm_requires_new_input_and_explicit_start(self):
        core, sample = self.make()
        core.step(sample)
        core.request('start')
        self.now += 5_000_000
        core.step()
        old_receipt = core.coordinator.last_bilateral_receipt
        self.assertTrue(hasattr(core, 'rearm_at_home'))
        with self.assertRaises(ValueError):
            core.rearm_at_home()
        core.request('return')
        for _ in range(2000):
            self.now += 5_000_000
            core.step()
            if core.coordinator.state.state == 'idle':
                break
        self.assertEqual(core.coordinator.state.state, 'idle')
        ack = core.rearm_at_home()
        self.assertEqual(ack['execution_epoch'], 2)
        self.assertEqual(core.producer.guard.execution_epoch, 2)
        self.assertFalse(core.request('start').accepted)
        self.assertFalse(core.producer.observe_execution(old_receipt, self.now))
        receiver = ReferenceTjvrReceiver('owner-source', .15, .6)
        receiver.ingest(packet(1), self.now)
        receiver.ingest(packet(2), self.now)
        buffered = receiver.try_read_latest()
        self.now += 5_000_000
        result = core.step(buffered)
        self.assertIsNone(result.native_result)
        self.assertFalse(core.request('start').accepted)
        self.now += 5_000_000
        receiver.ingest(packet(3), self.now)
        result = core.step(receiver.try_read_latest())
        self.assertIsNone(result.native_result)
        self.assertTrue(core.request('start').accepted)
        self.now += 5_000_000
        result = core.step()
        self.assertEqual(result.native_result['tick_id'], 1)
        self.assertEqual(core.coordinator.last_bilateral_receipt['execution_epoch'], 2)

    def test_source_transport_failure_faults_but_tracking_loss_uses_native_hold(self):
        core, sample = self.make()
        core.step(sample)
        core.request('start')
        self.now += 5_000_000
        core.step()
        self.now += 5_000_000
        core.step(source_failure='UDP socket lost')
        self.assertEqual(core.coordinator.state.state, 'fault')

    def test_source_failure_before_start_is_visible_to_launcher(self):
        core, _ = self.make()
        core.step(source_failure='UDP socket lost before start')
        self.assertFalse(core.request('start').accepted)
        self.assertTrue(hasattr(core, 'failure'))
        self.assertEqual(core.failure, 'UDP socket lost before start')

    def test_required_hands_gate_start_then_commands_reach_same_simulator(self):
        from tianji_teleop.producers.spark.live_simulation import SparkLiveSimulation
        self.now = 1_000_000_000
        source = Source()
        recorded = []
        core = SparkLiveSimulation(ROOT, run_id='test', router_zid='router', instance_id='owner',
            clock=lambda: self.now, hand_sides=('left', 'right'), hand_source=source, hand_backend=Backend(),
            hand_command_sink=recorded.append)
        self.addCleanup(core.close)
        receiver = ReferenceTjvrReceiver('owner-source', .15, .6)
        receiver.ingest(packet(1), self.now)
        core.step(receiver.try_read_latest())
        self.assertFalse(core.request('start').accepted)
        def callback(seq):
            source.rows.append(ManusCallback('owner-manus', seq, self.now, (0.,) * 126,
                                            {'left': seq, 'right': seq}, {'left': seq, 'right': seq}))
            deadline = time.monotonic() + 2
            while core.hand_loop.processed_callbacks < seq and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertEqual(core.hand_loop.processed_callbacks, seq)
        callback(1)
        self.now += 5_000_000
        core.step()
        self.assertTrue(core.request('start').accepted)
        self.now += 5_000_000
        core.step()
        callback(2)
        self.now += 5_000_000
        core.step()
        self.assertEqual(core.sim.hand_state('left').position_rad, [i / 100 for i in range(20)])
        self.assertTrue(hasattr(core, 'last_hand_commands'))
        self.assertEqual(set(core.last_hand_commands), {'left', 'right'})
        self.assertEqual(core.last_hand_commands['left'].position_rad, core.sim.hand_state('left').position_rad)
        self.assertEqual(recorded[-1], core.last_hand_commands)
        self.assertEqual(core.coordinator.state.state, 'teleop')

        telemetry = core.hand_telemetry
        self.assertEqual(set(telemetry), {'producer', 'executors'})
        self.assertEqual(set(telemetry['executors']), {'left', 'right'})
        self.assertGreaterEqual(telemetry['producer']['diagnostics']['processed_callbacks'], 2)
        self.assertIn('max_callback_age_ns', telemetry['producer']['diagnostics'])
        self.assertIn('max_retarget_duration_ns', telemetry['producer']['diagnostics'])
        telemetry['producer']['diagnostics']['processed_callbacks'] = -1
        self.assertGreaterEqual(core.hand_telemetry['producer']['diagnostics']['processed_callbacks'], 2)

        core.request('return')
        for _ in range(2000):
            self.now += 5_000_000
            core.step()
            if core.coordinator.state.state == 'idle':
                break
        self.assertEqual(core.coordinator.state.state, 'idle')
        reset = core.producer.backend.reset_at_rest
        def slow_reset(*args, **kwargs):
            result = reset(*args, **kwargs)
            self.now += 1_500_000_000
            return result
        with patch.object(core.producer.backend, 'reset_at_rest', side_effect=slow_reset):
            ack = core.rearm_at_home()
        self.assertEqual(ack['execution_epoch'], 2)
        self.assertTrue(core.sim.status.healthy)
        self.assertFalse(core.request('start').accepted)
        for side in core.hand_sides:
            self.assertTrue(core.sim.hand_config.at_zero(core.sim.hand_state(side).position_rad))

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional official hand worker')
    def test_rawviz_process_and_official_hand_worker_join_actual_spark_clock(self):
        self._record_and_reconstruct_actual_hands(('left', 'right'))

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional official single-left worker')
    def test_single_left_rawviz_worker_simulation_and_recording_are_consistent(self):
        self._record_and_reconstruct_actual_hands(('left',))

    @unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional native hand environment')
    def test_cpp_hand_scheduler_with_cpp_arm_adapters_and_recording(self):
        self._record_and_reconstruct_actual_hands(('left', 'right'), scheduler_backend='cpp')

    def _record_and_reconstruct_actual_hands(self, sides, *, backend=None, scheduler_backend='python'):
        from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND, MAPPED_PALM_BACKEND
        backend = backend or SPARK_BACKEND
        target_source = 'mapped_corrected_palm' if backend == MAPPED_PALM_BACKEND else 'packet'
        from tianji_teleop.producers.spark.live_simulation import SparkLiveSimulation
        from tianji_teleop.producers.spark.live_runner import _InputSlot, _make_hand_client
        from tianji_teleop.hand_tracking.reference_manus_process import ReferenceManusProcess
        from tests.test_reference_manus_process import rawviz_records
        from tests.test_gesture_recognition import hand_points
        from tianji_teleop.recording.async_dual import AsyncDualRecorder
        from tianji_teleop.recording.live_capture import LiveCapture
        from tianji_teleop.recording.session_h5 import SessionH5Reader
        from tianji_teleop.recording.hand_command_check import (
            hand_replay_asset_hashes, check_manus_hand_commands)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        capture_path = Path(directory.name) / 'joined.h5'
        recorder = AsyncDualRecorder(capture_path, router_zid='router', metadata=dict(
            synthetic=True, run_id='joined', resolved_configuration=dict(asset_sha256=hand_replay_asset_hashes(ROOT),
                manus_filter_continuity_ns=200_000_000,
                tjvr_stream_contract=dict(version=1, initial_state='reset',
                    max_position_jump_m=.15, max_orientation_jump_rad=.6, target_source=target_source),
                manus_input_contract=dict(
                version=1, sides=[side for side in ('right', 'left') if side in sides],
                right_glove=None, left_glove=None,
                callback_order='right_then_left', callback_trigger='each_accepted_pose'))))
        self.addCleanup(recorder.close)
        capture = LiveCapture(recorder, run_id='joined')
        capture.audit('lifecycle', {'stage': 'opening'})
        hand = _make_hand_client(ROOT, sides, scheduler_backend=scheduler_backend)
        self.addCleanup(hand.close)
        slot = _InputSlot()
        core = SparkLiveSimulation(ROOT, run_id='joined', router_zid='router', instance_id='joined',
            backend=backend,
            **(dict(native_result_format='binary', execution_guard='cpp', simulation_backend='cpp',
                    coordinator_math='cpp') if scheduler_backend == 'cpp' else {}),
            hand_sides=sides, hand_source=slot, hand_backend=hand,
            hand_command_sink=capture.hand_output,
            hand_expired_input_sink=lambda row: capture.audit(
                'manus_superseded_input' if row['reason'] == 'superseded_before_retarget'
                else 'manus_expired_input', row))
        self.addCleanup(core.close)
        # Nominal throughput requires meter-scale non-collinear hands, not
        # the tens-of-meters collinear semantic-index fixture. Keep the
        # original per-POSE callbacks, input rate and 200ms execution gate.
        points = hand_points()
        template = (rawviz_records('right', 1, canonical_points=points) +
                    rawviz_records('left', 1, canonical_points=points)).replace(
            'POSE right-glove 1 ', 'POSE right-glove {sequence} ').replace(
            'POSE left-glove 1 ', 'POSE left-glove {sequence} ')
        # Original EnsureTopology prints HAND/NODE only on first discovery;
        # subsequent callbacks carry POSE only, without repeated metadata.
        topology = '\n'.join(line for line in template.splitlines() if not line.startswith('POSE '))
        poses = '\n'.join(line for line in template.splitlines() if line.startswith('POSE '))
        program = ('import time\nprint(' + repr(topology) + ',flush=True)\nsequence=0\nwhile True:\n'
                   ' sequence+=1\n print(' + repr(poses) + '.format(sequence=sequence),flush=True)\n time.sleep(.02)\n')
        slot.source = ReferenceManusProcess(command=[sys.executable, '-u', '-c', program],
            receiver_instance_id='joined-manus', sides=sides,
            raw_line_sink=capture.rawviz, callback_sink=capture.callback)
        self.addCleanup(slot.source.close)
        receiver = ReferenceTjvrReceiver('joined-source', .15, .6,
            target_source=target_source,
            raw_frame_sink=capture.tjvr, decision_sink=capture.tjvr_decision)
        sequence, commanded = 0, False
        started = time.monotonic()
        deadline = started + 5
        # Exercise ongoing capture, not just the first successful hand command.
        while time.monotonic() < deadline and (not commanded or time.monotonic() - started < 2):
            sequence += 1
            receiver.ingest(packet(sequence), time.monotonic_ns())
            result = core.step(receiver.try_read_latest())
            capture.cycle(core, result)
            self.assertNotEqual(core.coordinator.state.state, 'fault', {
                'reason': core.coordinator.state.reason,
                'hand': core.hand_telemetry,
                'hand_loop_failure': core.hand_loop.failure,
                'now_ns': time.monotonic_ns(),
            })
            if core.coordinator.state.state == 'idle':
                core.request('start')
            commanded = set(core.last_hand_commands) == set(sides)
            time.sleep(.005)
        self.assertTrue(commanded, core.hand_loop.failure)
        self.assertIsNone(recorder.failure)
        self.assertGreater(core.hand_loop.processed_callbacks, 10)
        self.assertEqual(core.coordinator.state.state, 'teleop')
        for side in sides:
            self.assertEqual(core.sim.hand_state(side).position_rad, core.last_hand_commands[side].position_rad)
        slot.source.close()
        core.close()
        recorder.close()
        with SessionH5Reader(capture_path) as reader:
            self.assertGreater(len(reader.read_manus_callbacks()), 0)
            self.assertGreater(len(reader.read_hand_command('left')), 0)
            if sides == ('left',):
                self.assertEqual(reader.read_hand_command('right'), [])
            kinds = {row['kind'] for row in reader.read_dual_audit()}
            self.assertTrue({'manus_rawviz_line', 'manus_callback_metadata', 'hand_output'} <= kinds)
            statuses = [row['payload'] for row in reader.read_dual_audit()
                        if row['kind'] == 'component_status']
            self.assertTrue(statuses)
            self.assertIn('max_callback_age_ns', statuses[-1]['producer']['diagnostics'])
        from tianji_teleop.recording.manus_check import check_manus_recording
        reconstruction = check_manus_recording(capture_path)
        self.assertTrue(reconstruction['passed'], reconstruction)
        self.assertGreater(reconstruction['matched_callbacks'], 10)
        if scheduler_backend == 'python':
            joint_reconstruction = check_manus_hand_commands(capture_path, root=ROOT)
            self.assertTrue(joint_reconstruction['passed'], joint_reconstruction)
            self.assertGreater(joint_reconstruction['matched_commands'], 10)
        else:
            # Latest-result coalescing does not identify every stateful solver
            # consumption; do not replay all raw inputs and claim equivalence.
            self.assertIsNone(hand.async_diagnostics['failure'])
            self.assertGreater(hand.async_diagnostics['last_output_sequence'], 10)
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        gate_reconstruction = check_tjvr_recording(capture_path)
        self.assertTrue(gate_reconstruction['passed'], gate_reconstruction)
        self.assertGreater(gate_reconstruction['matched_decisions'], 10)
        self.assertEqual(gate_reconstruction['native_input_check'], 'checked')
        self.assertGreater(gate_reconstruction['matched_native_inputs'], 10)
