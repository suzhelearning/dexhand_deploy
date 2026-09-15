"""Same event stream must yield the same disposition in Python and C++."""
import copy
import random
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from tests import test_spark_execution_state as reference_tests
from tianji_teleop.producers.spark.native_execution import extension_path


@unittest.skipUnless(extension_path().is_file(), 'run pixi run build-native-control')
class NativeExecutionGuardTest(reference_tests.SparkExecutionStateTest):
    def make(self, **overrides):
        from tianji_teleop.producers.spark.native_execution import NativeExecutionGuard
        config = dict(run_id='run', execution_epoch=1, coordinator_instance_id='coord',
                      router_zid='router', maximum_receipt_age_ns=100, max_in_flight=2)
        config.update(overrides)
        return NativeExecutionGuard(**config)

    def test_differential_event_sequences(self):
        rng = random.Random(713)
        for case in range(120):
            native = self.make()
            reference = reference_tests.SparkExecutionStateTest.make(self)
            tick, now = 0, 100
            for index in range(25):
                now += rng.randrange(0, 12)
                action = rng.choice(('register', 'observe', 'check'))
                if action == 'register':
                    tick += 1
                    args = (tick, now, self.q(.1))
                elif action == 'observe':
                    row = self.receipt(tick=max(1,tick), q=self.q(.1))
                    row['timestamp_ns'] = now
                    if index > 4:
                        key, value = rng.choice([('accepted', False), ('run_id', 'foreign'),
                            ('timestamp_ns', now+1), ('schema_version', True),
                            ('reason', 3), ('stage', 'other'), ('tick_id', True),
                            ('command_position_rad', self.q(.2)), ('accepted', True)])
                        row[key] = value
                    args = (row, now)
                else:
                    args = (now if index % 8 else now-20,)
                outcomes = []
                for guard in (reference, native):
                    try:
                        result = getattr(guard, action)(*copy.deepcopy(args))
                        outcomes.append(('result', result))
                    except Exception as exc:
                        outcomes.append((type(exc).__name__, str(exc)))
                self.assertEqual(*outcomes, (case, index, action))
                self.assertEqual((reference.paused, reference.reason, reference.in_flight),
                                 (native.paused, native.reason, native.in_flight))

    def test_invalid_positions_and_receipt_schema(self):
        for value in (None, {}, {'left':[0.]*7}, self.q(float('nan')),
                      self.q(True), {'left':[0.]*6, 'right':[0.]*7}):
            for method in ('register', 'observe'):
                guards = [reference_tests.SparkExecutionStateTest.make(self), self.make()]
                outcomes = []
                for guard in guards:
                    try:
                        if method == 'register':
                            result = guard.register(1, 100, value)
                        else:
                            guard.register(1, 100, self.q())
                            row = self.receipt()
                            row['command_position_rad'] = value
                            result = guard.observe(row, 120)
                        outcomes.append(('result', result, guard.reason))
                    except Exception as exc:
                        outcomes.append((type(exc).__name__, str(exc)))
                self.assertEqual(*outcomes)

    def test_copies_registered_positions_and_keeps_first_fault(self):
        guard = self.make()
        q = self.q()
        guard.register(1, 100, q)
        q['left'][0] = 1.
        self.assertTrue(guard.observe(self.receipt(), 120))
        guard.pause('first')
        guard.pause('second')
        self.assertEqual(guard.reason, 'first')

    def test_fault_reason_preserves_all_python_unicode(self):
        for value in ('回执故障', 'reason'+chr(0)+'tail', chr(0xd800)):
            guard=self.make()
            guard.pause(value)
            self.assertEqual(guard.reason,value)
            with self.assertRaises(RuntimeError) as caught:
                guard.register(1,100,self.q())
            self.assertEqual(str(caught.exception),value)

    def test_cpp_selection_survives_native_home_rearm(self):
        from tianji_teleop.producers.spark.live_simulation import SparkLiveSimulation
        from tianji_teleop.producers.spark.native_execution import NativeExecutionGuard
        from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
        from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND, MAPPED_PALM_BACKEND
        from tianji_teleop.producers.spark.backend_assets import bilateral_assets
        from tests.test_reference_tjvr_receiver import packet
        root = Path(__file__).resolve().parents[1]
        for backend in (SPARK_BACKEND, MAPPED_PALM_BACKEND):
            with self.subTest(backend=backend):
                if not bilateral_assets(root,backend)['worker'].is_file():
                    self.skipTest('native IK worker must be built')
                now = [1_000_000_000]
                core = SparkLiveSimulation(root, run_id='native-guard', router_zid='router',
                    instance_id='native-guard', clock=lambda:now[0], backend=backend,
                    native_result_format='binary', execution_guard='cpp')
                try:
                    receiver = ReferenceTjvrReceiver(core.source_instance_id,.15,.6,
                        **({'target_source':'mapped_corrected_palm'} if backend == MAPPED_PALM_BACKEND else {}))
                    receiver.ingest(packet(1),now[0])
                    core.step(receiver.try_read_latest())
                    core.producer.guard.pause('explicit Home rearm')
                    ack = core.rearm_at_home()
                    self.assertEqual(ack['execution_epoch'],2)
                    self.assertIsInstance(core.producer.guard,NativeExecutionGuard)
                    self.assertFalse(core.request('start').accepted)
                    now[0] += 5_000_000
                    receiver.ingest(packet(2),now[0])
                    core.step(receiver.try_read_latest())
                    self.assertTrue(core.request('start').accepted)
                    now[0] += 5_000_000
                    result = core.step()
                    self.assertTrue(result.receipt_accepted)
                    self.assertEqual(list(core.sim.arm_state.position_rad),
                        result.native_result['left']['q']+result.native_result['right']['q'])
                finally:
                    core.close()

    def test_configuration_and_clock_boundaries_match(self):
        for field in ('execution_epoch','maximum_receipt_age_ns','max_in_flight'):
            for value in (0,-1,True,1.2,2**63):
                outcomes=[]
                for make in (lambda **kw:reference_tests.SparkExecutionStateTest.make(self,**kw),self.make):
                    with self.assertRaises(ValueError) as caught:
                        make(**{field:value})
                    outcomes.append(str(caught.exception))
                self.assertEqual(*outcomes)
        for now in (0,-1,True,2**63):
            for guard in (reference_tests.SparkExecutionStateTest.make(self),self.make()):
                with self.assertRaisesRegex(ValueError,'now_ns'):
                    guard.check(now)


class NativeExecutionSelectionTest(unittest.TestCase):
    def test_missing_native_library_is_explicit_failure(self):
        from tianji_teleop.producers.spark import native_execution as native
        from tianji_teleop.producers.spark.execution import execution_guard_type
        native.load_native.cache_clear()
        try:
            with tempfile.TemporaryDirectory() as directory, patch.object(native,'extension_path',
                    return_value=Path(directory)/'absent.so'):
                with self.assertRaisesRegex(RuntimeError,'build-native-control'):
                    execution_guard_type('cpp')
                self.assertEqual(execution_guard_type('python').__name__,'ExecutionGuard')
        finally:
            native.load_native.cache_clear()

    def test_pico2_rejects_vr_guard_flag_before_startup(self):
        root=Path(__file__).resolve().parents[1]
        result=subprocess.run(['bash',str(root/'scripts/run_session.sh'),'--profile','pico2_hands_sim',
            '--execution-guard','cpp'],text=True,capture_output=True,timeout=10)
        self.assertEqual(result.returncode,2)
        self.assertIn('仅支持 VR/TJVR',result.stderr)
