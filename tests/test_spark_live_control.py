import time
import unittest
from threading import Event, get_ident
from types import SimpleNamespace
from unittest.mock import patch

import mujoco


class SparkLiveControlLoopTest(unittest.TestCase):
    def test_exit_trigger_preserves_interruption_during_home_return(self):
        from tianji_teleop.producers.spark.live_runner import live_exit_trigger
        self.assertEqual(live_exit_trigger(None, 'returning', True, 'SIGINT'), 'SIGINT')
        self.assertEqual(live_exit_trigger(None, 'idle', True, 'control_stop'), 'operator_shutdown')
        self.assertEqual(live_exit_trigger(None, 'returning', True, 'viewer_closed'), 'viewer_closed')
        self.assertEqual(live_exit_trigger('worker failed', 'returning', True, 'SIGINT'), 'fault')
        self.assertEqual(live_exit_trigger(None, 'fault', False, 'control_stop'), 'fault')

    def test_live_runner_bounds_python_thread_switch_interval(self):
        import tianji_teleop.producers.spark.live_runner as live_runner

        with patch.object(live_runner.sys, 'getswitchinterval', return_value=.005), \
             patch.object(live_runner.sys, 'setswitchinterval') as set_interval:
            previous = live_runner._bound_live_thread_switch_interval()
        self.assertEqual(previous, .005)
        set_interval.assert_called_once_with(.001)

        with patch.object(live_runner.sys, 'getswitchinterval', return_value=.0005), \
             patch.object(live_runner.sys, 'setswitchinterval') as set_interval:
            previous = live_runner._bound_live_thread_switch_interval()
        self.assertEqual(previous, .0005)
        set_interval.assert_not_called()

    def test_live_result_status_controls_recording_completion(self):
        import tianji_teleop.producers.spark.live_runner as live_runner

        class Recorder:
            def __init__(self):
                self.completions = []

            def close(self, *, complete):
                self.completions.append(complete)

        recorder = Recorder()
        live_runner._close_live_recording(recorder, exit_code=1)
        live_runner._close_live_recording(recorder, exit_code=0)
        live_runner._close_live_recording(None, exit_code=1)

        self.assertEqual(recorder.completions, [False, True])

    def test_startup_summary_omits_asset_dump(self):
        from tianji_teleop.producers.spark.live_runner import startup_summary
        resolved = {'config': {'ik_backend': 'mapped', 'hands_enabled': False, 'rate_hz': 200},
                    'record_path': '/tmp/session.h5', 'asset_sha256': {'large_mesh': 'abc'},
                    'mapped_palm_height_calibration': True}
        text = startup_summary('run', resolved)
        self.assertIn('IK=mapped', text)
        self.assertIn('/tmp/session.h5', text)
        self.assertNotIn('large_mesh', text)
        self.assertEqual(resolved['asset_sha256'], {'large_mesh': 'abc'})

    def test_final_recording_audit_is_skipped_after_recorder_failure(self):
        import tianji_teleop.producers.spark.live_runner as live_runner

        class Recorder:
            failure = 'recording queue overflow; incomplete capture'

        class Capture:
            recorder = Recorder()

            def audit(self, *_args, **_kwargs):
                raise AssertionError('failed recorder must not receive another append')

        self.assertFalse(live_runner._audit_live_capture(
            Capture(), 'lifecycle', {'stage': 'complete'}))

    def test_live_capture_ignores_callbacks_after_recording_failure(self):
        from tianji_teleop.recording.live_capture import LiveCapture

        class Recorder:
            failure = 'recording queue overflow; incomplete capture'

            def append(self, *_args, **_kwargs):
                raise AssertionError('failed recorder must not receive another append')

        capture = LiveCapture(Recorder(), run_id='run')
        capture.audit('operator_result', {'action': 'start'})
        capture.rawviz('diagnostic', 123)

        self.assertEqual(capture._rawviz_sequence, 1)

    def test_render_state_is_copied_to_independent_mujoco_data(self):
        from tianji_teleop.coordination.arm_command_coordinator import ArmCommandCoordinator
        from tianji_teleop.producers.spark.factory import reference_robot_config
        from tianji_teleop.executors.mujoco.node import MujocoExecutor

        assets = __import__('pathlib').Path(__file__).resolve().parents[1] / 'src/tianji_teleop'
        model = mujoco.MjModel.from_xml_path(str(assets / 'assets/spark/marvin_m6_wuji2.xml'))
        robot = reference_robot_config(
            assets / 'config/producers/spark_reference.yaml',
            assets / 'assets/spark/marvin_m6_s_ccs_696_v4_local.urdf',
            assets / 'config/robot/arm.yaml',
        )
        sim = MujocoExecutor(
            model=model, data=mujoco.MjData(model), robot_config=robot,
            publisher_instance_id='sim', router_zid='router', coordinator_instance_id='coord',
            hand_sides=(), session=None,
        )
        self.addCleanup(sim.close)
        render_data = mujoco.MjData(model)
        positions = [0.01 * index for index in range(14)]

        sim.copy_joint_state_to(render_data, positions)

        for side_index, side in enumerate(('left', 'right')):
            for joint_index, name in enumerate(getattr(robot, f'{side}_joint_names')):
                address = sim._arm_addresses[side][name]
                self.assertAlmostEqual(float(render_data.qpos[address]), positions[side_index * 7 + joint_index])

    def test_spark_control_owner_serializes_core_and_operator_actions(self):
        from tianji_teleop.producers.spark.live_control import (
            SnapshotSideEffectLoop,
            SparkControlLoop,
        )

        class Status:
            def to_dict(self):
                return {'ready': True}

        class Core:
            hand_sides = ()
            failure = None

            def __init__(self):
                self.clock_value = 1_000_000_000
                self.threads = []
                self.source_status = Status()
                self.producer = SimpleNamespace(
                    guard=SimpleNamespace(execution_epoch=1),
                    status=lambda _now: Status(),
                )
                self.sim = SimpleNamespace(status=Status(), arm_state='arm-state')
                self.coordinator = SimpleNamespace(
                    state=SimpleNamespace(state='idle', reason='test'),
                    last_bilateral_receipt=None,
                    last_bilateral_command=None,
                )

            def clock(self):
                self.clock_value += 5_000_000
                return self.clock_value

            def step(self, _sample=None, *, source_failure=None):
                self.threads.append(('step', get_ident()))
                # Simulate native tick IDs restarting across Home resets.
                tick = 1 + (sum(kind == 'step' for kind, _ in self.threads) - 1) % 3
                return SimpleNamespace(
                    commands={}, native_result={'tick_id': tick}, native_attempt=None,
                    receipt_accepted=False,
                )

            def request(self, action):
                self.threads.append((action, get_ident()))
                return SimpleNamespace(accepted=True, reason='accepted')

            @property
            def hand_telemetry(self):
                return {}

            @property
            def last_hand_commands(self):
                return {}

        class Receiver:
            def try_read_latest(self):
                return None

        core = Core()
        stop = Event()
        side_effects = SnapshotSideEffectLoop(lambda _snapshot: None)
        loop = SparkControlLoop(core, Receiver(), stop_event=stop, period_s=0.005,
                                failure_fn=lambda: None, snapshot_sink=side_effects)
        side_effects.start()
        loop.start()
        time.sleep(0.030)
        self.assertTrue(loop.submit('start'))
        time.sleep(0.030)
        stop.set()
        loop.join(1.0)
        side_effects.close()

        self.assertFalse(loop.is_alive())
        self.assertGreaterEqual(loop.tick_count, 8)
        self.assertEqual(loop.native_ticks_total, loop.tick_count)
        self.assertLessEqual(loop.native_ticks, 3)
        self.assertEqual(loop.timing['core_step']['count'], loop.tick_count)
        self.assertEqual(loop.timing['schedule_lag']['count'], loop.tick_count)
        self.assertGreaterEqual(loop.timing['cycle_work']['max_ms'],
                                loop.timing['core_step']['max_ms'])
        thread_ids = {thread_id for _kind, thread_id in core.threads}
        self.assertEqual(len(thread_ids), 1)
        self.assertNotEqual(thread_ids.pop(), get_ident())
        self.assertTrue(any(kind == 'start' for kind, _thread_id in core.threads))

    def test_operator_report_is_queued_before_the_following_control_snapshot(self):
        from tianji_teleop.producers.spark.live_control import (
            LiveOperatorEvent,
            SnapshotSideEffectLoop,
            SparkControlLoop,
        )

        class Status:
            def to_dict(self):
                return {'ready': True}

        class Core:
            hand_sides = ()
            failure = None

            def __init__(self):
                self.clock_value = 1_000_000_000
                self.source_status = Status()
                self.producer = SimpleNamespace(
                    guard=SimpleNamespace(execution_epoch=1),
                    status=lambda _now: Status(),
                )
                self.sim = SimpleNamespace(status=Status(), arm_state='arm-state')
                self.coordinator = SimpleNamespace(
                    state=SimpleNamespace(state='idle', reason='test'),
                    last_bilateral_receipt=None,
                    last_bilateral_command=None,
                )

            def clock(self):
                self.clock_value += 5_000_000
                return self.clock_value

            def step(self, _sample=None, *, source_failure=None):
                return SimpleNamespace(commands={}, native_result=None,
                                       native_attempt=None, receipt_accepted=False)

            def request(self, _action):
                return SimpleNamespace(accepted=True, reason='accepted')

            @property
            def hand_telemetry(self):
                return {}

            @property
            def last_hand_commands(self):
                return {}

        events = []
        stop = Event()
        side_effects = SnapshotSideEffectLoop(events.append)
        loop = SparkControlLoop(Core(), SimpleNamespace(try_read_latest=lambda: None),
                                stop_event=stop, period_s=0.005, failure_fn=lambda: None,
                                snapshot_sink=side_effects)
        side_effects.start()
        loop.start()
        time.sleep(0.015)
        self.assertTrue(loop.submit('start'))
        deadline = time.monotonic() + 1.0
        while not loop.poll_reports() and time.monotonic() < deadline:
            time.sleep(0.001)
        stop.set()
        loop.join(1.0)
        side_effects.close()

        event_index = next(index for index, item in enumerate(events)
                           if isinstance(item, LiveOperatorEvent))
        snapshot_index = next(index for index, item in enumerate(events[event_index + 1:], event_index + 1)
                              if not isinstance(item, LiveOperatorEvent))
        self.assertLess(event_index, snapshot_index)

    def test_async_output_owns_zenoh_puts_off_the_control_thread(self):
        from tianji_teleop.producers.spark.live_output import AsyncLiveOutput
        import tianji_teleop.producers.spark.live_output as live_output

        class Publisher:
            def __init__(self):
                self.values = []
                self.undeclared = False

            def put(self, payload, encoding=None):
                self.values.append((payload, encoding))

            def undeclare(self):
                self.undeclared = True

        class Session:
            def __init__(self):
                self.publishers = []
                self.values = []

            def declare_publisher(self, _topic):
                publisher = Publisher()
                self.publishers.append(publisher)
                return publisher

            def put(self, topic, payload, encoding=None):
                self.values.append((topic, payload, encoding))

        session = Session()
        output = AsyncLiveOutput(session, capacity=8)
        publisher = output.declare_publisher('topic')
        publisher.put(b'core', encoding='application/json')
        publisher.put_json({'core': 1})
        output.put_json('direct', {'value': 1})
        publisher.undeclare()
        output.close()

        self.assertEqual(session.publishers[0].values, [
            (b'core', 'application/json'), (b'{"core":1}', 'application/json')])
        self.assertEqual(session.values, [('direct', b'{"value":1}', 'application/json')])
        self.assertTrue(session.publishers[0].undeclared)

        json_threads = []
        original_dumps = live_output.json.dumps

        def tracked_dumps(*args, **kwargs):
            json_threads.append(get_ident())
            return original_dumps(*args, **kwargs)

        with patch.object(live_output.json, 'dumps', side_effect=tracked_dumps):
            session = Session()
            output = AsyncLiveOutput(session, capacity=8)
            output.put_json('direct', {'value': 2})
            output.close()
        self.assertEqual(len(json_threads), 1)
        self.assertNotEqual(json_threads[0], get_ident())

    def test_internal_publishers_can_defer_json_encoding(self):
        from tianji_teleop.coordination.arm_command_coordinator import ArmCommandCoordinator
        from tianji_teleop.executors.mujoco.node import _put

        class Publisher:
            def __init__(self):
                self.json_values = []
                self.raw_values = []

            def put_json(self, value):
                self.json_values.append(value)

            def put(self, value, encoding=None):
                self.raw_values.append((value, encoding))

        publisher = Publisher()
        _put(publisher, {'node': 1})
        coordinator = object.__new__(ArmCommandCoordinator)
        coordinator._publishers = {'state': publisher}
        coordinator._publish('state', {'coordinator': 1})

        self.assertEqual(publisher.json_values, [{'node': 1}, {'coordinator': 1}])
        self.assertEqual(publisher.raw_values, [])

    def test_live_capture_can_record_a_snapshot_without_touching_core(self):
        from tianji_teleop.recording.live_capture import LiveCapture, LiveCycleSnapshot

        recorded = []

        class Recorder:
            def append(self, method, *args, **kwargs):
                recorded.append((method, args, kwargs))

        class Command:
            def __init__(self, side):
                self.side = side

            def to_dict(self):
                return {'side': self.side}

        class Status:
            def to_dict(self):
                return {'ready': True}

        snapshot = LiveCycleSnapshot(
            result=SimpleNamespace(
                commands={'left': Command('left'), 'right': Command('right')},
                native_result={'tick_id': 1},
                native_attempt=None,
                receipt_accepted=True,
            ),
            hand_telemetry={},
            producer_status=Status(),
            executor_status=Status(),
            arm_state='arm-state',
            session_state='session-state',
            hand_states={},
            execution_epoch=1,
            coordinator_receipt=None,
            bilateral_command=None,
            hand_commands={},
            source_status=Status(),
            received_timestamp_ns=123,
        )

        LiveCapture(Recorder(), run_id='run').cycle_snapshot(snapshot)

        self.assertEqual([row[0] for row in recorded], [
            'append_arm_command', 'append_arm_command', 'append_arm_state',
            'append_session_state', 'append_dual_audit',
        ])
        self.assertEqual(recorded[-1][1][0], 'native_cycle')

    def test_live_capture_uses_owned_enqueue_for_completed_control_snapshots(self):
        from tianji_teleop.recording.live_capture import LiveCapture, LiveCycleSnapshot

        recorded = []

        class Recorder:
            def append_owned(self, method, *args, **kwargs):
                recorded.append(method)

        class Command:
            def to_dict(self):
                return {'side': 'left'}

        class Status:
            def to_dict(self):
                return {'ready': True}

        snapshot = LiveCycleSnapshot(
            result=SimpleNamespace(
                commands={'left': Command()}, native_result={'tick_id': 1},
                native_attempt=None, receipt_accepted=True,
            ),
            source_status=Status(), producer_status=Status(), executor_status=Status(),
            arm_state='arm', session_state='session', hand_telemetry={}, hand_states={},
            hand_commands={}, execution_epoch=1, coordinator_receipt=None,
            bilateral_command=None, received_timestamp_ns=123,
        )

        LiveCapture(Recorder(), run_id='run').cycle_snapshot(snapshot)

        self.assertEqual(recorded, [
            'append_arm_command', 'append_arm_state', 'append_session_state',
            'append_dual_audit',
        ])

    def test_live_capture_uses_one_owned_cycle_write_when_recorder_supports_it(self):
        from tianji_teleop.recording.live_capture import LiveCapture, LiveCycleSnapshot

        recorded = []

        class Recorder:
            def append_live_cycle_snapshot(self, snapshot, *, run_id):
                recorded.append((snapshot, run_id))

        class Command:
            def to_dict(self):
                return {'side': 'left'}

        snapshot = LiveCycleSnapshot(
            result=SimpleNamespace(
                commands={'left': Command()}, native_result={'tick_id': 1},
                native_attempt=None, receipt_accepted=True,
            ),
            source_status=SimpleNamespace(to_dict=lambda: {'ready': True}),
            producer_status=SimpleNamespace(to_dict=lambda: {'ready': True}),
            executor_status=SimpleNamespace(to_dict=lambda: {'ready': True}),
            arm_state='arm', session_state='session', hand_telemetry={},
            hand_states={}, hand_commands={}, execution_epoch=1,
            coordinator_receipt=None, bilateral_command=None,
            received_timestamp_ns=123,
        )

        LiveCapture(Recorder(), run_id='run').cycle_snapshot(snapshot)

        self.assertEqual(recorded, [(snapshot, 'run')])

    def test_slow_side_effect_does_not_block_fixed_rate_control_loop(self):
        from tianji_teleop.producers.spark.live_control import (
            FixedRateControlLoop,
            SnapshotSideEffectLoop,
        )

        stop = Event()
        produced = []
        side_effect_started = Event()

        def slow_consumer(_item):
            side_effect_started.set()
            time.sleep(0.020)

        side_effects = SnapshotSideEffectLoop(slow_consumer, capacity=128)
        counter = [0]

        def tick():
            counter[0] += 1
            return counter[0]

        loop = FixedRateControlLoop(
            period_s=0.005,
            tick=tick,
            stop_event=stop,
            tick_sink=lambda item: (produced.append(item), side_effects.submit(item)),
        )
        side_effects.start()
        loop.start()
        self.assertTrue(side_effect_started.wait(1.0))
        time.sleep(0.080)
        stop.set()
        loop.join(1.0)
        side_effects.close()

        self.assertFalse(loop.is_alive())
        self.assertGreaterEqual(loop.tick_count, 10)
        self.assertEqual(loop.tick_count, len(produced))
        self.assertIsNone(loop.failure)
        self.assertIsNone(side_effects.failure)

    def test_fixed_rate_control_keeps_absolute_deadline_after_late_tick(self):
        from tianji_teleop.producers.spark.live_control import FixedRateControlLoop

        class TestEvent(Event):
            def __init__(self):
                super().__init__()
                self.waits = []

            def wait(self, timeout=None):
                self.waits.append(timeout)
                self.set()
                return True

        class Clock:
            def __init__(self):
                self.value = 0.0

            def __call__(self):
                return self.value

        stop = TestEvent()
        clock = Clock()
        ticks = []

        def tick():
            ticks.append(len(ticks) + 1)
            clock.value = 0.012 if len(ticks) == 1 else 0.013
            if len(ticks) >= 3:
                stop.set()

        loop = FixedRateControlLoop(period_s=0.005, tick=tick,
                                    stop_event=stop, clock=clock)
        loop.start()
        loop.join(1.0)

        self.assertFalse(loop.is_alive())
        self.assertEqual(ticks, [1, 2, 3])
        self.assertEqual(stop.waits, [0.002])
        self.assertEqual(loop.late_cycles, 2)

    def test_side_effect_queue_overflow_is_reported_without_blocking_submit(self):
        from tianji_teleop.producers.spark.live_control import SnapshotSideEffectLoop

        release = Event()
        started = Event()

        def blocked_consumer(_item):
            started.set()
            release.wait(1.0)

        side_effects = SnapshotSideEffectLoop(blocked_consumer, capacity=1)
        side_effects.start()
        self.assertTrue(side_effects.submit(1))
        self.assertTrue(started.wait(1.0))
        self.assertTrue(side_effects.submit(2))
        started_at = time.monotonic()
        self.assertFalse(side_effects.submit(3))
        self.assertLess(time.monotonic() - started_at, 0.1)
        self.assertIn('overflow', side_effects.failure)
        release.set()
        side_effects.close()


if __name__ == '__main__':
    unittest.main()
