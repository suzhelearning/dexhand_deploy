import queue
import threading
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from tianji_teleop.producers.spark import native_live_runner as runner
from tianji_teleop.producers.spark.live_output import AsyncLiveOutput


class NativeShutdownTest(unittest.TestCase):
    def test_immediate_reply_is_registered_before_send(self):
        consumer=self.consumer(None)
        def send(action,*,next_epoch,command_id):
            for reason in ('sampling','accepted'):
                consumer._consume(SimpleNamespace(kind='reply',timestamp_ns=1,payload={
                    'id':command_id,'accepted':True,'reason':reason}))
            return command_id
        gateway=SimpleNamespace(reserve_command_id=lambda:12,send_action=send)
        self.assertEqual(consumer.send_action(gateway,'calibrate'),12)
        self.assertEqual([r['action'] for r in consumer.poll_reports()],['calibrate','calibrate'])
        self.assertFalse(consumer._actions)
        self.assertFalse(consumer._pending_calibrations)

    def test_send_failure_cleans_registered_action(self):
        consumer=self.consumer(None)
        def send(*args,**kwargs):
            self.assertEqual(consumer._actions,{12:'calibrate'})
            raise OSError('injected send failure')
        gateway=SimpleNamespace(reserve_command_id=lambda:12,send_action=send)
        with self.assertRaisesRegex(OSError,'injected'):
            consumer.send_action(gateway,'calibrate')
        self.assertFalse(consumer._actions)
        self.assertFalse(consumer._pending_calibrations)

    def test_calibration_keeps_terminal_action_until_final_reply(self):
        for success in (True,False):
            consumer=self.consumer(None)
            consumer.map_action(7,'calibrate')
            for accepted,reason in ((True,'hold both arms horizontal and steady for 2 seconds'),
                                    (success,'accepted' if success else 'wrist moved too much')):
                consumer._consume(SimpleNamespace(kind='reply',timestamp_ns=1,payload={
                    'id':7,'accepted':accepted,'reason':reason}))
            self.assertEqual([r['action'] for r in consumer.poll_reports()],['calibrate','calibrate'])
            self.assertNotIn(7,consumer._actions)
            self.assertNotIn(7,consumer._pending_calibrations)

    def test_rejected_calibration_is_not_left_pending(self):
        consumer=self.consumer(None)
        consumer.map_action(8,'calibrate')
        consumer._consume(SimpleNamespace(kind='reply',timestamp_ns=1,payload={
            'id':8,'accepted':False,'reason':'height calibration requires healthy exact Home'}))
        self.assertEqual(consumer.poll_reports()[0]['action'],'calibrate')
        self.assertFalse(consumer._actions)
        self.assertFalse(consumer._pending_calibrations)

    def test_internal_rearm_report_is_not_unknown_operator(self):
        consumer = self.consumer(None)
        consumer._consume(SimpleNamespace(kind='reply', timestamp_ns=1, payload={
            'id': 0, 'accepted': True, 'reason': 'accepted'}))
        self.assertEqual(consumer.poll_reports()[0]['action'], 'auto_rearm')

    def consumer(self, gateway):
        return runner._NativeFrameConsumer(
            gateway, manifest={}, prefix='spark', output=None, capture=None,
            overlay=None, mapped_overlay=None, receiver_instance_id='test')

    def test_drain_consumes_completion_after_queued_work(self):
        frames = queue.Queue()
        entered, release = threading.Event(), threading.Event()
        frames.put(SimpleNamespace(kind='work'))
        frames.put(SimpleNamespace(kind='complete', payload={
            'complete': True, 'state': 'idle', 'epoch': 2, 'reason': 'return complete'}))
        gateway = SimpleNamespace(get_frame=lambda timeout: frames.get(timeout=timeout),
                                  failure=None, process=None)
        consumer = self.consumer(gateway)
        original = consumer._consume
        def consume(frame):
            if frame.kind == 'work':
                entered.set()
                release.wait(2)
            else:
                original(frame)
        consumer._consume = consume
        consumer.start()
        self.assertTrue(entered.wait(2))
        joining = threading.Event()
        original_join = consumer._thread.join
        def join(*args, **kwargs):
            joining.set()
            return original_join(*args, **kwargs)
        consumer._thread.join = join
        closer = threading.Thread(target=consumer.close)
        closer.start()
        self.assertTrue(joining.wait(2))
        release.set()
        closer.join(3)
        self.assertFalse(closer.is_alive())
        self.assertTrue(consumer.complete)
        self.assertTrue(frames.empty())
        self.assertIsNone(consumer.failure)

    def test_runtime_checks_all_supervisors(self):
        for index in range(4):
            supervisors = [SimpleNamespace(failure=None) for _ in range(4)]
            supervisors[index].failure = 'injected authority/transport failure'
            with self.assertRaisesRegex(RuntimeError, 'injected'):
                runner._check_native_health(*supervisors)

    def test_completion_uses_native_ack_not_unrelated_error(self):
        consumer = self.consumer(None)
        consumer._consume(SimpleNamespace(kind='complete', payload={
            'complete': True, 'state': 'idle', 'epoch': 2, 'reason': 'return complete'}))
        self.assertTrue(consumer.home_return_completed)
        self.assertEqual(consumer.control_status, ('idle', 2))
        consumer._set_failure('late publication failure')
        self.assertTrue(consumer.home_return_completed)

    def test_incomplete_ack_does_not_confirm_home(self):
        consumer = self.consumer(None)
        consumer._consume(SimpleNamespace(kind='complete', payload={
            'complete': False, 'state': 'fault', 'epoch': 2, 'reason': 'fault'}))
        self.assertFalse(consumer.home_return_completed)

    def test_finalization_checks_publish_failure_after_drain(self):
        entered, release = threading.Event(), threading.Event()
        class Transport:
            def put(self, *args, **kwargs):
                entered.set()
                release.wait(2)
                raise RuntimeError('tail publication failed')
        output = AsyncLiveOutput(Transport())
        output.put('test/topic', b'data')
        self.assertTrue(entered.wait(2))
        consumer = SimpleNamespace(close=lambda: release.set(), failure=None,
                                   home_return_completed=True)
        process = SimpleNamespace(wait=lambda timeout: 0)
        try:
            failure = runner._finish_native_outputs(
                consumer, SimpleNamespace(process=process, failure=None),
                SimpleNamespace(failure=None), output)
            self.assertIn('tail publication failed', failure)
        finally:
            release.set()
            output.close()

    def test_nonzero_child_exit_is_not_success(self):
        failure = runner._finish_native_outputs(
            SimpleNamespace(close=lambda: None, failure=None, home_return_completed=True),
            SimpleNamespace(process=SimpleNamespace(wait=lambda timeout: 1), failure=None),
            SimpleNamespace(failure=None), SimpleNamespace(close=lambda: None, failure=None))
        self.assertIn('exit', failure)

    def test_live_runner_aborts_gateway_when_authority_is_lost(self):
        # Replace only external resources; execute the real live supervision loop.
        gateway = Mock(failure=None)
        consumer = Mock(failure=None, complete=False)
        guard = SimpleNamespace(failure=None)
        consumer.start.side_effect = lambda: setattr(guard, 'failure', 'authority lost in teleop')
        args = SimpleNamespace(record=None, spark_overlay=False, viewer=False,
                               tjvr_bind='127.0.0.1', tjvr_port=15000, duration_s=None)
        resolved = {'config': {'hands_enabled': False, 'rate_hz': 200,
                    'ik_backend': 'spark_upper_qpoases_headroom_feedforward_velocity_qp'}}
        with ExitStack() as stack:
            stack.enter_context(patch.dict('os.environ', {
                'TIANJI_DUAL_MANAGED': '1', 'TIANJI_RUN_ID': 'test',
                'TIANJI_DUAL_INSTANCE_ID': 'test', 'TIANJI_ROUTER_ZID': 'test'}))
            for name, value in {
                'NativeGatewayProcess': Mock(return_value=gateway),
                '_NativeFrameConsumer': Mock(return_value=consumer),
                'LiveDomainGuard': Mock(return_value=guard),
                '_declare_native_authorities': Mock(return_value={'test'}),
                '_wait_for_guard': Mock(return_value=Mock()),
                'build_native_gateway_manifest': Mock(return_value={'source_authority': {'instance': 'test'}}),
                'write_native_gateway_manifest': Mock(return_value=Mock()),
                'AsyncLiveOutput': Mock(return_value=Mock(failure=None)),
            }.items():
                stack.enter_context(patch.object(runner, name, value))
            stack.enter_context(patch('tianji_teleop.zenoh_util.open_session', return_value=Mock()))
            stack.enter_context(patch('tianji_teleop.zenoh_util.require_single_router', return_value='test'))
            stack.enter_context(patch.object(runner.os, 'isatty', return_value=False))
            stack.enter_context(patch('builtins.print'))
            with self.assertRaisesRegex(RuntimeError, 'authority lost in teleop'):
                runner.run_live_native(Path('.'), args, resolved)
        gateway.close.assert_called_with(graceful=False)
        consumer.close.assert_called_with(drain=False)

    def test_native_joint_runner_transfers_pipe_and_cleans_up_on_driver_failure(self):
        gateway=Mock(failure=None)
        consumer=Mock(failure=None,complete=False)
        driver=Mock(failure='rawviz exited (7)')
        driver.fileno.return_value=15
        args=SimpleNamespace(record='fixture.h5',spark_overlay=False,viewer=True,
            viewer_backend='cpp',publication_backend='cpp',recording_adapter='cpp',
            tjvr_bind='127.0.0.1',tjvr_port=15000,duration_s=None,manus_rawviz='/fixture/rawviz',
            manus_user='gjy',manus_library_dir=None,right_glove=None,left_glove=None)
        resolved={'config':dict(hands_enabled=True,active_hand_sides=['left','right'],hand_input='manus',
            rate_hz=200,ik_backend='spark_upper_qpoases_headroom_feedforward_velocity_qp')}
        with ExitStack() as stack:
            stack.enter_context(patch.dict('os.environ',{'TIANJI_DUAL_MANAGED':'1','TIANJI_RUN_ID':'test',
                'TIANJI_DUAL_INSTANCE_ID':'test','TIANJI_ROUTER_ZID':'test','TIANJI_ROUTER_ENDPOINT':'tcp/127.0.0.1:7447'}))
            factory=Mock(return_value=gateway)
            for name,value in dict(NativeGatewayProcess=factory,_NativeFrameConsumer=Mock(return_value=consumer),
                LiveDomainGuard=Mock(return_value=SimpleNamespace(failure=None)),
                _declare_native_authorities=Mock(return_value={'test'}),_wait_for_guard=Mock(return_value=Mock()),
                build_native_gateway_manifest=Mock(return_value={'source_authority':{'instance':'test'}}),
                write_native_gateway_manifest=Mock(return_value=Mock()),AsyncLiveOutput=Mock(return_value=Mock(failure=None))).items():
                stack.enter_context(patch.object(runner,name,value))
            stack.enter_context(patch('tianji_teleop.hand_tracking.native_manus_process.NativeManusProcess',return_value=driver))
            stack.enter_context(patch('tianji_teleop.hand_tracking.manus_environment.manus_environment',return_value=({},{})))
            build=stack.enter_context(patch('tianji_teleop.producers.spark.native_hand_launch.build_hand_manifest',return_value={}))
            stack.enter_context(patch('tianji_teleop.recording.native_owner.NativeRecordingOwner',return_value=Mock()))
            stack.enter_context(patch('tianji_teleop.zenoh_util.open_session',return_value=Mock()))
            stack.enter_context(patch('tianji_teleop.zenoh_util.require_single_router',return_value='test'))
            stack.enter_context(patch.object(runner.os,'isatty',return_value=False))
            stack.enter_context(patch('builtins.print'))
            with self.assertRaisesRegex(RuntimeError,'rawviz exited'):
                runner.run_live_native(Path('.'),args,resolved)
            self.assertEqual(factory.call_args.kwargs['manus_fd'],15)
            self.assertEqual(build.call_args.kwargs['stdout_fd'],15)
        driver.mark_transferred.assert_called_once()
        driver.close.assert_called_once()
        gateway.close.assert_called_with(graceful=False)
