"""Managed simulation runner. No real executor or automatic authorization."""
from contextlib import ExitStack
import json
import os
import queue
import select
import signal
import sys
import termios
from threading import Event
import time
import tty

from ...hand_tracking.reference_tjvr_udp import ReferenceTjvrUdp
from ...hand_tracking.reference_manus_process import ReferenceManusProcess
from ...coordination.live_domain_guard import LiveDomainGuard
from ...protocol import topics
from ...protocol.reference_inputs import RAW_REFERENCE_TJVR, ReferenceTjvrRaw
from ...zenoh_util import open_session, require_single_router, declare_component_liveliness
from ..hand_retarget import OfficialHandClient
from .live_control import LiveOperatorEvent, SnapshotSideEffectLoop, SparkControlLoop
from .live_output import AsyncLiveOutput
from .live_simulation import SparkLiveSimulation


_LIVE_THREAD_SWITCH_INTERVAL_S = 0.001


def _close_live_recording(recorder, *, exit_code):
    """Persist the final process result before ExitStack cleanup runs."""
    if recorder is not None:
        recorder.close(complete=exit_code == 0)


def _audit_live_capture(capture, kind, payload):
    """Do not append a final audit row to an already failed recording."""
    if capture is None:
        return False
    recorder = getattr(capture, 'recorder', None)
    if recorder is not None and getattr(recorder, 'failure', None):
        return False
    capture.audit(kind, payload)
    return True


def _bound_live_thread_switch_interval():
    """Bound GIL handoff latency while the live control owner is active."""
    previous = sys.getswitchinterval()
    if previous > _LIVE_THREAD_SWITCH_INTERVAL_S:
        sys.setswitchinterval(_LIVE_THREAD_SWITCH_INTERVAL_S)
    return previous


class _InputSlot:
    """Attach acquisition only after all model/retarget workers are loaded."""
    source = None

    @property
    def failure(self):
        return self.source.failure if self.source is not None else None

    def try_read(self):
        return self.source.try_read() if self.source is not None else None


def decode_key(value):
    """GLFW supplies uppercase key codes; terminal input supplies bytes."""
    if type(value) is not int or not 0 <= value < 128:
        return None
    key = chr(value).lower()
    return key if key in ('s', 'h', 'r', 'q') else None


def _make_hand_client(root, sides, *, client_factory=None):
    sides = tuple(sides)
    if sides not in (('left',), ('right',), ('left', 'right'), ('right', 'left')):
        raise ValueError('explicit enabled hand sides required for the hand worker')
    factory = OfficialHandClient if client_factory is None else client_factory
    return factory(python=root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
        script=root / 'scripts/wuji_hand_worker.py', startup_handshake=True,
        filter_continuity_ns=200_000_000,
        single_hand_side='left' if sides == ('left',) else 'right')


def run_live(root, args, resolved):
    resolved = dict(resolved, manus_filter_continuity_ns=200_000_000)
    required = ('TIANJI_RUN_ID', 'TIANJI_DUAL_INSTANCE_ID', 'TIANJI_ROUTER_ZID')
    if os.environ.get('TIANJI_DUAL_MANAGED') != '1' or any(not os.environ.get(k) for k in required):
        raise RuntimeError('use the managed run_session launcher; explicit session identities required')
    run_id, instance, expected_router = (os.environ[k] for k in required)
    stop = Event()
    keys = queue.Queue(maxsize=64)
    def key_callback(value):
        key = decode_key(value)
        if key is not None:
            try:
                keys.put_nowait(key)
            except queue.Full:
                stop.set()
    with ExitStack() as stack:
        previous_switch_interval = _bound_live_thread_switch_interval()
        if previous_switch_interval > _LIVE_THREAD_SWITCH_INTERVAL_S:
            stack.callback(sys.setswitchinterval, previous_switch_interval)
        session = open_session()
        stack.callback(session.close)
        router = require_single_router(session, expected_router)
        recorder = capture = None
        if args.record is not None:
            from ...recording.async_dual import AsyncDualRecorder
            from ...recording.live_capture import LiveCapture
            recorder = stack.enter_context(AsyncDualRecorder(args.record, router_zid=router,
                metadata=dict(run_id=run_id, resolved_configuration=resolved,
                              real_time_qualified=False, input_clock='host_monotonic_ns')))
            capture = LiveCapture(recorder, run_id=run_id)
            capture.audit('lifecycle', dict(stage='opening'))

        output = AsyncLiveOutput(session)
        stack.callback(output.close)

        hand_client = None
        slot = _InputSlot()
        if not args.disable_hands:
            hand_client = _make_hand_client(root, resolved['config']['active_hand_sides'])
            stack.callback(hand_client.close)
        core = SparkLiveSimulation(root, run_id=run_id, router_zid=router, instance_id=instance,
            session=output.session_proxy(), hand_sides=tuple(resolved['config']['active_hand_sides']),
            hand_source=slot if hand_client else None, hand_backend=hand_client,
            hand_command_sink=capture.hand_output if capture else None,
            hand_expired_input_sink=(lambda row: capture.audit(
                'manus_superseded_input' if row['reason'] == 'superseded_before_retarget'
                else 'manus_expired_input', row)) if capture else None)
        stack.callback(core.close)
        overlay = None
        if args.spark_overlay:
            from ...executors.mujoco.spark_overlay import SparkOverlay
            overlay = SparkOverlay(core.source_instance_id)
        if capture:
            capture.audit('lifecycle', dict(stage='core_ready', authorities=core.authorities,
                manus_receiver_instance_id=instance + '-manus' if hand_client else None,
                upstream_pico_calibration='external_not_verified'))

        roles = [('source', 'source'), ('producer_arm', 'producer/arm'), ('executor_arm', 'executor/arm')]
        expected_tokens = {f'tj/live/coordinator/arm/arm/{core.coordinator.publisher_instance_id}'}
        if hand_client:
            roles.append(('producer_hand', 'producer/hand'))
        for role, token_role in roles:
            identity = core.authorities[role]
            token = declare_component_liveliness(session, role=token_role,
                logical_id=identity['logical_id'], instance_id=identity['publisher_instance_id'])
            stack.callback(token.undeclare)
            expected_tokens.add(f"tj/live/{token_role}/{identity['logical_id']}/{identity['publisher_instance_id']}")
        for side in core.hand_sides:
            identity = core.authorities['executor_hand'][side]
            token = declare_component_liveliness(session, role='executor/hand',
                logical_id=identity['logical_id'], instance_id=identity['publisher_instance_id'])
            stack.callback(token.undeclare)
            expected_tokens.add(f"tj/live/executor/hand/{identity['logical_id']}/{identity['publisher_instance_id']}")
        guard = LiveDomainGuard(expected_tokens)
        import zenoh
        live_subscriber = session.liveliness().declare_subscriber('tj/live/**',
            lambda sample: guard.observe(str(sample.key_expr), present=sample.kind == zenoh.SampleKind.PUT),
            history=True)
        stack.callback(live_subscriber.undeclare)
        # Initial synchronous inventory is BEFORE acquisition/control begins;
        # subsequent supervision is callback-based and never blocks IK.
        for reply in session.liveliness().get('tj/live/**', timeout=.5):
            if not reply.ok:
                raise RuntimeError('live authority inventory failed')
            guard.observe(str(reply.result.key_expr), present=True)
        if guard.failure:
            raise RuntimeError(guard.failure)

        if hand_client:
            from ...hand_tracking.manus_environment import manus_environment
            manus_env, _ = manus_environment(args.manus_rawviz, library_dir=args.manus_library_dir)
            slot.source = ReferenceManusProcess(command=[str(args.manus_rawviz), '--user', args.manus_user],
                env=manus_env,
                receiver_instance_id=instance + '-manus', sides=core.hand_sides,
                right_glove=args.right_glove, left_glove=args.left_glove,
                cwd=os.path.dirname(os.path.abspath(os.fspath(args.manus_rawviz))),
                raw_line_sink=capture.rawviz if capture else None,
                callback_sink=capture.callback if capture else None)
            stack.callback(slot.source.close)

        def raw_frame(frame):
            if overlay:
                overlay.ingest_raw(frame)
            if capture:
                capture.tjvr(frame)
            output.put_json(RAW_REFERENCE_TJVR, ReferenceTjvrRaw(frame, router).to_dict())

        receiver = ReferenceTjvrUdp(receiver_instance_id=core.source_instance_id,
            host=args.tjvr_bind, port=args.tjvr_port, raw_frame_sink=raw_frame,
            max_position_jump_m=resolved['tjvr_stream_contract']['max_position_jump_m'],
            max_orientation_jump_rad=resolved['tjvr_stream_contract']['max_orientation_jump_rad'],
            decision_sink=capture.tjvr_decision if capture else None)
        stack.callback(receiver.close)

        def publish_snapshot(snapshot):
            if isinstance(snapshot, LiveOperatorEvent):
                if capture:
                    capture.audit('operator_result', snapshot.report)
                return
            if overlay and snapshot.result.native_result is not None:
                overlay.ingest_native(snapshot.result.native_result,
                                      execution_epoch=snapshot.execution_epoch)
            if capture:
                capture.cycle_snapshot(snapshot)
            output.put_json(topics.SOURCE_STATUS, snapshot.source_status.to_dict())
            output.put_json(topics.PRODUCER_STATUS, snapshot.producer_status.to_dict())
            output.put_json(topics.ARM_STATE, snapshot.arm_state.to_dict())
            output.put_json(topics.EXECUTOR_STATUS, snapshot.executor_status.to_dict())
            if snapshot.hand_telemetry:
                output.put_json(topics.PRODUCER_STATUS, snapshot.hand_telemetry['producer'])
                for side, status in snapshot.hand_telemetry['executors'].items():
                    output.put_json(topics.hand_executor_status(side), status)
            for side, command in snapshot.hand_commands.items():
                output.put_json(topics.hand_command(side), command.to_dict())
            for side, state in snapshot.hand_states.items():
                output.put_json(topics.hand_state(side), state.to_dict())

        side_effects = SnapshotSideEffectLoop(publish_snapshot, capacity=1024)
        side_effects.start()
        stack.callback(side_effects.close)

        def live_failure():
            return (guard.failure or receiver.failure or side_effects.failure or output.failure or
                    (recorder.failure if recorder else None))

        for sig in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(sig, lambda *_: stop.set())
            stack.callback(signal.signal, sig, previous)
        if sys.stdin.isatty():
            saved = termios.tcgetattr(sys.stdin.fileno())
            stack.callback(termios.tcsetattr, sys.stdin.fileno(), termios.TCSANOW, saved)
            tty.setcbreak(sys.stdin.fileno())

        viewer = None
        render_data = None
        if args.viewer:
            import mujoco
            import mujoco.viewer
            render_data = mujoco.MjData(core.sim.model)
            render_data.qpos[:] = core.sim.data.qpos
            mujoco.mj_forward(core.sim.model, render_data)
            viewer = stack.enter_context(mujoco.viewer.launch_passive(
                core.sim.model, render_data, key_callback=key_callback))

        control = SparkControlLoop(core, receiver, stop_event=stop,
            period_s=1. / resolved['config']['rate_hz'], failure_fn=live_failure,
            snapshot_sink=side_effects)

        def emit_reports():
            for report in control.poll_reports():
                print(json.dumps(report), flush=True)

        print(json.dumps(dict(kind='dual_live_started', run_id=run_id, router_zid=router,
            tjvr_bind=list(receiver.address), resolved=resolved, authorities=core.authorities)), flush=True)
        print('s: start (requires fresh input); h: return Home; r: rearm at Home (then new input + s); '
              'q: return and exit. Fault requires restart.', flush=True)
        started = time.monotonic()
        control.start()
        try:
            while not stop.is_set():
                if viewer is not None and not viewer.is_running():
                    stop.set()
                    break
                if args.duration_s is not None and time.monotonic() - started >= args.duration_s:
                    stop.set()
                    break
                if select.select([sys.stdin], [], [], 0)[0]:
                    data = os.read(sys.stdin.fileno(), 128)
                    for value in data:
                        key_callback(value)
                while not keys.empty():
                    key = keys.get_nowait()
                    action = 'rearm' if key == 'r' else 'start' if key == 's' else 'shutdown' if key == 'q' else 'return'
                    control.submit(action)
                emit_reports()
                if viewer is not None:
                    import mujoco
                    snapshot = control.latest
                    with viewer.lock():
                        if snapshot is not None:
                            core.sim.copy_joint_state_to(
                                render_data, snapshot.arm_state.position_rad,
                                {side: state.position_rad for side, state in snapshot.hand_states.items()})
                        viewer.user_scn.ngeom = 0
                        if overlay:
                            overlay.append(viewer.user_scn, mujoco, time.monotonic_ns())
                    viewer.sync()
                stop.wait(.005)
        finally:
            stop.set()
            control.join(timeout=5)
            if control.is_alive():
                raise RuntimeError('Spark control loop did not stop')

        emit_reports()
        receiver.close()
        if slot.source is not None:
            slot.source.close()
        side_effects.close()
        failure = (control.failure or guard.failure or receiver.failure or side_effects.failure or
                   output.failure or (recorder.failure if recorder else None) or core.failure)
        state = control.state
        reason = failure or control.reason
        report = dict(kind='dual_live_complete', state=state, reason=reason,
            native_ticks=control.native_ticks, late_cycles=control.late_cycles,
            control_ticks=control.tick_count, real_time_qualified=False)
        exit_code = 1 if failure or state == 'fault' else 0
        try:
            _audit_live_capture(capture, 'lifecycle', report)
        except Exception as exc:
            failure = failure or f'recording final audit failed: {type(exc).__name__}: {exc}'
        exit_code = 1 if failure or state == 'fault' else 0
        try:
            _close_live_recording(recorder, exit_code=exit_code)
        except Exception as exc:
            failure = failure or str(exc)
            exit_code = 1
        report['reason'] = failure or control.reason
        if recorder is not None:
            report['recording'] = recorder.statistics
        print(json.dumps(report), flush=True)
        return exit_code
