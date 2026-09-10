"""Managed simulation runner. No real executor or automatic authorization."""
from contextlib import ExitStack, nullcontext
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
from .live_simulation import SparkLiveSimulation


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
        single_hand_side='left' if sides == ('left',) else 'right')


def run_live(root, args, resolved):
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
        def publish(topic, value):
            session.put(topic, json.dumps(value, allow_nan=False, separators=(',', ':')).encode(),
                        encoding='application/json')
        hand_client = None
        slot = _InputSlot()
        if not args.disable_hands:
            hand_client = _make_hand_client(root, resolved['config']['active_hand_sides'])
            stack.callback(hand_client.close)
        core = SparkLiveSimulation(root, run_id=run_id, router_zid=router, instance_id=instance,
            session=session, hand_sides=tuple(resolved['config']['active_hand_sides']),
            hand_source=slot if hand_client else None, hand_backend=hand_client,
            hand_command_sink=capture.hand_output if capture else None)
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
                raw_line_sink=capture.rawviz if capture else None,
                callback_sink=capture.callback if capture else None)
            stack.callback(slot.source.close)
        def raw_frame(frame):
            if overlay:
                overlay.ingest_raw(frame)
            if capture:
                capture.tjvr(frame)
            publish(RAW_REFERENCE_TJVR, ReferenceTjvrRaw(frame, router).to_dict())
        receiver = ReferenceTjvrUdp(receiver_instance_id=core.source_instance_id,
            host=args.tjvr_bind, port=args.tjvr_port, raw_frame_sink=raw_frame,
            max_position_jump_m=resolved['tjvr_stream_contract']['max_position_jump_m'],
            max_orientation_jump_rad=resolved['tjvr_stream_contract']['max_orientation_jump_rad'],
            decision_sink=capture.tjvr_decision if capture else None)
        stack.callback(receiver.close)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(sig, lambda *_: stop.set())
            stack.callback(signal.signal, sig, previous)
        if sys.stdin.isatty():
            saved = termios.tcgetattr(sys.stdin.fileno())
            stack.callback(termios.tcsetattr, sys.stdin.fileno(), termios.TCSANOW, saved)
            tty.setcbreak(sys.stdin.fileno())
        viewer = None
        if args.viewer:
            import mujoco.viewer
            viewer = stack.enter_context(mujoco.viewer.launch_passive(core.sim.model, core.sim.data,
                                                                      key_callback=key_callback))
        print(json.dumps(dict(kind='dual_live_started', run_id=run_id, router_zid=router,
            tjvr_bind=list(receiver.address), resolved=resolved, authorities=core.authorities)), flush=True)
        print('s: start (requires fresh input); h: return Home; r: rearm at Home (then new input + s); '
              'q: return and exit. Fault requires restart.', flush=True)
        started = time.monotonic()
        deadline = started
        period = 1. / resolved['config']['rate_hz']
        late_cycles = 0
        exit_after_home = False
        result = None
        while not stop.is_set():
            if viewer is not None and not viewer.is_running():
                break
            if args.duration_s is not None and time.monotonic() - started >= args.duration_s:
                break
            if select.select([sys.stdin], [], [], 0)[0]:
                data = os.read(sys.stdin.fileno(), 128)
                for value in data:
                    key_callback(value)
            while not keys.empty():
                key = keys.get_nowait()
                if key == 'r':
                    try:
                        with viewer.lock() if viewer is not None else nullcontext():
                            ack = core.rearm_at_home()
                    except ValueError as exc:
                        report = dict(kind='operator_result', action='rearm', accepted=False, reason=str(exc))
                    else:
                        report = dict(kind='operator_result', action='rearm', accepted=True,
                                      execution_epoch=ack['execution_epoch'],
                                      reset_ack=ack,
                                      reason='fresh input and explicit start required')
                    if capture:
                        capture.audit('operator_result', report)
                    print(json.dumps(report), flush=True)
                    continue
                action = 'start' if key == 's' else 'shutdown' if key == 'q' else 'return'
                outcome = core.request(action)
                report = dict(kind='operator_result', action=action, accepted=outcome.accepted, reason=outcome.reason)
                if capture:
                    capture.audit('operator_result', report)
                print(json.dumps(report), flush=True)
                if key == 'q':
                    exit_after_home = True
            with viewer.lock() if viewer is not None else nullcontext():
                result = core.step(receiver.try_read_latest(), source_failure=guard.failure or receiver.failure
                                   or (recorder.failure if recorder else None))
            if capture:
                capture.cycle(core, result)
            publish(topics.SOURCE_STATUS, core.source_status.to_dict())
            publish(topics.PRODUCER_STATUS, core.producer.status(time.monotonic_ns()).to_dict())
            publish(topics.ARM_STATE, core.sim.arm_state.to_dict())
            publish(topics.EXECUTOR_STATUS, core.sim.status.to_dict())
            hand_telemetry = core.hand_telemetry
            if hand_telemetry:
                publish(topics.PRODUCER_STATUS, hand_telemetry['producer'])
                for side, status in hand_telemetry['executors'].items():
                    publish(topics.hand_executor_status(side), status)
            for side, command in core.last_hand_commands.items():
                publish(topics.hand_command(side), command.to_dict())
            for side in core.hand_sides:
                publish(topics.hand_state(side), core.sim.hand_state(side).to_dict())
            if viewer is not None:
                if overlay:
                    import mujoco
                    if result.native_result is not None:
                        overlay.ingest_native(result.native_result,
                                              execution_epoch=core.producer.guard.execution_epoch)
                    with viewer.lock():
                        viewer.user_scn.ngeom = 0
                        overlay.append(viewer.user_scn, mujoco, time.monotonic_ns())
                viewer.sync()
            if guard.failure or core.failure or core.coordinator.state.state == 'fault' or (exit_after_home and core.coordinator.state.state == 'idle'):
                break
            deadline += period
            remaining = deadline - time.monotonic()
            if remaining < 0:
                late_cycles += 1
                deadline = time.monotonic()
            else:
                stop.wait(remaining)
        report = dict(kind='dual_live_complete', state=core.coordinator.state.state,
            reason=guard.failure or core.failure or core.coordinator.state.reason, native_ticks=(core.producer.last_result or {}).get('tick_id', 0),
            late_cycles=late_cycles, real_time_qualified=False)
        exit_code = 1 if guard.failure or core.failure or core.coordinator.state.state == 'fault' else 0
        if capture:
            # Stop writers at their source before draining disk work. Close
            # methods are idempotent when ExitStack later releases ownership.
            receiver.close()
            if slot.source is not None:
                slot.source.close()
            core.close()
            capture.audit('lifecycle', report)
            recorder.close(complete=exit_code == 0)
        print(json.dumps(report), flush=True)
        return exit_code
