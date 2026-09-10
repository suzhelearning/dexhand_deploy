"""Explicit-authority, single-clock live SPARK/MuJoCo control core.

No automatic start, deterministic budget override, device execution or input
socket. A launcher owns input threads, terminal, router/guard and scheduling.
Unlike the offline harness, idle/return/fault remain ordinary control states.
"""
from types import SimpleNamespace
from collections import deque
from copy import deepcopy
from threading import Lock
import time

import mujoco

from ...coordination.arm_command_coordinator import ArmCommandCoordinator
from ...executors.mujoco.authorized_hand import AuthorizedHandMujoco
from ...hand_tracking.spark_worker_client import SparkWorkerClient
from ...protocol.messages import ComponentStatus, HandExecutorStatus
from ..hand_retarget import HandRetargetProducer
from ..hand_retarget_loop import HandRetargetLoop
from .coordinator_cycle import SparkCoordinatorCycle
from .factory import reference_robot_config
from .node import SparkProducer


class SparkLiveSimulation:
    def __init__(self, root, *, run_id, router_zid, instance_id, clock=time.monotonic_ns,
                 hand_sides=(), hand_source=None, hand_backend=None, session=None, hand_command_sink=None):
        for value in (run_id, router_zid, instance_id):
            if not isinstance(value, str) or not value.strip() or '/' in value:
                raise ValueError('explicit run/router/instance identities required')
        if len(set(hand_sides)) != len(hand_sides) or set(hand_sides) - {'left', 'right'}:
            raise ValueError('hand sides must be distinct left/right')
        if bool(hand_sides) != (hand_source is not None and hand_backend is not None):
            raise ValueError('active hands require both source and official backend')
        if not hand_sides and (hand_source is not None or hand_backend is not None):
            raise ValueError('disabled hands must not start a source or backend')
        self.clock = clock
        self.router_zid = router_zid
        self.source_instance_id = instance_id + '-source'
        self._event = 0
        self._intent = 0
        self._last_ns = 0
        self._input_after_ns = 0
        self._closed = False
        self._failure = None
        self.hand_sides = tuple(hand_sides)
        self.hand_loop = None
        self._hand_producer_status = None
        self._hand_executor_statuses = {}
        self._hand_commands = deque()
        self._last_hand_commands = {}
        self._hand_lock = Lock()
        self._hand_command_sink = hand_command_sink
        assets = root / 'src/tianji_teleop'
        config = assets / 'config/producers/spark_reference.yaml'
        model_path = assets / 'assets/spark/marvin_m6_wuji2.xml'
        urdf = assets / 'assets/spark/marvin_m6_s_ccs_696_v4_local.urdf'
        robot = reference_robot_config(config, urdf, assets / 'config/robot/arm.yaml')
        coordinator_id, producer_id, executor_id = (instance_id + '-' + name for name in ('coord', 'spark', 'sim'))
        def authority(logical, identity, enabled=True):
            return dict(logical_id=logical, publisher_instance_id=identity, router_zid=router_zid, enabled=enabled)
        self.authorities = dict(source=authority('tjvr', self.source_instance_id),
            producer_arm=authority('ik_spark_headroom', producer_id),
            coordinator_arm=authority('arm', coordinator_id), executor_arm=authority('mujoco', executor_id),
            producer_hand=authority('official_wuji_hand2', instance_id + '-hand', bool(hand_sides)),
            executor_hand={side: authority('wuji_' + side, executor_id, side in hand_sides)
                           for side in ('left', 'right')})
        model = mujoco.MjModel.from_xml_path(str(model_path))
        self.sim = AuthorizedHandMujoco(model=model, data=mujoco.MjData(model), robot_config=robot,
            publisher_instance_id=executor_id, coordinator_instance_id=coordinator_id,
            router_zid=router_zid, hand_sides=self.hand_sides, clock=clock,
            hand_producer_id='official_wuji_hand2', hand_producer_instance_id=instance_id + '-hand',
            bilateral_session=dict(run_id=run_id, execution_epoch=1))
        self.coordinator = None
        try:
            settings = ArmCommandCoordinator._coordinator_config(assets / 'config/coordinator/arm_v131.yaml')
            settings['command_step_clipping_enabled'] = False
            self.coordinator = ArmCommandCoordinator(session, publisher_instance_id=coordinator_id,
                router_zid=router_zid, robot_config=robot, coordinator_config=settings, clock=clock,
                profile=dict(required_capability='simulation', active_sides=['left', 'right'],
                    active_hand_sides=list(hand_sides),
                    authorities=self.authorities, bilateral_proposals=dict(run_id=run_id, execution_epoch=1)))
            worker = SparkWorkerClient(worker=root / 'build/spark-native/spark_native_worker',
                config=config, model=model_path, urdf=urdf, startup_handshake=True)
        except BaseException:
            if self.coordinator is not None:
                self.coordinator.close()
            self.sim.close()
            raise
        try:
            self.producer = SparkProducer(worker, run_id=run_id, execution_epoch=1,
                publisher_instance_id=producer_id, coordinator_instance_id=coordinator_id,
                router_zid=router_zid, receiver_instance_id=self.source_instance_id,
                maximum_receipt_age_ns=100_000_000, session_timeout_ns=200_000_000, max_in_flight=1)
        except BaseException:
            worker.close()
            self.coordinator.close()
            self.sim.close()
            raise
        self.cycle = SparkCoordinatorCycle(self.coordinator, self.producer)
        if hand_sides:
            try:
                hand_producer = HandRetargetProducer(hand_backend,
                    publisher_instance_id=instance_id + '-hand', router_zid=router_zid,
                    coordinator_instance_id=coordinator_id, receiver_instance_id=instance_id + '-manus',
                    freshness_ns=200_000_000)
                self.hand_loop = HandRetargetLoop(hand_producer, hand_source, publish=self._queue_hands, clock=clock)
            except BaseException:
                self.close()
                raise

    def _queue_hands(self, commands):
        before_sink = self.clock()
        if self._hand_command_sink is not None:
            self._hand_command_sink(commands)
        with self._hand_lock:
            if len(self._hand_commands) >= 256:
                raise RuntimeError('hand command queue overflow')
            queued = self.clock()
            self._hand_commands.append((commands, queued, queued - before_sink))

    @property
    def failure(self):
        """Latched acquisition/processing failure, including while idle."""
        return self._failure

    @property
    def last_hand_commands(self):
        """Latest accepted command per side in this tick (not raw callback history)."""
        return deepcopy(self._last_hand_commands)

    @property
    def hand_telemetry(self):
        """Snapshots consumed by coordination; reading never advances a sequence."""
        if self._hand_producer_status is None:
            return {}
        return deepcopy(dict(producer=self._hand_producer_status.to_dict(),
            executors={side: status.to_dict() for side, status in self._hand_executor_statuses.items()}))

    def rearm_at_home(self):
        """Explicit Home-only recovery; not a fault clear or automatic start.

        All three participants are owned by this control thread. No transport
        callback can mutate their execution epoch during this barrier.
        """
        if self._closed or self._failure or not self.producer.paused:
            raise ValueError('Home rearm requires a paused, healthy session')
        epoch = self.producer.guard.execution_epoch + 1
        co = self.coordinator
        co.validate_bilateral_home_rearm(epoch)
        self.sim.validate_bilateral_home_rearm(epoch)
        self.producer.update_session(co.state)
        positions = list(self.sim.arm_state.position_rad)
        try:
            ack = self.producer.rearm_at_rest(positions, execution_epoch=epoch)
            # Worker reconstruction can outlast feedback freshness. Refresh
            # the stationary simulator snapshot; do not advance IK/coordinator.
            now = self.clock()
            self.sim.tick(now_ns=now)
            co.update_arm_state(self.sim.arm_state, received_ns=now)
            self._update_hand_feedback(now)
            # No commands were issued during reconstruction; refresh the idle
            # session heartbeat before executor validation on the same owner.
            co.rearm_bilateral_at_home(epoch)
            self.sim.on_session_state(co.state)
            self.sim.rearm_bilateral_at_home(epoch)
            self._input_after_ns = self.clock()
            co.update_component(self.producer.status(now), received_ns=now)
            with self._hand_lock:
                self._hand_commands.clear()
            return ack
        except Exception as exc:
            self._failure = f'Home rearm failed; restart required: {exc}'
            self.producer.guard.pause(self._failure)
            raise

    def _update_hand_feedback(self, now):
        """Read owned simulator state; never refresh external input age."""
        for side in self.hand_sides:
            state = self.sim.hand_state(side)
            self.coordinator.update_hand_state(state, received_ns=now)
            status = HandExecutorStatus(1, state.sequence, now, side,
                self.sim.status.ready, self.sim.status.healthy,
                self.sim.hand_config.at_zero(state.position_rad), self.coordinator.state.state == 'teleop',
                None, self.sim.publisher_instance_id, self.router_zid)
            self._hand_executor_statuses[side] = status
            self.coordinator.update_hand_executor_status(status, received_ns=now)

    def request(self, action):
        if self._closed:
            raise RuntimeError('simulation closed')
        if action not in ('start', 'return', 'shutdown'):
            raise ValueError('unsupported operator action')
        self._intent += 1
        return self.coordinator.handle_intent(SimpleNamespace(action=action, sequence=self._intent,
            reason='explicit keyboard request', source='tjvr', publisher_instance_id=self.source_instance_id,
            router_zid=self.router_zid))

    def step(self, sample=None, *, source_failure=None):
        if self._closed:
            raise RuntimeError('simulation closed')
        now = self.clock()
        if type(now) is not int or not self._last_ns < now < 2**63:
            raise ValueError('control clock must be increasing positive int64')
        self._last_ns = now
        self._last_hand_commands = {}
        self._event += 1
        if self.hand_loop is not None and self.hand_loop.failure:
            self._failure = self.hand_loop.failure
        if source_failure:
            self._failure = str(source_failure)
        # A queued frame received during reconstruction is not new input for
        # the new execution epoch. Drain it without treating it as a fault.
        if sample is not None and sample.observation.frame.received_timestamp_ns <= self._input_after_ns:
            sample = None
        if sample is not None and not self.producer.update_input(sample):
            self._failure = 'source identity/sequence rejected'
        if self._failure:
            self.producer.guard.pause(self._failure)
        self.source_status = ComponentStatus(1, self._event, now, 'source', 'tjvr',
            'fault' if self._failure else 'ready', not bool(self._failure), not bool(self._failure),
            ['simulation'], self._failure, {}, self.source_instance_id, self.router_zid)
        co = self.coordinator
        co.update_component(self.source_status, received_ns=now)
        self.sim.on_session_state(co.state)
        self.sim.tick(now_ns=now)
        co.update_component(self.sim.status, received_ns=now)
        co.update_arm_state(self.sim.arm_state, received_ns=now)
        if self.hand_loop is not None:
            self.hand_loop.update_session(co.state)
            self._hand_producer_status = self.hand_loop.status(now, self.hand_sides)
            co.update_component(self._hand_producer_status, received_ns=now)
            self._update_hand_feedback(now)
        cycle_started = time.monotonic_ns()
        result = self.cycle.step(now)
        cycle_duration_ns = time.monotonic_ns() - cycle_started
        if not self.sim.on_bilateral_command(co.last_bilateral_command):
            self.producer.guard.pause('paired simulator command rejected')
            raise RuntimeError('paired simulator command rejected')
        self.sim.on_session_state(co.state)
        if self.hand_loop is not None:
            self.hand_loop.update_session(co.state)
            with self._hand_lock:
                commands, self._hand_commands = self._hand_commands, deque()
            if co.state.state == 'teleop':
                for batch, queued_ns, sink_duration_ns in commands:
                    for side, command in batch.items():
                        if side in self.hand_sides and not self.sim.on_hand_command(command):
                            reason = 'authorized hand command rejected: ' + str(self.sim.last_hand_rejection) + str(dict(
                                input_to_enqueue_age_ns=queued_ns-command.timestamp_ns,
                                command_queue_age_ns=self.clock()-queued_ns,
                                hand_sink_duration_ns=sink_duration_ns,
                                arm_cycle_duration_ns=cycle_duration_ns,
                                hand_telemetry=self.hand_telemetry))
                            self.producer.guard.pause(reason)
                            raise RuntimeError(reason)
                        if side in self.hand_sides:
                            self._last_hand_commands[side] = command
        self.sim.tick(now_ns=self.clock())
        # Store executed feedback, not only the pre-command sample. In
        # particular the final Home command can become exact on this tick.
        co.update_arm_state(self.sim.arm_state, received_ns=self.clock())
        return result

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self.hand_loop is not None:
                self.hand_loop.close()
        finally:
            try:
                self.producer.close()
            finally:
                self.coordinator.close()
                self.sim.close()
