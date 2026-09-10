"""Offline single-clock integration harness, with no network/device authority.

This is not the live session launcher: it explicitly authorizes an offline
simulation after readiness. Coordinator disposition is checked before the
MuJoCo tick; actual simulator state is compared separately, never fed into
the reference model state. No retries or hidden processing stages.
"""
from types import SimpleNamespace

import mujoco
import numpy as np

from ...coordination.arm_command_coordinator import ArmCommandCoordinator
from ...executors.mujoco.node import MujocoExecutor
from ...hand_tracking.spark_worker_client import SparkWorkerClient
from ...protocol.messages import ComponentStatus
from .factory import reference_robot_config
from .node import SparkProducer
from .coordinator_cycle import SparkCoordinatorCycle


class OfflineSparkSimulation:
    def __init__(self, root, *, receiver_instance_id='offline-replay', hand_sides=(), cycle_sink=None,
                 automatic_start=True):
        if type(automatic_start) is not bool:
            raise ValueError('automatic_start must be boolean')
        self._automatic_start = automatic_start
        self._start_requested = False
        assets = root / 'src/tianji_teleop'
        config = assets / 'config/producers/spark_reference.yaml'
        model_path = assets / 'assets/spark/marvin_m6_wuji2.xml'
        urdf = assets / 'assets/spark/marvin_m6_s_ccs_696_v4_local.urdf'
        robot = reference_robot_config(config, urdf, assets / 'config/robot/arm.yaml')
        self.now_ns = 1_000_000_000
        self._event = 0
        self._failed = False
        self._cycle_sink = cycle_sink
        self.control_ticks = 0
        self.maximum_command_error_rad = 0.
        self.maximum_sim_error_rad = 0.
        self._minimum = np.full(14, np.inf)
        self._maximum = np.full(14, -np.inf)
        clock = lambda: self.now_ns
        model = mujoco.MjModel.from_xml_path(str(model_path))
        self.sim = MujocoExecutor(model=model, data=mujoco.MjData(model), robot_config=robot,
            publisher_instance_id='sim', router_zid='offline', coordinator_instance_id='coord',
            hand_sides=hand_sides, clock=clock,
            bilateral_session=dict(run_id='offline-smoke', execution_epoch=1))
        try:
            settings = ArmCommandCoordinator._coordinator_config(assets / 'config/coordinator/arm_v131.yaml')
            settings['command_step_clipping_enabled'] = False
            self.coordinator = ArmCommandCoordinator(None, publisher_instance_id='coord', router_zid='offline',
                robot_config=robot, coordinator_config=settings, clock=clock,
                profile=dict(required_capability='simulation', active_sides=['left', 'right'],
                             bilateral_proposals=dict(run_id='offline-smoke', execution_epoch=1)))
            worker = SparkWorkerClient(worker=root / 'build/spark-native/spark_native_worker',
                config=config, model=model_path, urdf=urdf, deterministic_test=True)
        except BaseException:
            self.sim.close()
            raise
        try:
            self.producer = SparkProducer(worker, run_id='offline-smoke', execution_epoch=1,
                publisher_instance_id='spark', coordinator_instance_id='coord', router_zid='offline',
                receiver_instance_id=receiver_instance_id, maximum_receipt_age_ns=100_000_000,
                session_timeout_ns=200_000_000, max_in_flight=1)
        except BaseException:
            worker.close()
            self.sim.close()
            raise
        self.cycle = SparkCoordinatorCycle(self.coordinator, self.producer)

    def step(self, tick):
        if self._failed:
            raise RuntimeError('offline simulation failed; no implicit resume')
        try:
            return self._step(tick)
        except BaseException:
            self._failed = True
            self.producer.guard.pause('offline downstream execution failure')
            raise

    def request_start(self, now_ns):
        """Explicit request for the NEXT tick; rejected requests never queue."""
        if type(now_ns) is not int or not 0 < now_ns < 2**63 or now_ns < self.now_ns:
            raise ValueError('start request clock must be current positive int64')
        self._start_requested = False
        if self._failed or self.coordinator.state.state != 'idle' or not self.producer.status(now_ns).ready:
            return False
        self._start_requested = True
        return True

    def _step(self, tick):
        if (type(tick.now_ns) is not int or not 0 < tick.now_ns < 2**63 or
                (self._event and tick.now_ns <= self.now_ns)):
            raise ValueError('offline clock must increase')
        self.now_ns = tick.now_ns
        self._event += 1
        if tick.sample is not None and not self.producer.update_input(tick.sample):
            raise ValueError('offline input identity/sequence rejected')
        co = self.coordinator
        co.update_component(ComponentStatus(1, self._event, self.now_ns, 'source', 'tjvr',
            'ready', True, True, ['simulation'], None, {}, 'source', 'offline'))
        status = self.producer.status(self.now_ns)
        co.update_component(status)
        self.sim.tick(now_ns=self.now_ns)
        co.update_component(self.sim.status)
        co.update_arm_state(self.sim.arm_state)
        if co.state.state == 'idle':
            requested = self._automatic_start or self._start_requested
            self._start_requested = False
            if not requested or not status.ready:
                self.cycle.step(self.now_ns)
                return None
            accepted = co.handle_intent(SimpleNamespace(action='start', sequence=1,
                source='tjvr', reason='explicit offline simulation')).accepted
            if not accepted:
                raise RuntimeError('offline coordinator readiness rejected')
        disposition = self.cycle.step(self.now_ns)
        if disposition.native_result is None:
            raise RuntimeError(self.producer.guard.reason or 'native proposal unavailable')
        commands = disposition.commands
        if co.state.state != 'teleop':
            raise RuntimeError(co.state.reason)
        if not disposition.receipt_accepted:
            raise RuntimeError(self.producer.guard.reason or 'execution receipt rejected')
        result = disposition.native_result
        for side in ('left', 'right'):
            error = max(abs(a - b) for a, b in zip(commands[side].position_rad, result[side]['q']))
            self.maximum_command_error_rad = max(self.maximum_command_error_rad, error)
            if error != 0:
                raise RuntimeError('altered or rejected simulation command')
        if not self.sim.on_bilateral_command(co.last_bilateral_command):
            raise RuntimeError('paired simulation command rejected')
        applied = self.sim.tick(now_ns=self.now_ns)
        if set(applied['arm']) != {'left', 'right'}:
            raise RuntimeError('incomplete bilateral simulation execution')
        expected = np.asarray(commands['left'].position_rad + commands['right'].position_rad)
        error = float(np.max(np.abs(np.asarray(self.sim.arm_state.position_rad) - expected)))
        self.maximum_sim_error_rad = max(self.maximum_sim_error_rad, error)
        if error != 0:
            raise RuntimeError('simulation state differs from command')
        self.control_ticks += 1
        self._minimum = np.minimum(self._minimum, expected)
        self._maximum = np.maximum(self._maximum, expected)
        if self._cycle_sink is not None:
            self._cycle_sink(result, commands, self.sim.arm_state, co.state)
        return result

    def report(self):
        return dict(simulation_only=True, real_time_qualified=False,
            deterministic_test=True, control_ticks=self.control_ticks,
            maximum_command_error_rad=self.maximum_command_error_rad,
            maximum_sim_error_rad=self.maximum_sim_error_rad,
            maximum_joint_range_rad=float(np.max(self._maximum - self._minimum)) if self.control_ticks else 0.,
            complete=not self._failed)

    def close(self):
        self.producer.close()
        self.sim.close()
