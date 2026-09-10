"""Actual native SPARK through coordinator and MuJoCo, not a solver mock."""
import os
from pathlib import Path
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np

from tianji_teleop.coordination.arm_command_coordinator import ArmCommandCoordinator
from tianji_teleop.executors.mujoco.node import MujocoExecutor
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.hand_tracking.spark_replay import iter_reference_ticks
from tianji_teleop.hand_tracking.spark_worker_client import SparkWorkerClient
from tianji_teleop.hand_tracking.tjvr_trace import iter_tjvr_records
from tianji_teleop.producers.spark.factory import reference_robot_config
from tianji_teleop.producers.spark.node import SparkProducer
from tianji_teleop.protocol.messages import ComponentStatus

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'src/tianji_teleop'


@unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional native coordinated simulation')
class SparkCoordinatedSimTest(unittest.TestCase):
    def test_entire_short_trace_commands_equal_native_and_sim_state(self):
        trace_root = ROOT / 'vendor/reference_inputs/remote-192.168.110.210-20260908'
        traces = list(trace_root.rglob('output_continuity_retest.tjvr'))
        if len(traces) != 1:
            self.skipTest('private short trace unavailable')
        config = ASSETS / 'config/producers/spark_reference.yaml'
        model_path = ASSETS / 'assets/spark/marvin_m6_wuji2.xml'
        urdf = ASSETS / 'assets/spark/marvin_m6_s_ccs_696_v4_local.urdf'
        robot = reference_robot_config(config, urdf, ASSETS / 'config/robot/arm.yaml')
        now = 1_000_000_000
        clock = lambda: now
        model = mujoco.MjModel.from_xml_path(str(model_path))
        sim = MujocoExecutor(model=model, data=mujoco.MjData(model), robot_config=robot,
            publisher_instance_id='sim', router_zid='router', coordinator_instance_id='coord',
            hand_sides=(), clock=clock)
        self.addCleanup(sim.close)
        settings = ArmCommandCoordinator._coordinator_config(ASSETS / 'config/coordinator/arm_v131.yaml')
        settings['command_step_clipping_enabled'] = False
        coordinator = ArmCommandCoordinator(None, publisher_instance_id='coord', router_zid='router',
            robot_config=robot, coordinator_config=settings, clock=clock,
            profile=dict(required_capability='simulation', active_sides=['left', 'right'],
                         bilateral_proposals=dict(run_id='test', execution_epoch=1)))
        worker = SparkWorkerClient(worker=ROOT / 'build/spark-native/spark_native_worker',
            config=config, model=model_path, urdf=urdf, deterministic_test=True)
        self.addCleanup(worker.close)
        producer = SparkProducer(worker, run_id='test', execution_epoch=1,
            publisher_instance_id='spark', coordinator_instance_id='coord', router_zid='router',
            receiver_instance_id='trace', maximum_receipt_age_ns=100_000_000,
            session_timeout_ns=200_000_000, max_in_flight=1)
        receiver = ReferenceTjvrReceiver('trace', .15, .6)
        positions = []
        with traces[0].open('rb') as stream:
            for tick in iter_reference_ticks(iter_tjvr_records(stream), receiver=receiver):
                now = tick.now_ns
                if tick.sample is not None:
                    self.assertTrue(producer.update_input(tick.sample))
                coordinator.update_component(ComponentStatus(1, tick.tick_id, now, 'source', 'tjvr',
                    'ready', True, True, ['simulation'], None, {}, 'source', 'router'))
                coordinator.update_component(producer.status(now))
                sim.tick(now_ns=now)
                coordinator.update_component(sim.status)
                coordinator.update_arm_state(sim.arm_state)
                if coordinator.state.state == 'idle':
                    if not producer.status(now).ready:
                        continue
                    self.assertTrue(coordinator.handle_intent(SimpleNamespace(
                        action='start', sequence=1, source='tjvr', reason='offline test')).accepted)
                producer.update_session(coordinator.state)
                proposal = producer.tick(now)
                self.assertIsNotNone(proposal, producer.guard.reason)
                self.assertTrue(coordinator.update_bilateral_proposal(proposal, received_ns=now))
                commands = coordinator.tick(now_ns=now)
                self.assertEqual(coordinator.state.state, 'teleop', coordinator.state.reason)
                self.assertTrue(producer.observe_execution(coordinator.last_bilateral_receipt, now))
                for side in ('left', 'right'):
                    self.assertEqual(commands[side].position_rad, producer.last_result[side]['q'])
                    self.assertTrue(sim.on_arm_command(commands[side]))
                applied = sim.tick(now_ns=now)
                self.assertEqual(set(applied['arm']), {'left', 'right'})
                expected = commands['left'].position_rad + commands['right'].position_rad
                np.testing.assert_array_equal(sim.arm_state.position_rad, expected)
                positions.append(expected)
        self.assertGreater(len(positions), 5000)
        self.assertGreater(np.ptp(np.asarray(positions), axis=0).max(), 1.)
