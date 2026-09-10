import importlib.util
from pathlib import Path
import unittest

import mujoco
from tianji_teleop.producers.spark.factory import reference_robot_config
from tianji_teleop.protocol.messages import HAND_JOINT_NAMES, HandJointCommand, SessionState

ROOT = Path(__file__).resolve().parents[1] / 'src/tianji_teleop'


class AuthorizedHandMujocoTest(unittest.TestCase):
    def make(self):
        name = 'tianji_teleop.executors.mujoco.authorized_hand'
        self.assertIsNotNone(importlib.util.find_spec(name))
        from tianji_teleop.executors.mujoco.authorized_hand import AuthorizedHandMujoco
        robot = reference_robot_config(ROOT / 'config/producers/spark_reference.yaml',
            ROOT / 'assets/spark/marvin_m6_s_ccs_696_v4_local.urdf', ROOT / 'config/robot/arm.yaml')
        model = mujoco.MjModel.from_xml_path(str(ROOT / 'assets/spark/marvin_m6_wuji2.xml'))
        self.now = 1_000_000_000
        sim = AuthorizedHandMujoco(model=model, data=mujoco.MjData(model), robot_config=robot,
            publisher_instance_id='sim', router_zid='router', coordinator_instance_id='coord',
            bilateral_session=dict(run_id='run', execution_epoch=1), clock=lambda: self.now,
            hand_producer_id='official_wuji_hand2', hand_producer_instance_id='hand')
        self.addCleanup(sim.close)
        return sim

    def state(self, phase, seq):
        return SessionState(1, seq, self.now, phase, 'test', 'coordinator', None, 'coord', 'router')

    def command(self, seq=1, instance='hand'):
        return HandJointCommand(1, seq, self.now, 'official_wuji_hand2', 'left',
            list(HAND_JOINT_NAMES['left']), [.1] * 20, instance, 'router')

    def test_unarmed_and_foreign_commands_cannot_move_hand(self):
        sim = self.make()
        self.assertFalse(sim.on_hand_command(self.command()))
        sim.on_session_state(self.state('teleop', 1))
        self.assertFalse(sim.on_hand_command(self.command(instance='foreign')))
        self.assertTrue(sim.on_hand_command(self.command()))
        sim.tick()
        self.assertEqual(sim.hand_state('left').position_rad, [.1] * 20)

    def test_stop_discards_pending_command_and_fault_holds_actual_pose(self):
        sim = self.make()
        sim.on_session_state(self.state('teleop', 1))
        self.assertTrue(sim.on_hand_command(self.command()))
        sim.on_session_state(self.state('fault', 2))
        sim.tick()
        self.assertEqual(sim.hand_state('left').position_rad, [0.] * 20)

    def test_rejection_diagnostics_distinguish_command_age_from_session_age(self):
        sim = self.make()
        self.assertTrue(hasattr(sim, 'last_hand_rejection'))
        sim.on_session_state(self.state('teleop', 1))
        old = self.command()
        self.now += 300_000_000
        sim.on_session_state(self.state('teleop', 2))
        self.assertFalse(sim.on_hand_command(old))
        self.assertIn('command_age_ns=300000000', sim.last_hand_rejection)
        self.assertTrue(sim.on_hand_command(self.command(2)))
        self.assertIsNone(sim.last_hand_rejection)
        self.now += 300_000_000
        self.assertFalse(sim.on_hand_command(self.command(3)))
        self.assertIn('session_age_ns=300000000', sim.last_hand_rejection)

    def test_return_zero_is_executor_action_not_retarget_and_stale_session_holds(self):
        sim = self.make()
        sim.on_session_state(self.state('teleop', 1))
        sim.on_hand_command(self.command())
        sim.tick()
        self.now += 300_000_000
        self.assertFalse(sim.on_hand_command(self.command(2)))
        sim.tick()
        self.assertEqual(sim.hand_state('left').position_rad, [.1] * 20)
        sim.on_session_state(self.state('returning', 2))
        sim.tick()
        self.assertEqual(sim.hand_state('left').position_rad, [0.] * 20)

    def test_exact_200ms_command_boundary_is_not_relaxed_by_fresh_session(self):
        sim = self.make()
        old = self.command()
        self.now += 200_000_000
        sim.on_session_state(self.state('teleop', 1))
        self.assertTrue(sim.on_hand_command(old))
        sim.tick()
        held = list(sim.hand_state('left').position_rad)
        self.now += 1
        sim.on_session_state(self.state('teleop', 2))
        old.sequence += 1
        old.position_rad = [.2] * 20
        self.assertFalse(sim.on_hand_command(old))
        self.assertIn('command_age_ns=200000001', sim.last_hand_rejection)
        sim.tick()
        self.assertEqual(sim.hand_state('left').position_rad, held)
