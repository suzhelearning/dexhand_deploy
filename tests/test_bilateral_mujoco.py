from pathlib import Path
import unittest

import mujoco

from tianji_teleop.executors.mujoco.node import MujocoExecutor
from tianji_teleop.producers.spark.factory import reference_robot_config
from tianji_teleop.protocol.bilateral import ArmBilateralCommand
from tianji_teleop.protocol.messages import ArmJointCommand

ROOT = Path(__file__).resolve().parents[1] / 'src/tianji_teleop'


class BilateralMujocoTest(unittest.TestCase):
    def make(self):
        robot = reference_robot_config(ROOT / 'config/producers/spark_reference.yaml',
            ROOT / 'assets/spark/marvin_m6_s_ccs_696_v4_local.urdf', ROOT / 'config/robot/arm.yaml')
        model = mujoco.MjModel.from_xml_path(str(ROOT / 'assets/spark/marvin_m6_wuji2.xml'))
        sim = MujocoExecutor(model=model, data=mujoco.MjData(model), robot_config=robot,
            publisher_instance_id='sim', router_zid='router', coordinator_instance_id='coord',
            bilateral_session=dict(run_id='run', execution_epoch=1), hand_sides=(), clock=lambda: 1_000_000_000)
        self.addCleanup(sim.close)
        return sim

    def pair(self, sim, seq=1, delta=.01):
        values = {side: ArmJointCommand(1, seq, 1_000_000_000, 'coordinator', side, 'teleop', seq, 5,
                     list(getattr(sim.robot, side + '_joint_names')),
                     [value + delta for value in getattr(sim.robot, side + '_home_rad')], 'coord', 'router')
                  for side in ('left', 'right')}
        return ArmBilateralCommand('run', 1, seq, values['left'], values['right'])

    def test_valid_pair_applies_both_and_bad_side_latches_hold(self):
        sim = self.make()
        self.assertTrue(sim.on_bilateral_command(self.pair(sim)))
        sim.tick()
        before = sim.arm_state.position_rad
        bad = self.pair(sim, 2, .02).to_dict()
        bad['right']['position_rad'][0] = 99.
        self.assertFalse(sim.on_bilateral_command(bad))
        sim.tick()
        self.assertEqual(sim.arm_state.position_rad, before)
        self.assertFalse(sim.on_bilateral_command(self.pair(sim, 3, .03)))
        sim.tick()
        self.assertEqual(sim.arm_state.position_rad, before)

    def test_foreign_run_epoch_and_duplicate_do_not_replace_command(self):
        sim = self.make()
        pair = self.pair(sim)
        self.assertTrue(sim.on_bilateral_command(pair))
        self.assertFalse(sim.on_bilateral_command(pair))
        for field, value in (('run_id', 'old'), ('execution_epoch', 2)):
            row = self.pair(sim, 2).to_dict()
            row[field] = value
            self.assertFalse(sim.on_bilateral_command(row))
        self.assertTrue(sim.on_bilateral_command(self.pair(sim, 2)))

    def test_single_command_cannot_enter_bilateral_executor(self):
        sim = self.make()
        self.assertFalse(sim.on_arm_command(self.pair(sim).left))
        self.assertEqual(sim.tick()['arm'], {})

    def test_failure_discards_unexecuted_pair_instead_of_moving_to_it(self):
        sim = self.make()
        before = sim.arm_state.position_rad
        self.assertTrue(sim.on_bilateral_command(self.pair(sim)))
        bad = self.pair(sim, 2).to_dict()
        bad['right']['position_rad'][0] = 99.
        self.assertFalse(sim.on_bilateral_command(bad))
        self.assertEqual(sim.tick()['arm'], {})
        self.assertEqual(sim.arm_state.position_rad, before)
