from pathlib import Path
import unittest

import mujoco
import numpy as np

from tianji_teleop.executors.mujoco.node import MujocoExecutor
from tianji_teleop.producers.spark.factory import reference_robot_config
from tianji_teleop.protocol.messages import ArmJointCommand

ROOT = Path(__file__).resolve().parents[1] / 'src/tianji_teleop'


class SparkMujocoExecutorTest(unittest.TestCase):
    def test_reference_model_home_and_asymmetric_command_limits(self):
        robot = reference_robot_config(
            ROOT / 'config/producers/spark_reference.yaml',
            ROOT / 'assets/spark/marvin_m6_s_ccs_696_v4_local.urdf',
            ROOT / 'config/robot/arm.yaml')
        model = mujoco.MjModel.from_xml_path(str(ROOT / 'assets/spark/marvin_m6_wuji2.xml'))
        sim = MujocoExecutor(model=model, data=mujoco.MjData(model),
                             publisher_instance_id='sim', router_zid='router',
                             coordinator_instance_id='coord', robot_config=robot,
                             hand_sides=(), clock=lambda: 1_000_000_000)
        self.addCleanup(sim.close)
        np.testing.assert_allclose(sim.arm_state.position_rad, robot.home_all)
        for side in ('left', 'right'):
            values = list(getattr(robot, f'{side}_home_rad'))
            command = ArmJointCommand(1, 1, 1_000_000_000, 'coordinator', side, 'teleop', 1, 1,
                                      list(getattr(robot, f'{side}_joint_names')), values,
                                      'coord', 'router')
            self.assertTrue(sim.on_arm_command(command))
            # Joint 3 is negative only on the left, positive only on the right.
            values[2] = 0.5 if side == 'left' else -0.5
            invalid = command.to_dict()
            invalid['sequence'] = 2
            invalid['position_rad'] = values
            self.assertFalse(sim.on_arm_command(invalid))
