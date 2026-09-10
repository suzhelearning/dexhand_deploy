import importlib.util
from pathlib import Path
import unittest

from tianji_teleop.coordination.arm_command_coordinator import ArmRobotConfig


class BilateralRobotTest(unittest.TestCase):
    def test_old_robot_limits_api_preserves_shared_values(self):
        robot = ArmRobotConfig.load()
        self.assertTrue(callable(getattr(robot, 'limits', None)))
        self.assertEqual(robot.limits('left'), (robot.lower_limits_rad, robot.upper_limits_rad))
        self.assertEqual(robot.limits('right'), robot.limits('left'))
        with self.assertRaises(ValueError):
            robot.limits('unknown')

    def test_outside_home_urdf_rejected(self):
        module = 'tianji_teleop.coordination.bilateral_robot'
        self.assertIsNotNone(importlib.util.find_spec(module), 'missing side-specific robot configuration')
        from tianji_teleop.coordination.bilateral_robot import BilateralArmRobotConfig
        root = Path(__file__).resolve().parents[1]
        with self.assertRaises(ValueError):
            BilateralArmRobotConfig.from_urdf(
                root / 'src/tianji_teleop/assets/spark/marvin_m6_s_ccs_696_v4_local.urdf',
                [99.] * 7, [0.] * 7)
