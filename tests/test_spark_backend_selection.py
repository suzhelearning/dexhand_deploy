import importlib.util
from pathlib import Path
import unittest

from tianji_teleop.hand_tracking.input_modes import resolve_input_mode, SPARK_BACKEND

ROOT = Path(__file__).resolve().parents[1]


class SparkBackendSelectionTest(unittest.TestCase):
    def module(self):
        name = 'tianji_teleop.producers.spark.factory'
        self.assertIsNotNone(importlib.util.find_spec(name), 'missing bilateral backend factory')
        from tianji_teleop.producers.spark import factory
        return factory

    def mode(self, arm='tjvr_corrected_palm'):
        return resolve_input_mode(dict(input_mode='vr_manus', hand_input='manus', arm_input=arm, operator_input='controller'))

    def test_pose_only_unknown_and_real_rejected_before_spawn(self):
        factory = self.module()
        for backend, mode, capability in ((SPARK_BACKEND, self.mode('xr_controller'), 'simulation'),
                                          ('pico_ee_dexhand_qp', self.mode(), 'simulation'),
                                          (SPARK_BACKEND, self.mode(), 'real')):
            with self.assertRaises(ValueError):
                factory.create_bilateral_backend(backend, mode, required_capability=capability)

    def test_reference_home_and_urdf_limits_are_used_without_changing_old_yaml(self):
        factory = self.module()
        base = ROOT / 'src/tianji_teleop/config/robot/arm.yaml'
        before = base.read_bytes()
        robot = factory.reference_robot_config(
            ROOT / 'src/tianji_teleop/config/producers/spark_reference.yaml',
            ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_s_ccs_696_v4_local.urdf', base)
        self.assertEqual(robot.left_home_rad, (1.10, -1.52, -1.52, -1.10, 0., 0., 0.))
        self.assertEqual(robot.right_home_rad, (-1.10, -1.52, 1.52, -1.10, 0., 0., 0.))
        self.assertEqual(robot.limits('left')[1][5], 1.0472)
        self.assertEqual(robot.limits('left')[1][2], 0.)
        self.assertEqual(robot.limits('right')[0][2], 0.)
        self.assertEqual(robot.limits('left')[0][0], -1.5708)
        self.assertEqual(robot.limits('right')[1][0], 1.5708)
        self.assertEqual(base.read_bytes(), before)

    def test_live_home_comes_from_arm_yaml(self):
        import yaml
        factory = self.module()
        base = ROOT / 'src/tianji_teleop/config/robot/arm.yaml'
        robot = factory.reference_robot_config(
            ROOT/'src/tianji_teleop/config/producers/spark_reference.yaml',
            ROOT/'src/tianji_teleop/assets/spark/marvin_m6_s_ccs_696_v4_local.urdf',
            base, use_arm_home=True)
        expected = yaml.safe_load(base.read_text())
        self.assertEqual(list(robot.left_home_rad), expected['left_home_rad'])
        self.assertEqual(list(robot.right_home_rad), expected['right_home_rad'])
