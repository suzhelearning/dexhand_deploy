import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import yaml

from tianji_teleop.coordination.arm_command_coordinator import ArmRobotConfig

ROOT = Path(__file__).resolve().parents[1]
ARM = ROOT / 'src/tianji_teleop/config/robot/arm.yaml'
URDF = ROOT / 'src/tianji_teleop/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf'


class JointLimitSourceTest(unittest.TestCase):
    def test_urdf_snapshot_preserves_home_and_replaces_all_limits(self):
        from tianji_teleop.joint_limit_source import resolve_arm_config
        result = resolve_arm_config(ARM, URDF, 'urdf')
        original = yaml.safe_load(ARM.read_text())
        for key in ('left_joint_names', 'right_joint_names', 'left_home_rad', 'right_home_rad'):
            self.assertEqual(result[key], original[key])
        self.assertEqual(result['upper_limits_rad'], [3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708])
        ArmRobotConfig.from_mapping(result)

    def test_yaml_mode_is_unchanged(self):
        from tianji_teleop.joint_limit_source import resolve_arm_config
        self.assertEqual(resolve_arm_config(ARM, URDF, 'yaml'), yaml.safe_load(ARM.read_text()))

    def test_snapshot_roundtrip_native_layout_and_no_overwrite(self):
        from tianji_teleop.joint_limit_source import resolve_arm_config, write_snapshot
        value = resolve_arm_config(ARM, URDF, 'urdf')
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'arm.yaml'
            write_snapshot(value, output)
            self.assertEqual(ArmRobotConfig.load(output), ArmRobotConfig.from_mapping(value))
            self.assertIn('upper_limits_rad: [', output.read_text())
            self.assertIn('left_joint_names:\n- Joint1_L\n', output.read_text())
            with self.assertRaises(FileExistsError):
                write_snapshot(value, output)

    def test_selected_limits_reach_coordinator_and_mujoco_without_changing_home(self):
        from tianji_teleop.joint_limit_source import resolve_arm_config, write_snapshot
        from tianji_teleop.coordination.arm_command_coordinator import ArmCommandCoordinator
        from tianji_teleop.executors.mujoco.node import MujocoExecutor
        from test_h5_interaction_overlay import _FakeModel, _FakeData
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'arm.yaml'
            write_snapshot(resolve_arm_config(ARM, URDF, 'urdf'), output)
            with patch.dict(os.environ, {'TIANJI_ARM_CONFIG': str(output)}):
                coordinator = ArmCommandCoordinator(session=None, publisher_instance_id='coord', router_zid='test')
                executor = MujocoExecutor(model=_FakeModel(), data=_FakeData(),
                    publisher_instance_id='sim', router_zid='test', coordinator_instance_id='coord', hand_sides=())
                self.addCleanup(executor.close)
                self.addCleanup(coordinator.close)
                self.assertEqual(executor.robot, coordinator.robot)
                self.assertEqual(executor.robot.upper_limits_rad[5], 1.0472)
                self.assertEqual(executor.robot.home_all, ArmRobotConfig.load(ARM).home_all)

    def test_missing_or_asymmetric_urdf_fails_closed(self):
        from tianji_teleop.joint_limit_source import resolve_arm_config
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'robot.urdf'
            p.write_text('<robot/>')
            with self.assertRaises(ValueError):
                resolve_arm_config(ARM, p, 'urdf')
            import xml.etree.ElementTree as ET
            tree = ET.parse(URDF)
            tree.find("joint[@name='Joint6_R']/limit").set('upper', '0.9')
            tree.write(p)
            with self.assertRaisesRegex(ValueError, 'left/right'):
                resolve_arm_config(ARM, p, 'urdf')

    def test_loader_honors_session_config_but_explicit_path_wins(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'arm.yaml'
            value = yaml.safe_load(ARM.read_text())
            value['upper_limits_rad'][5] = 1.0472
            p.write_text(yaml.safe_dump(value))
            with patch.dict(os.environ, {'TIANJI_ARM_CONFIG': str(p)}):
                self.assertEqual(ArmRobotConfig.load().upper_limits_rad[5], 1.0472)
                self.assertEqual(ArmRobotConfig.load(ARM).upper_limits_rad[5], 0.9599310886)

    def test_launcher_rejects_real_and_unknown_sources_before_starting(self):
        for profile, source in [('mocap_live_real', 'urdf'), ('hand_tracking_sim', 'invalid')]:
            result = subprocess.run(['bash', 'scripts/run_session.sh', '--profile', profile,
                                     '--joint-limit-source', source], cwd=ROOT,
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2)
            self.assertIn('joint-limit-source', result.stderr)
