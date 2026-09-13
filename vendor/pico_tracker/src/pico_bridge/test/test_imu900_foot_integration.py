from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).parents[1]


class Imu900FootIntegrationTest(unittest.TestCase):
    def test_driver_parameters_match_reference_profile(self):
        config_path = ROOT / "config/imu900_feet.yaml"
        self.assertTrue(config_path.exists(), f"missing integration config: {config_path}")
        config = yaml.safe_load(config_path.read_text())
        params = config["im900_foot_multi_node"]["ros__parameters"]
        self.assertEqual(
            params["channel_names"], ["im900/left_foot", "im900/right_foot"]
        )
        self.assertEqual(
            params["frame_ids"], ["left_foot_imu_link", "right_foot_imu_link"]
        )
        self.assertEqual(params["baudrate"], 115200)
        self.assertEqual(params["report_hz"], 110)
        self.assertEqual(params["report_tag"], 46)
        self.assertEqual(params["target_address"], 255)
        self.assertTrue(params["enable_compass"])
        self.assertTrue(params["use_device_timestamp"])
        self.assertTrue(params["use_quaternion_continuity"])
        self.assertFalse(params["clear_world_axes"])
        self.assertFalse(params["restore_world_axes"])

    def test_launch_defaults_and_remaps_are_explicit(self):
        path = ROOT / "launch/start_pico_foot_fusion.launch.py"
        self.assertTrue(path.exists(), f"missing integration launch: {path}")
        spec = spec_from_file_location("start_pico_foot_fusion", path)
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(module.DEFAULT_LEFT_PORT, "/dev/ttyUSB0")
        self.assertEqual(module.DEFAULT_RIGHT_PORT, "/dev/ttyUSB1")
        self.assertEqual(
            module.IMU_REMAPPINGS,
            [
                ("im900/left_foot/imuData_raw", "/imu/left_feet"),
                ("im900/right_foot/imuData_raw", "/imu/right_feet"),
                ("im900/left_foot/ready", "/imu/left_feet/ready"),
                ("im900/right_foot/ready", "/imu/right_feet/ready"),
            ],
        )

    def test_fusion_has_no_legacy_driver_dependency(self):
        source = (ROOT / "src/pico_foot_imu_fusion_node.cpp").read_text()
        cmake = (ROOT / "CMakeLists.txt").read_text()
        package = (ROOT / "package.xml").read_text()
        for text in (source, cmake, package):
            self.assertNotIn("pico_imu900_driver", text)
            self.assertNotIn("DriverStatus", text)
        self.assertIn("orientation_covariance[0] < 0.0", source)

    def test_wrong_driver_is_removed_from_build_workflow(self):
        repository = ROOT.parents[1]
        self.assertFalse((repository / "src/pico_imu900_driver").exists())
        pixi = (repository / "pixi.toml").read_text()
        self.assertNotIn("pico_imu900_driver", pixi)
        self.assertIn("imu_ros2", pixi)

    def test_manifest_declares_launch_and_test_dependencies(self):
        package = (ROOT / "package.xml").read_text()
        self.assertIn("<exec_depend>ament_index_python</exec_depend>", package)
        self.assertIn("<exec_depend>launch</exec_depend>", package)
        self.assertIn("<exec_depend>launch_ros</exec_depend>", package)
        self.assertIn("<exec_depend>ros2launch</exec_depend>", package)
        self.assertIn("<test_depend>python3-yaml</test_depend>", package)

    def test_runtime_driver_exposes_ready_and_reconnects(self):
        source = (ROOT.parents[1] / "src/imu_ros2/src/imu_multi_node.cpp").read_text()
        self.assertIn("std_msgs::msg::Bool", source)
        self.assertIn("ready_publisher_", source)
        self.assertIn("try_reconnect", source)
        self.assertIn("mark_disconnected", source)

    def test_fusion_launch_remaps_ready_topics(self):
        launch = (ROOT / "launch/start_pico_foot_fusion.launch.py").read_text()
        self.assertIn("im900/left_foot/ready", launch)
        self.assertIn("im900/right_foot/ready", launch)

    def test_launch_defaults_to_validated_identity_mount_correction(self):
        launch = (ROOT / "launch/start_pico_foot_fusion.launch.py").read_text()
        self.assertIn('DeclareLaunchArgument("left_mount_quaternion"', launch)
        self.assertIn('DeclareLaunchArgument("right_mount_quaternion"', launch)
        self.assertIn('"left_mount_quaternion": _quaternion_argument(', launch)
        self.assertIn('"right_mount_quaternion": _quaternion_argument(', launch)
        self.assertIn('DEFAULT_MOUNT_QUATERNION = "0,0,0,1"', launch)

    def test_fusion_node_auto_calibrates_after_pico_world_reset(self):
        source = (ROOT / "src/pico_foot_imu_fusion_node.cpp").read_text()
        for token in ("/pico/world_reset", "/im900/left_foot/zero_z_axis",
                      "/im900/right_foot/zero_z_axis", "async_send_request",
                      "auto_calibrate_on_world_reset"):
            self.assertIn(token, source)

    def test_reset_settle_delay_and_sixty_frame_default_are_exposed(self):
        source = (ROOT / "src/pico_foot_imu_fusion_node.cpp").read_text()
        launch = (ROOT / "launch/start_pico_foot_fusion.launch.py").read_text()
        self.assertIn('declare_parameter("imu_reset_settle_sec", 1.0)', source)
        self.assertIn("create_wall_timer", source)
        self.assertIn('DeclareLaunchArgument("imu_reset_settle_sec", default_value="1.0")', launch)
        self.assertIn('DeclareLaunchArgument("calibration_samples", default_value="60")', launch)
        self.assertIn('"imu_reset_settle_sec": ParameterValue(', launch)


if __name__ == "__main__":
    unittest.main()
