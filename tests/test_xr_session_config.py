import os
from pathlib import Path
import tempfile
import unittest

from tianji_teleop import config_loader


class XrSessionConfigTest(unittest.TestCase):
    def test_explicit_xr_manus_profile_resolves_to_manus_and_xr(self):
        value = config_loader.load_yaml(
            config_loader.component_path("sessions/vr_manus_xr_sim.yaml")
        )
        result = config_loader.resolve_dual_session_config(value, disable_hands=False)
        self.assertEqual(result["input_mode"], "vr_manus")
        self.assertEqual(result["arm_input"], "xr_controller")
        self.assertEqual(result["receivers"], ["manus", "xr"])
        self.assertEqual(result["arm_pose_mapper"], "xr_incremental")
        self.assertEqual(result["arm_target_processor"], "conditioned")
        self.assertFalse(result["requires_upper_limb_skeleton"])

    def test_xr_manus_default_is_controller_only_and_has_no_tracker_requirement(self):
        value = config_loader.load_yaml(
            config_loader.component_path("sources/xr_manus_observation.yaml")
        )
        self.assertEqual(value["xr"]["arm_input"], "xr_controller")
        self.assertEqual(value["xr"]["tracker_serials"], {})
        self.assertEqual(value["xr"]["elbow_tracker_serials"], {})

    def test_xr_controller_defaults_match_reference_controller_only_conditioning(self):
        from tianji_teleop.hand_tracking.target_node import _load_config

        value = _load_config(
            config_loader.component_path("sources/hand_tracking_target_xr_manus.yaml")
        )
        self.assertEqual(value["arm_pose_mapper_config"]["expected_tracked_frame"], "controller")
        self.assertEqual(value["arm_pose_mapper_config"]["min_cutoff"], 1.2)
        self.assertEqual(value["arm_pose_mapper_config"]["beta"], 0.45)
        self.assertFalse(value["arm_pose_mapper_config"]["dynamic_elbow_direction"])
        self.assertEqual(value["arm_target_processor"], "conditioned")
        self.assertEqual(value["arm_target_processor_config"]["translation_gain"], [0.90, 0.90, 0.90])
        self.assertEqual(value["arm_target_processor_config"]["workspace_relative_radii_m"], [0.42, 0.38, 0.38])
        self.assertEqual(value["arm_target_processor_config"]["maximum_linear_speed_m_s"], 0.36)
        self.assertEqual(value["arm_target_processor_config"]["maximum_angular_speed_rad_s"], 1.55)

    def test_xr_controller_can_be_selected_without_changing_hand_input(self):
        value = config_loader.load_yaml(
            config_loader.component_path("sessions/vr_manus_xr_sim.yaml")
        )
        value["arm_input"] = "xr_controller"
        result = config_loader.resolve_dual_session_config(value, disable_hands=True)
        self.assertEqual(result["receivers"], ["xr"])
        self.assertEqual(result["active_hand_sides"], [])

    def test_xr_pose_only_route_requires_incremental_mapper(self):
        value = config_loader.load_yaml(
            config_loader.component_path("sessions/vr_manus_xr_sim.yaml")
        )
        value["arm_pose_mapper"] = "none"
        with self.assertRaisesRegex(ValueError, "xr_incremental"):
            config_loader.resolve_dual_session_config(value)

    def test_xr_target_config_is_accepted_by_the_existing_target_boundary(self):
        from tianji_teleop.hand_tracking.target_node import _load_config, select_xr_arm_input
        value = _load_config(
            config_loader.component_path("sources/hand_tracking_target_xr_manus.yaml")
        )
        self.assertEqual(value["arm_input_source"], "xr")
        self.assertEqual(value["arm_pose_mapper"], "xr_incremental")
        self.assertIn("operator_config", value)
        select_xr_arm_input(value, "xr_controller")
        self.assertEqual(value["arm_pose_mapper_config"]["expected_tracked_frame"], "controller")
        select_xr_arm_input(value, "xr_tracker")
        self.assertEqual(value["arm_pose_mapper_config"]["expected_tracked_frame"], "wrist_tracker")

    def test_xr_controller_selects_its_own_tracked_to_wrist_extrinsic(self):
        from tianji_teleop.hand_tracking.target_node import _load_config, select_xr_arm_input
        value = _load_config(
            config_loader.component_path("sources/hand_tracking_target_xr_manus.yaml")
        )
        tracker_pose = {
            "left": [0.01, 0.02, 0.03, 0.0, 0.0, 0.0, 1.0],
            "right": [0.04, 0.05, 0.06, 0.0, 0.0, 0.0, 1.0],
        }
        controller_pose = {
            "left": [0.11, 0.12, 0.13, 0.0, 0.0, 0.0, 1.0],
            "right": [0.14, 0.15, 0.16, 0.0, 0.0, 0.0, 1.0],
        }
        value["arm_pose_mapper_config"]["tracked_to_wrist_pose"] = tracker_pose
        value["arm_pose_mapper_config"]["controller_to_wrist_pose"] = controller_pose

        select_xr_arm_input(value, "xr_controller")
        self.assertEqual(
            value["arm_pose_mapper_config"]["tracked_to_wrist_pose"], controller_pose
        )
        select_xr_arm_input(value, "xr_tracker")
        self.assertEqual(
            value["arm_pose_mapper_config"]["tracked_to_wrist_pose"], tracker_pose
        )

    def test_xr_arm_input_selection_cannot_modify_pico_target_config(self):
        from tianji_teleop.hand_tracking.target_node import _load_config, select_xr_arm_input
        value = _load_config(
            config_loader.component_path("sources/hand_tracking_target.yaml")
        )
        with self.assertRaisesRegex(ValueError, "XR/Manus"):
            select_xr_arm_input(value, "xr_controller")

    def test_xr_manus_observation_config_validates_native_bindings(self):
        from tianji_teleop.hand_tracking.xr_manus_observation import (
            _load_config, _parser, _config_from_args, validate_manus_runtime,
        )
        value = _load_config(
            config_loader.component_path("sources/xr_manus_observation.yaml")
        )
        self.assertEqual(value["xr"]["arm_input"], "xr_controller")
        self.assertIsNone(value["manus"]["user"])
        self.assertIsNone(value["manus"]["rawviz"])
        self.assertIsNone(value["manus"]["library_dir"])
        self.assertIsNone(value["manus"]["right_glove"])
        self.assertIsNone(value["manus"]["left_glove"])
        args = _parser().parse_args([
            "--config", str(config_loader.component_path("sources/xr_manus_observation.yaml")),
            "--manus-rawviz", "/tmp/example-manus/rawviz.out",
            "--manus-user", "gjy",
        ])
        configured = _config_from_args(args)
        self.assertEqual(configured["manus"]["rawviz"], "/tmp/example-manus/rawviz.out")
        self.assertEqual(configured["manus"]["user"], "gjy")

    def test_xr_operator_publisher_uses_runtime_publisher_identity(self):
        from tianji_teleop.hand_tracking.xr_manus_observation import (
            _make_operator_publisher,
        )
        from tianji_teleop.hand_tracking.xr_operator import XrControllerOperatorConfig

        publisher = _make_operator_publisher(
            publisher_instance_id="run-observation",
            epoch=1,
            config=XrControllerOperatorConfig.from_mapping({
                "start": {"side": "right", "control": "grip", "threshold": 0.8},
                "home": {"side": "left", "control": "grip", "threshold": 0.8},
                "clutch": {"side": "right", "control": "trigger", "threshold": 0.5},
            }),
        )
        self.assertEqual(publisher.source_instance_id, "run-observation")

    def test_manus_runtime_assets_are_validated_from_supplied_paths(self):
        from tianji_teleop.hand_tracking.xr_manus_observation import validate_manus_runtime
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rawviz = root / "manus" / "rawviz.out"
            rawviz.parent.mkdir()
            rawviz.touch()
            rawviz.chmod(0o755)
            calibration = rawviz.parent / "calibration"
            calibration.mkdir()
            for suffix in ("LeftMetaglovePro.mcal", "RightMetaglovePro.mcal"):
                (calibration / f"gjy{suffix}").touch()
            library = rawviz.parent / "ManusSDK" / "lib"
            library.mkdir(parents=True)
            (library / "libManusSDK_Integrated.so").touch()
            result = validate_manus_runtime({
                "rawviz": str(rawviz), "user": "gjy", "library_dir": None,
            })
            self.assertEqual(result["rawviz"], rawviz)
            self.assertEqual(result["library"], library / "libManusSDK_Integrated.so")
            with self.assertRaisesRegex(ValueError, "rawviz"):
                validate_manus_runtime({"rawviz": None, "user": "gjy", "library_dir": None})

    def test_manus_process_uses_rawviz_directory_as_working_directory(self):
        from unittest.mock import patch

        from tianji_teleop.hand_tracking.xr_manus_observation import _make_manus_process

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "manus"
            rawviz = root / "rawviz.out"
            rawviz.parent.mkdir(parents=True)
            rawviz.touch()
            rawviz.chmod(0o755)
            calibration = root / "calibration"
            calibration.mkdir()
            for suffix in ("LeftMetaglovePro.mcal", "RightMetaglovePro.mcal"):
                (calibration / f"gjy{suffix}").touch()
            library = root / "ManusSDK" / "lib"
            library.mkdir(parents=True)
            (library / "libManusSDK_Integrated.so").touch()

            with patch(
                "tianji_teleop.hand_tracking.xr_manus_observation.ReferenceManusProcess"
            ) as process:
                _make_manus_process(
                    {"rawviz": str(rawviz), "user": "gjy", "library_dir": None,
                     "callback_capacity": 32},
                    receiver_instance_id="xr-observation-manus",
                    raw_line_sink=lambda _text, _timestamp: None,
                )

            self.assertEqual(process.call_args.kwargs["cwd"], str(rawviz.parent))


if __name__ == "__main__":
    unittest.main()
