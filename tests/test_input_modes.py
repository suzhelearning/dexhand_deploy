"""Input contracts reject mixed devices before runtime dependencies are loaded."""
import unittest

from tianji_teleop.hand_tracking import input_modes


class InputModesTest(unittest.TestCase):
    def pico(self, **overrides):
        return dict(input_mode="pico2_hands", hand_input="pico2",
                    arm_input="pico2_head_wrist", operator_input="keyboard", **overrides)

    def vr(self):
        return dict(input_mode="vr_manus", hand_input="manus",
                    arm_input="tjvr_corrected_palm", operator_input="controller")

    def test_pico_selects_only_pico_receiver(self):
        result = input_modes.resolve_input_mode(self.pico())
        self.assertEqual(result.receivers, ("pico2",))
        self.assertFalse(result.requires_upper_limb_skeleton)

    def test_vr_selects_manus_and_tjvr(self):
        result = input_modes.resolve_input_mode(self.vr())
        self.assertEqual(result.receivers, ("manus", "tjvr"))
        self.assertTrue(result.requires_upper_limb_skeleton)

    def test_mixed_skeleton_rejected(self):
        value = self.pico()
        value["hand_input"] = "manus"
        with self.assertRaisesRegex(ValueError, "hand_input"):
            input_modes.resolve_input_mode(value)

    def test_unknown_fields_and_modes_rejected(self):
        for value in (self.pico(other_source="manus"), self.pico() | {"input_mode": ["pico2_hands", "vr_manus"]}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                input_modes.resolve_input_mode(value)

    def test_missing_field_not_inferred(self):
        value = self.vr()
        del value["arm_input"]
        with self.assertRaisesRegex(ValueError, "missing"):
            input_modes.resolve_input_mode(value)

    def test_controller_and_tracker_are_distinct_sources(self):
        for source in ("xr_controller", "xr_tracker"):
            value = self.vr() | {"arm_input": source}
            result = input_modes.resolve_input_mode(value)
            self.assertEqual(result.arm_input, source)
            self.assertEqual(result.receivers, ("manus", "xr"))
            self.assertFalse(result.requires_upper_limb_skeleton)

    def test_vr_gesture_not_silently_enabled(self):
        with self.assertRaisesRegex(ValueError, "operator_input"):
            input_modes.resolve_input_mode(self.vr() | {"operator_input": "gesture"})

    def test_pico_controller_not_silently_enabled(self):
        with self.assertRaisesRegex(ValueError, "operator_input"):
            input_modes.resolve_input_mode(self.pico() | {"operator_input": "controller"})

    def test_spark_rejects_pose_only_input(self):
        mode = input_modes.resolve_input_mode(self.vr() | {"arm_input": "xr_controller"})
        with self.assertRaisesRegex(ValueError, "upper.limb"):
            input_modes.validate_ik_input(mode, input_modes.SPARK_BACKEND)

    def test_spark_accepts_corrected_skeleton_without_claiming_runtime_ready(self):
        mode = input_modes.resolve_input_mode(self.vr())
        input_modes.validate_ik_input(mode, input_modes.SPARK_BACKEND)
        self.assertNotIn("runtime_ready", mode.to_dict())

    def test_unknown_backend_rejected(self):
        mode = input_modes.resolve_input_mode(self.pico())
        with self.assertRaisesRegex(ValueError, "backend"):
            input_modes.validate_ik_input(mode, "spark_typo")

    def test_serialized_config_is_detached_from_result(self):
        mode = input_modes.resolve_input_mode(self.vr())
        output = mode.to_dict()
        output["receivers"].append("pico2")
        self.assertEqual(mode.receivers, ("manus", "tjvr"))


if __name__ == "__main__":
    unittest.main()
