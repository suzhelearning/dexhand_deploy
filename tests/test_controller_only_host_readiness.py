from __future__ import annotations

import unittest

from pico_body_tianji.host_readiness import HostReadinessGate
from pico_body_tianji.marvin_hardware_bridge import MarvinHardwareBridge


LEFT_HOME = [55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0]
RIGHT_HOME = [-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0]


def _gate(input_mode: str) -> HostReadinessGate:
    return HostReadinessGate(
        left_home_deg=LEFT_HOME,
        right_home_deg=RIGHT_HOME,
        freshness_timeout_s=1.0,
        command_timeout_s=0.2,
        maximum_pair_skew_s=0.03,
        home_tolerance_deg=1.0,
        input_mode=input_mode,
    )


def _controller_only_status() -> dict:
    return {
        "state": "idle",
        "source": "live",
        "input": "pico_controllers_only",
        "mapping": "controller_relative_end_pose_conditioned_v1",
        "body_tracking": "disabled",
        "motion_trackers_required": False,
        "elbow_constraint": "published_default_zsp_backend_selected",
        "smpl_used": False,
        "scope": "controller_only_ik",
        "at_safe_home": True,
        "error": None,
    }


def _mocap_live_status() -> dict:
    return {
        "state": "idle",
        "source": "live",
        "input": "mocap_live",
        "scope": "mocap_live",
        "mapping": "controller_relative_end_pose_conditioned_v1",
        "control_mode": "motive_reference_keyboard_step",
        "body_tracking": "disabled",
        "motion_trackers_required": True,
        "elbow_constraint": "published_default_zsp_backend_selected",
        "smpl_used": False,
        "at_safe_home": True,
        "left_rigid_id": 1,
        "right_rigid_id": 2,
        "error": None,
    }


def _mocap_step_status() -> dict:
    return {
        "state": "idle",
        "source": "offline_replay",
        "input": "mocap_keyboard_step",
        "scope": "mocap_keyboard_step",
        "mapping": "controller_relative_end_pose_conditioned_v1",
        "body_tracking": "disabled",
        "motion_trackers_required": False,
        "elbow_constraint": "published_default_zsp_backend_selected",
        "smpl_used": False,
        "at_safe_home": True,
        "step_mm": 10.0,
        "side": "right",
        "error": None,
    }


def _observe_safe_host(
    gate: HostReadinessGate,
    input_status: dict,
) -> None:
    gate.observe_input_status(input_status, received_at=10.0)
    gate.observe_sim_status(
        {
            "mode": "idle",
            "at_safe_home": True,
            "ik_interface": "arm_ik_solver_v1",
            "ik_backend": "pinocchio_cpp",
            "robot_connected": False,
            "scope": "preview_only",
        },
        received_at=10.0,
    )
    gate.observe_teleop_state("idle", received_at=10.0)
    gate.observe_command(
        "left",
        LEFT_HOME,
        frame_id="left_base_marvin_degrees",
        received_at=10.0,
    )
    gate.observe_command(
        "right",
        RIGHT_HOME,
        frame_id="right_base_marvin_degrees",
        received_at=10.01,
    )


class ControllerOnlyHostReadinessTest(unittest.TestCase):
    def test_exact_controller_only_host_is_ready(self) -> None:
        gate = _gate("controller_only")
        _observe_safe_host(gate, _controller_only_status())

        decision = gate.evaluate(now=10.02)

        self.assertTrue(decision.ready)
        self.assertEqual(decision.reason, "ready")

    def test_rejects_unknown_ik_interface(self) -> None:
        gate = _gate("controller_only")
        _observe_safe_host(gate, _controller_only_status())
        gate.observe_sim_status(
            {
                "mode": "idle",
                "at_safe_home": True,
                "ik_interface": "unknown",
                "ik_backend": "pinocchio_cpp",
                "robot_connected": False,
                "scope": "preview_only",
            },
            received_at=10.0,
        )

        decision = gate.evaluate(now=10.02)

        self.assertFalse(decision.ready)
        self.assertEqual(
            decision.reason,
            "sim_not_isolated_expected_ik",
        )

    def test_accepts_official_backend_with_same_interface(self) -> None:
        gate = _gate("controller_only")
        _observe_safe_host(gate, _controller_only_status())
        gate.observe_sim_status(
            {
                "mode": "idle",
                "at_safe_home": True,
                "ik_interface": "arm_ik_solver_v1",
                "ik_backend": "tianji_official",
                "robot_connected": False,
                "scope": "preview_only",
            },
            received_at=10.0,
        )

        decision = gate.evaluate(now=10.02)

        self.assertTrue(decision.ready)
        self.assertEqual(decision.reason, "ready")

    def test_controller_only_mode_rejects_smpl_identity(self) -> None:
        gate = _gate("controller_only")
        status = _controller_only_status()
        status.update(
            {
                "input": "pico_controllers_plus_smpl_upper_body",
                "smpl_used": True,
            }
        )
        _observe_safe_host(gate, status)

        decision = gate.evaluate(now=10.02)

        self.assertFalse(decision.ready)
        self.assertEqual(
            decision.reason,
            "pico_controller_only_not_live",
        )

    def test_controller_only_mode_rejects_tracker_requirement(self) -> None:
        gate = _gate("controller_only")
        status = _controller_only_status()
        status["motion_trackers_required"] = True
        _observe_safe_host(gate, status)

        decision = gate.evaluate(now=10.02)

        self.assertFalse(decision.ready)
        self.assertEqual(
            decision.reason,
            "pico_controller_only_not_live",
        )

    def test_smpl_mode_keeps_original_readiness_contract(self) -> None:
        gate = _gate("smpl")
        _observe_safe_host(
            gate,
            {
                "state": "idle",
                "source": "live",
                "smpl_source": "live",
                "smpl_used": True,
                "at_safe_home": True,
                "error": None,
            },
        )

        decision = gate.evaluate(now=10.02)

        self.assertTrue(decision.ready)
        self.assertEqual(decision.reason, "ready")


class MocapLiveHostReadinessTest(unittest.TestCase):
    def test_mocap_live_host_is_ready(self) -> None:
        gate = _gate("controller_only")
        _observe_safe_host(gate, _mocap_live_status())

        decision = gate.evaluate(now=10.02)

        self.assertTrue(decision.ready)
        self.assertEqual(decision.reason, "ready")

    def test_mocap_live_requires_trackers(self) -> None:
        gate = _gate("controller_only")
        status = _mocap_live_status()
        status["motion_trackers_required"] = False

        _observe_safe_host(gate, status)
        decision = gate.evaluate(now=10.02)

        self.assertFalse(decision.ready)
        self.assertEqual(decision.reason, "mocap_live_not_ready")

    def test_mocap_live_rejects_continuous_rigid_feedback_mode(self) -> None:
        gate = _gate("controller_only")
        status = _mocap_live_status()
        status["control_mode"] = "motive_continuous_follow"

        _observe_safe_host(gate, status)
        decision = gate.evaluate(now=10.02)

        self.assertFalse(decision.ready)
        self.assertEqual(decision.reason, "mocap_live_not_ready")


    def test_mocap_live_rejects_offline_source(self) -> None:
        gate = _gate("controller_only")
        status = _mocap_live_status()
        status["source"] = "offline_replay"

        _observe_safe_host(gate, status)
        decision = gate.evaluate(now=10.02)

        self.assertFalse(decision.ready)
        self.assertEqual(decision.reason, "mocap_live_not_ready")


class _InputStatusCapture:
    def __init__(self) -> None:
        self.calls: list[tuple[str, float]] = []

    def observe_input_status(self, text: str, *, received_at: float) -> None:
        self.calls.append((text, received_at))


class MarvinBridgeStateAuthorityTest(unittest.TestCase):
    def test_status_state_cannot_override_dedicated_teleop_state(self) -> None:
        bridge = MarvinHardwareBridge.__new__(MarvinHardwareBridge)
        bridge._readiness = _InputStatusCapture()
        bridge._last_error = None
        observed_states: list[str] = []
        bridge._observe_state = (
            lambda state, received_at: observed_states.append(state)
        )

        bridge._on_input_status('{"state":"idle","input":"mocap_live"}')
        self.assertEqual(len(bridge._readiness.calls), 1)
        self.assertEqual(observed_states, [])
        self.assertIsNone(bridge._last_error)

        bridge._on_teleop_state("teleop")
        self.assertEqual(observed_states, ["teleop"])


if __name__ == "__main__":
    unittest.main()
