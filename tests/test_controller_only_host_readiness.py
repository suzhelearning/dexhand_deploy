from __future__ import annotations

import unittest

from pico_body_tianji.host_readiness import HostReadinessGate


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


def _mocap_replay_status() -> dict:
    return {
        "state": "idle",
        "source": "offline_replay",
        "input": "mocap_h5_replay",
        "mapping": "controller_relative_end_pose_conditioned_v1",
        "body_tracking": "disabled",
        "motion_trackers_required": False,
        "elbow_constraint": "published_default_zsp_backend_selected",
        "smpl_used": False,
        "scope": "mocap_replay",
        "at_safe_home": True,
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

    def test_mocap_replay_host_is_ready(self) -> None:
        gate = _gate("controller_only")
        _observe_safe_host(gate, _mocap_replay_status())

        decision = gate.evaluate(now=10.02)

        self.assertTrue(decision.ready)
        self.assertEqual(decision.reason, "ready")

    def test_mocap_replay_host_rejected_without_ready_fields(self) -> None:
        gate = _gate("controller_only")
        status = _mocap_replay_status()
        status["at_safe_home"] = False
        _observe_safe_host(gate, status)

        decision = gate.evaluate(now=10.02)

        self.assertFalse(decision.ready)
        self.assertEqual(decision.reason, "mocap_replay_not_ready")

    def test_mocap_replay_host_rejected_when_not_idle(self) -> None:
        gate = _gate("controller_only")
        status = _mocap_replay_status()
        status["state"] = "teleop"
        _observe_safe_host(gate, status)

        decision = gate.evaluate(now=10.02)

        self.assertFalse(decision.ready)
        self.assertEqual(decision.reason, "host_not_idle")


if __name__ == "__main__":
    unittest.main()
