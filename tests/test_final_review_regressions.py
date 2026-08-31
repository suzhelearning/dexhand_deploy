from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from pico_body_tianji.coordination.arm_command_coordinator import ArmCommandCoordinator
from pico_body_tianji.protocol.messages import HAND_JOINT_NAMES, HandExecutorStatus, HandJointState


class FinalReviewRegressionTest(unittest.TestCase):
    def test_hand_teleop_health_requires_tracking_and_matching_state(self) -> None:
        coordinator = object.__new__(ArmCommandCoordinator)
        coordinator._state = mock.Mock(state="teleop")
        coordinator.profile = {}
        coordinator._hand_status = {"right": mock.Mock(value=HandExecutorStatus(
            1, 1, 100, "right", True, True, False, False, None, "hand-i", "router"
        ))}
        coordinator._hand_state = {"right": mock.Mock(value=HandJointState(
            1, 1, 100, "executor", "right", list(HAND_JOINT_NAMES["right"]),
            [0.0] * 20, None, "hand-i", "router"
        ))}
        coordinator._profile_hand_sides = lambda: ("right",)
        coordinator._fresh = lambda value, now: True
        self.assertFalse(coordinator._hand_tracking_fresh(100))
        coordinator._hand_status["right"].value = HandExecutorStatus(
            1, 2, 100, "right", True, True, True, False, None, "hand-i", "router"
        )
        self.assertTrue(coordinator._hand_tracking_fresh(100))

    def test_analyzer_global_protocol_drops_are_a_gate(self) -> None:
        path = Path(__file__).parents[1] / "scripts" / "validation" / "analyze_runs.py"
        spec = importlib.util.spec_from_file_location("analyze_runs_final_review", path)
        assert spec and spec.loader
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        manifest = {
            "case_id": "pico_sim", "profile": "pico_sim", "required_capability": "simulation",
            "authority_contract": [],
        }
        case = {"velocity_ratio": 1.0}
        metrics = {"protocol_drops": 1, "protocol_order_errors": 0, "rates": {}}
        with self.assertRaises(module.AnalysisError):
            module._validate_pass_gate(Path(tempfile.mkdtemp()), manifest, case, [], [], [], metrics, [], {"outcome": "fail"})
    def test_protocol_sequence_folds_cross_topic_pair_but_rejects_duplicate_topic(self) -> None:
        from scripts.validation.analyze_runs import _folded_protocol_sequence_metric

        folded = _folded_protocol_sequence_metric([
            ("instance", 1, 1, "status"),
            ("instance", 1, 1, "proposal"),
            ("instance", 2, 2, "status"),
        ])
        self.assertEqual(folded, {"drops": 0, "order_errors": 0})
        duplicate = _folded_protocol_sequence_metric([
            ("instance", 1, 1, "status"),
            ("instance", 2, 1, "status"),
        ])
        self.assertEqual(duplicate["order_errors"], 1)

    def test_replay_router_is_provenance_not_admission(self) -> None:
        from pico_body_tianji.recording import replay
        source = Path(replay.__file__).read_text(encoding="utf-8")
        self.assertNotIn('recording router_zid does not match replay router', source)
    def test_replay_cli_drives_authorized_start_and_control_ticks(self) -> None:
        from pico_body_tianji.recording import replay_cli

        node = mock.Mock()
        node.phase = "replaying"

        def tick() -> None:
            node.phase = "armed"
            node._return_requested = True

        node.tick.side_effect = tick
        with mock.patch.dict("os.environ", {
            "TIANJI_COMPONENT_INSTANCE_ID": "source",
            "TIANJI_COORDINATOR_INSTANCE_ID": "coordinator",
        }, clear=False), mock.patch.object(replay_cli, "open_session", return_value=mock.Mock()), mock.patch.object(
            replay_cli, "require_single_router", return_value="router"
        ), mock.patch.object(replay_cli, "TargetReplaySource", return_value=node):
            self.assertEqual(replay_cli.main(["target", "recording.h5", "--headless"]), 0)
        node.request_start.assert_called_once_with()
        node.tick.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
