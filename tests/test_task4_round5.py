from __future__ import annotations

import json
from pathlib import Path
import unittest

from tianji_teleop.protocol.messages import ProtocolError, strict_loads


class ProducerInitializationContractTest(unittest.TestCase):
    def test_ready_and_liveliness_are_announced_after_all_io_declarations(self) -> None:
        source = (
            Path(__file__).parents[1]
            / "src/tianji_teleop/src/producers/arm_ik_producer_node.cpp"
        ).read_text(encoding="utf-8")
        publisher = source.index('status_publisher_ = session_.declare_publisher')
        target_subscriber = source.index('target_subscribers_[index] = session_.declare_subscriber')
        command_subscriber = source.index('command_subscribers_[index] = session_.declare_subscriber')
        liveliness = source.index('liveliness_token_ = session_.liveliness_declare_token')
        ready = source.index('publish_status("")')
        self.assertLess(publisher, target_subscriber)
        self.assertLess(target_subscriber, command_subscriber)
        self.assertLess(command_subscriber, liveliness)
        self.assertLess(liveliness, ready)


class StrictProtocolFixtureTest(unittest.TestCase):
    def test_duplicate_keys_are_rejected_before_schema_parser(self) -> None:
        with self.assertRaises(ProtocolError):
            strict_loads('{"schema_version":1,"schema_version":1}')

    def test_nonfinite_json_constants_are_rejected(self) -> None:
        with self.assertRaises(ProtocolError):
            strict_loads('{"position_m":[NaN,0,0]}')

    def test_valid_arrays_are_preserved(self) -> None:
        value = strict_loads(json.dumps({"position_m": [0.1, 0.2, 0.3]}))
        self.assertEqual(value["position_m"], [0.1, 0.2, 0.3])


    def test_python_arm_target_is_accepted_by_cpp_fixture(self) -> None:
        payload = {
            "schema_version": 1,
            "publisher_instance_id": "source-1",
            "router_zid": "router-1",
            "sequence": 7,
            "timestamp_ns": 1,
            "source_timestamp_ns": None,
            "source": "mocap_live",
            "side": "right",
            "frame_id": "Base_R",
            "position_m": [0.1, 0.2, 0.3],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "elbow_reference_direction": [1.0, 0.0, 0.0],
        }
        fixture = Path(__file__).parents[1] / "build/ik/protocol_cpp_fixture"
        result = __import__("subprocess").run(
            [str(fixture), "target", "router-1", "right"],
            input=json.dumps(payload),
            text=True,
            capture_output=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_cpp_fixture_emissions_are_accepted_by_python_protocol(self) -> None:
        from tianji_teleop.protocol.messages import ArmJointProposal, ArmSolvedPose, ComponentStatus
        import subprocess

        fixture = Path(__file__).parents[1] / "build/ik/protocol_cpp_fixture"
        for mode, parser in (
            ("proposal", ArmJointProposal),
            ("solved", ArmSolvedPose),
            ("status", ComponentStatus),
        ):
            result = subprocess.run([str(fixture), "emit-" + mode], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            parser.from_dict(strict_loads(result.stdout))

    def test_python_arm_command_is_accepted_by_cpp_fixture(self) -> None:
        from tianji_teleop.protocol.messages import ArmJointCommand
        import subprocess

        command = ArmJointCommand(
            schema_version=1,
            sequence=10,
            timestamp_ns=1,
            producer="coordinator",
            side="left",
            mode="teleop",
            proposal_sequence=8,
            target_sequence=7,
            names=[f"Joint{i}_L" for i in range(1, 8)],
            position_rad=[0.0] * 7,
            publisher_instance_id="coord-1",
            router_zid="router-1",
        )
        fixture = Path(__file__).parents[1] / "build/ik/protocol_cpp_fixture"
        result = subprocess.run(
            [str(fixture), "command", "router-1", "left"],
            input=json.dumps(command.to_dict()),
            text=True,
            capture_output=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
