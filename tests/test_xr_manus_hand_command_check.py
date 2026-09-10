from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from tianji_teleop.hand_tracking.reference_manus import HandInputAssembler, RawvizHandInputProcessor
from tianji_teleop.hand_tracking.reference_manus_process import ManusCallback
from tianji_teleop.protocol import topics
from tianji_teleop.protocol.messages import HAND_JOINT_NAMES, HandJointCommand, HandTargetCommand
from tianji_teleop.recording.live_capture import LiveCapture
from tianji_teleop.recording.session_h5 import SessionH5Writer
from tests.test_reference_manus_process import rawviz_records


ROOT = Path(__file__).resolve().parents[1]


class _Backend:
    def __init__(self, side):
        self.side = side
        self.sequences = []
        self.closed = False

    def retarget(self, points, *, sequence, timestamp_ns):
        del points, timestamp_ns
        self.sequences.append(sequence)
        return {
            side: {
                "valid": side == self.side,
                "joint_names": list(HAND_JOINT_NAMES[side]),
                "position_rad": [sequence / 100.0] * 20,
            }
            for side in ("left", "right")
        }

    def close(self):
        self.closed = True


class XrManusHandCommandCheckTest(unittest.TestCase):
    def _recording(self, path):
        contract = dict(
            version=1,
            sides=["right", "left"],
            right_glove=None,
            left_glove=None,
            callback_order="right_then_left",
            callback_trigger="each_accepted_pose",
        )
        metadata = {
            "resolved_configuration": {
                "manus_input_contract": contract,
                "asset_sha256": {"test-asset": "digest"},
            }
        }
        with SessionH5Writer(
            path,
            source_type="vr_manus_xr_sim",
            robot_model="marvin",
            router_zid="router",
            schema_version="1.2",
            metadata=metadata,
        ) as writer:
            class Sink:
                def append(self, method, *args, **kwargs):
                    getattr(writer, method)(*args, **kwargs)

            capture = LiveCapture(Sink(), run_id="run-1")
            now = 1000
            callback_sequence = 0

            def publish(frame):
                nonlocal callback_sequence
                callback_sequence += 1
                capture.callback(
                    ManusCallback(
                        "manus",
                        callback_sequence,
                        now,
                        tuple(frame.values.tolist()),
                        frame.sequences,
                        frame.source_timestamps_ns,
                    )
                )

            processor = RawvizHandInputProcessor(
                HandInputAssembler(True, True), publish
            )
            records = "".join(
                rawviz_records(side, seq)
                for side, seq in (("right", 1), ("left", 1), ("right", 2), ("left", 2))
            )
            for line in records.splitlines():
                now += 1
                capture.rawviz(line, now)
                processor.process_line(line)

            for side in ("left", "right"):
                target = HandTargetCommand(
                    1,
                    100,
                    3000,
                    2000,
                    "hand_tracking_target",
                    side,
                    "wrist_relative_mediapipe",
                    np.zeros((21, 3), dtype=np.float64).tolist(),
                    "target-publisher",
                    "router",
                )
                writer.append_hand_target(target, received_time_ns=3000)
                for command_sequence in (10, 11):
                    command = HandJointCommand(
                        1,
                        command_sequence,
                        4000 + command_sequence,
                        f"wuji_retarget_{side}",
                        side,
                        list(HAND_JOINT_NAMES[side]),
                        [1.0] * 20,
                        "hand-producer",
                        "router",
                    )
                    writer.append_hand_command(command, received_time_ns=4000 + command_sequence)
                    writer.append_dual_audit(
                        "hand_output",
                        {
                            "schema_version": 1,
                            "kind": "hand_output",
                            "router_zid": "router",
                            "run_id": "run-1",
                            "executor_instance_id": f"executor-{side}",
                            "side": side,
                            "target": target.to_dict(),
                            "command": command.to_dict(),
                        },
                        received_timestamp_ns=4000 + command_sequence,
                    )

    def test_replays_each_target_once_and_checks_every_executor_command(self):
        from tianji_teleop.recording.xr_manus_hand_command_check import check_xr_manus_hand_commands

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.h5"
            self._recording(path)
            clients = []

            def factory(**kwargs):
                client = _Backend(kwargs["single_hand_side"])
                clients.append(client)
                return client

            with patch(
                "tianji_teleop.recording.xr_manus_hand_command_check.hand_replay_asset_hashes",
                return_value={"test-asset": "digest"},
            ):
                report = check_xr_manus_hand_commands(path, root=ROOT, backend_factory=factory)

            self.assertTrue(report["passed"], report)
            self.assertEqual(report["replayed_targets"], 2)
            self.assertEqual(report["matched_commands"], 4)
            self.assertEqual([client.sequences for client in clients], [[100], [100]])
            self.assertTrue(all(client.closed for client in clients))


if __name__ == "__main__":
    unittest.main()
