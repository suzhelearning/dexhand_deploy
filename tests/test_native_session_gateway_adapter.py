import struct
import unittest


class NativeGatewayWireTest(unittest.TestCase):
    def test_no_new_input_tick_accepts_zero_source_metadata(self):
        from tianji_teleop.producers.spark.native_gateway import decode_cycle_payload, CYCLE_PREFIX_STRUCT
        payload = bytearray(self._cycle_payload(packet=b''))
        offset = CYCLE_PREFIX_STRUCT.size + 28 * 8
        struct.pack_into('<QqqQQB7xI', payload, offset, 4, 10_000, 0, 0, 0, 0, 0)
        result = decode_cycle_payload(payload, prefix='spark')
        self.assertEqual(result['request']['packet'], b'')
        self.assertEqual(result['request']['received_ns'], 0)
        for field_offset, fmt, value in ((16, '<q', -1), (16, '<q', 9000),
                                          (24, '<Q', 1), (32, '<Q', 1), (40, '<B', 1)):
            invalid = bytearray(payload)
            struct.pack_into(fmt, invalid, offset + field_offset, value)
            with self.assertRaises(ValueError):
                decode_cycle_payload(invalid, prefix='spark')

    def test_nonempty_request_still_requires_positive_receive_time(self):
        from tianji_teleop.producers.spark.native_gateway import decode_cycle_payload, CYCLE_PREFIX_STRUCT
        payload = bytearray(self._cycle_payload())
        struct.pack_into('<q', payload, CYCLE_PREFIX_STRUCT.size + 28 * 8 + 16, 0)
        with self.assertRaises(ValueError):
            decode_cycle_payload(payload, prefix='spark')

    def _cycle_payload(self, *, flags=0xF7, packet=b"TJVR-test"):
        values = [float(index) for index in range(28)]
        payload = bytearray()
        payload += struct.pack(
            "<qqQQQQqQQBBBB",
            10_000,
            9_000,
            7,
            11,
            13,
            2,
            3,
            17,
            1,
            2,
            flags,
            0,
            0,
        )
        payload += struct.pack("<28d", *values)
        payload += struct.pack("<QQQQQB7xI", 4, 10_000, 9_000, 11, 23, 0, len(packet))
        payload += packet + bytes(656 - len(packet))
        payload += bytes(8 * 4 + 4 + 1206)
        reason = b"teleop"
        payload += struct.pack("<I", len(reason)) + reason
        return bytes(payload)

    def test_cycle_decodes_fixed_layout_and_rejects_trailing_bytes(self):
        from tianji_teleop.producers.spark.native_gateway import decode_cycle_payload

        row = decode_cycle_payload(self._cycle_payload(), prefix="spark")
        self.assertEqual(row["timestamp_ns"], 10_000)
        self.assertEqual(row["state"], "teleop")
        self.assertEqual(row["source"], {
            "received_ns": 9_000,
            "revision": 7,
            "epoch": 11,
            "sequence": 13,
            "generation": 2,
            "accepted": True,
            "skeleton_valid": True,
            "rotations_valid": True,
        })
        self.assertEqual(row["request"]["packet"], b"TJVR-test")
        self.assertEqual(row["request"]["source_sequence"], 23)
        self.assertTrue(row["command"])
        with self.assertRaisesRegex(ValueError, "trailing"):
            decode_cycle_payload(self._cycle_payload() + b"x", prefix="spark")

    def test_raw_frame_decodes_the_valid_packet_before_stream_gate(self):
        from tianji_teleop.producers.spark.native_gateway import decode_gateway_frame

        packet = b"decoded-before-gate"
        payload = struct.pack("<QqB3xI", 41, 12_345, 0, len(packet))
        payload += packet + bytes(656 - len(packet))
        frame = struct.pack("<4sBBHIQq", b"TJSO", 1, 6, 0, len(payload), 41, 12_345) + payload
        decoded = decode_gateway_frame(frame, prefix="spark")
        self.assertEqual(decoded.kind, "raw")
        self.assertEqual(decoded.payload, {
            "sequence": 41,
            "received_ns": 12_345,
            "accepted": False,
            "packet": packet,
        })

    def test_command_encoding_is_exact_and_validates_identity(self):
        from tianji_teleop.producers.spark.native_gateway import encode_gateway_command

        self.assertEqual(
            encode_gateway_command("start", 9, 0),
            struct.pack("<4sBBH Q q", b"TJAC", 1, 1, 0, 9, 0),
        )
        with self.assertRaisesRegex(ValueError, "action"):
            encode_gateway_command("unknown", 9, 0)
        with self.assertRaisesRegex(ValueError, "id"):
            encode_gateway_command("start", 0, 0)

    def test_native_cycle_is_reconstructed_into_existing_typed_snapshot(self):
        from tianji_teleop.producers.spark.native_live_runner import _typed_cycle
        from tianji_teleop.protocol.messages import ArmJointState, ComponentStatus, SessionState

        cycle = {
            "timestamp_ns": 10_000,
            "source": {"received_ns": 9_000, "revision": 7, "epoch": 11,
                        "sequence": 13, "generation": 2, "accepted": True,
                        "skeleton_valid": True, "rotations_valid": True},
            "state": "idle", "state_epoch": 1, "ticks": 17, "late_ticks": 2,
            "reset_pending": False, "capture_failed": False,
            "command": {"left": [1.0] * 7, "right": [-1.0] * 7},
            "feedback": {"left": [1.0] * 7, "right": [-1.0] * 7},
            "request": None, "result": None, "ik_adopted": False, "reason": "Home",
        }
        manifest = {
            "run_id": "run", "algorithm": "spark_upper_qpoases_headroom_feedforward_velocity_qp",
            "model": "/portable/model.xml",
            "home": [[1.0] * 7, [-1.0] * 7],
            "source_authority": {"logical": "tjvr", "instance": "source", "router": "router"},
            "producer_authority": {"logical": "ik_spark_headroom", "instance": "producer", "router": "router"},
            "coordinator_authority": {"logical": "arm", "instance": "coordinator", "router": "router"},
            "executor_authority": {"logical": "mujoco", "instance": "executor", "router": "router"},
        }
        snapshot = _typed_cycle(cycle, manifest=manifest, prefix="spark", sample=None)
        self.assertIsInstance(snapshot.arm_state, ArmJointState)
        self.assertIsInstance(snapshot.source_status, ComponentStatus)
        self.assertIsInstance(snapshot.session_state, SessionState)
        self.assertEqual(snapshot.arm_state.position_rad, [1.0] * 7 + [-1.0] * 7)
        self.assertEqual(snapshot.result.commands["left"].mode, "idle")
        self.assertIsNone(snapshot.result.native_result)
        self.assertEqual(snapshot.execution_epoch, 1)


if __name__ == "__main__":
    unittest.main()
