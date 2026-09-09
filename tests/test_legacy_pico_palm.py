from __future__ import annotations

import struct
import unittest

import numpy as np

from tianji_teleop.hand_tracking.legacy_pico import (
    parse_legacy_pico_packet,
    legacy_palm_observation,
    select_legacy_palm_pose,
)


def _write_double(packet: bytearray, offset: int, value: float) -> None:
    struct.pack_into("<d", packet, offset, value)


def _write_pose(packet: bytearray, offset: int, index: int) -> None:
    for component, value in enumerate((float(index), float(index + 1), float(index + 2))):
        _write_double(packet, offset + component * 8, value)
    for component, value in enumerate((0.0, 0.0, 0.0, 1.0)):
        _write_double(packet, offset + 24 + component * 8, value)


def _crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def _packet(*, version: int = 4, flags: int = 0xCF) -> bytes:
    sizes = {1: 160, 2: 208, 3: 400, 4: 656}
    packet = bytearray(sizes[version])
    packet[:4] = b"TJVR"
    struct.pack_into("<HHQQqqI", packet, 4, version, len(packet), 7, 11, 1_000_000, 1_000_100, flags)
    _write_pose(packet, 44, 10)
    _write_pose(packet, 100, 20)
    if version >= 2:
        struct.pack_into("<ddd", packet, 156, 1.0, 0.0, 0.0)
        struct.pack_into("<ddd", packet, 180, 1.0, 0.0, 0.0)
    if version >= 3 and flags & (1 << 6):
        for index in range(8):
            struct.pack_into("<ddd", packet, 204 + index * 24, float(index), float(index + 1), float(index + 2))
    if version >= 4 and flags & (1 << 7):
        for index in range(8):
            struct.pack_into("<dddd", packet, 396 + index * 32, 0.0, 0.0, 0.0, 1.0)
    struct.pack_into("<I", packet, len(packet) - 4, _crc32(packet[:-4]))
    return bytes(packet)


class LegacyPicoPalmTest(unittest.TestCase):
    def test_v4_parser_retains_packet_and_selects_corrected_hand_points(self) -> None:
        packet = _packet()
        frame = parse_legacy_pico_packet(
            packet,
            receiver_instance_id="legacy-receiver",
            receiver_frame_sequence=4,
            received_timestamp_ns=100,
        )

        self.assertEqual(frame.protocol_version, 4)
        self.assertEqual(frame.tracking_epoch, 11)
        self.assertEqual(frame.raw_packet, packet)
        left = select_legacy_palm_pose(frame, "left")
        right = select_legacy_palm_pose(frame, "right")
        np.testing.assert_allclose(left[:3], [3.0, 4.0, 5.0])
        np.testing.assert_allclose(right[:3], [7.0, 8.0, 9.0])
        self.assertEqual(legacy_palm_observation(frame, "right").tracked_frame, "palm")

    def test_invalid_crc_and_missing_corrected_skeleton_are_rejected(self) -> None:
        packet = bytearray(_packet())
        packet[120] ^= 1
        with self.assertRaises(ValueError):
            parse_legacy_pico_packet(
                packet,
                receiver_instance_id="receiver",
                receiver_frame_sequence=1,
                received_timestamp_ns=1,
            )
        frame = parse_legacy_pico_packet(
            _packet(flags=0x0F),
            receiver_instance_id="receiver",
            receiver_frame_sequence=1,
            received_timestamp_ns=1,
        )
        with self.assertRaises(ValueError):
            select_legacy_palm_pose(frame, "left")


if __name__ == "__main__":
    unittest.main()
