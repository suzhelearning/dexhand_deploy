"""Decoder for the historical ``TJVR`` PICO arm/palm datagrams.

This is the input used by the Manus + PICO arm experiment in
``TJ_arm_control``.  It is intentionally separate from the PICO_2 TCP
protocol: both products use PICO data, but their wire formats and reference
frames are not interchangeable.
"""
from __future__ import annotations

import struct
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from .models import ArmInputObservation, LegacyPicoPalmFrame


MAGIC = b"TJVR"
PACKET_SIZES = {1: 160, 2: 208, 3: 400, 4: 656}
REQUIRED_FLAGS = 0x0F
SKELETON_VALID_FLAG = 1 << 6
ROTATIONS_VALID_FLAG = 1 << 7
BUTTON_FLAG = 1 << 8
LEFT_HAND_POINT = 3
RIGHT_HAND_POINT = 7


def _crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def _pose(packet: bytes, offset: int) -> np.ndarray:
    values = struct.unpack_from("<7d", packet, offset)
    pose = np.asarray((values[0], values[1], values[2], values[3], values[4], values[5], values[6]), dtype=np.float64)
    if not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1.0e-12:
        raise ValueError("TJVR pose is non-finite or has a zero quaternion")
    pose[3:] /= np.linalg.norm(pose[3:])
    return pose


def _quaternions(packet: bytes, offset: int) -> np.ndarray:
    result = np.empty((8, 4), dtype=np.float64)
    for index in range(8):
        values = np.asarray(struct.unpack_from("<4d", packet, offset + index * 32), dtype=np.float64)
        if not np.isfinite(values).all() or np.linalg.norm(values) < 1.0e-12:
            raise ValueError(f"TJVR upper-limb quaternion {index} is invalid")
        result[index] = values / np.linalg.norm(values)
    return result


def parse_legacy_pico_packet(
    packet: bytes,
    *,
    receiver_instance_id: str,
    receiver_frame_sequence: int,
    received_timestamp_ns: int,
) -> LegacyPicoPalmFrame:
    if not isinstance(packet, (bytes, bytearray)) or len(packet) < 8:
        raise ValueError("TJVR packet is shorter than its header")
    packet = bytes(packet)
    if packet[:4] != MAGIC:
        raise ValueError("TJVR packet has an invalid magic")
    version, declared_size = struct.unpack_from("<HH", packet, 4)
    if version not in PACKET_SIZES:
        raise ValueError(f"unsupported TJVR protocol version: {version}")
    expected_size = PACKET_SIZES[version]
    if declared_size != expected_size or len(packet) != expected_size:
        raise ValueError("TJVR packet size does not match protocol version")
    flags = struct.unpack_from("<I", packet, 40)[0]
    known_flags = {1: 0x0F, 2: 0x3F, 3: 0x7F, 4: 0x1FF}[version]
    if flags & REQUIRED_FLAGS != REQUIRED_FLAGS or flags & ~known_flags:
        raise ValueError("TJVR packet contains invalid flags")
    if version == 1 and flags != REQUIRED_FLAGS:
        raise ValueError("TJVR v1 cannot contain optional flags")
    if version == 2 and flags & SKELETON_VALID_FLAG:
        raise ValueError("TJVR v2 cannot contain an upper-limb skeleton")
    if version < 4 and flags & ROTATIONS_VALID_FLAG:
        raise ValueError("TJVR upper-limb rotations require TJVR v4")
    if flags & ROTATIONS_VALID_FLAG and not flags & SKELETON_VALID_FLAG:
        raise ValueError("TJVR rotations require a valid upper-limb skeleton")
    expected_crc = struct.unpack_from("<I", packet, expected_size - 4)[0]
    if expected_crc != _crc32(packet[:-4]):
        raise ValueError("TJVR packet CRC mismatch")

    sequence, tracking_epoch = struct.unpack_from("<QQ", packet, 8)
    source_timestamp_ns, bridge_send_monotonic_ns = struct.unpack_from("<qq", packet, 24)
    if sequence == 0 or tracking_epoch == 0 or source_timestamp_ns <= 0 or bridge_send_monotonic_ns <= 0:
        raise ValueError("TJVR packet metadata must be positive")
    left_pose = _pose(packet, 44)
    right_pose = _pose(packet, 100)
    points = np.zeros((8, 3), dtype=np.float64)
    if flags & SKELETON_VALID_FLAG:
        for index in range(8):
            points[index] = struct.unpack_from("<3d", packet, 204 + index * 24)
        if not np.isfinite(points).all():
            raise ValueError("TJVR upper-limb points are non-finite")
    rotations = np.tile(np.asarray([0.0, 0.0, 0.0, 1.0]), (8, 1))
    if flags & ROTATIONS_VALID_FLAG:
        rotations = _quaternions(packet, 396)
    return LegacyPicoPalmFrame(
        protocol_version=version,
        packet_size=expected_size,
        flags=flags,
        sequence=sequence,
        tracking_epoch=tracking_epoch,
        source_timestamp_ns=source_timestamp_ns,
        bridge_send_monotonic_ns=bridge_send_monotonic_ns,
        left_pose=left_pose,
        right_pose=right_pose,
        upper_limb_skeleton_valid=bool(flags & SKELETON_VALID_FLAG),
        upper_limb_rotations_valid=bool(flags & ROTATIONS_VALID_FLAG),
        upper_limb_points=points,
        upper_limb_rotations_xyzw=rotations,
        raw_packet=packet,
        received_timestamp_ns=received_timestamp_ns,
        receiver_instance_id=receiver_instance_id,
        receiver_frame_sequence=receiver_frame_sequence,
    )


def _basis(side: str) -> np.ndarray:
    if side == "left":
        return np.column_stack(([-0.0, -1.0, -0.0], [-0.0, -0.0, -1.0], [1.0, 0.0, 0.0]))
    if side == "right":
        return np.column_stack(([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]))
    raise ValueError("side must be left or right")


def select_legacy_palm_pose(frame: LegacyPicoPalmFrame, side: str, *, use_corrected_skeleton: bool = True) -> np.ndarray:
    if not isinstance(frame, LegacyPicoPalmFrame):
        raise TypeError("frame must be LegacyPicoPalmFrame")
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    if use_corrected_skeleton:
        if not frame.upper_limb_skeleton_valid or not frame.upper_limb_rotations_valid:
            raise ValueError("TJVR frame has no corrected upper-limb palm pose")
        point_index = LEFT_HAND_POINT if side == "left" else RIGHT_HAND_POINT
        rotation = Rotation.from_quat(frame.upper_limb_rotations_xyzw[point_index]) * Rotation.from_matrix(_basis(side))
        return np.concatenate((frame.upper_limb_points[point_index], rotation.as_quat()))
    return frame.left_pose.copy() if side == "left" else frame.right_pose.copy()


def legacy_palm_observation(
    frame: LegacyPicoPalmFrame,
    side: str,
    *,
    use_corrected_skeleton: bool = True,
) -> ArmInputObservation:
    pose = select_legacy_palm_pose(frame, side, use_corrected_skeleton=use_corrected_skeleton)
    return ArmInputObservation(
        source="legacy_pico_palm",
        side=side,
        tracked_frame="palm",
        reference_frame="legacy_pico_tracking",
        pose=pose,
        valid=True,
        source_timestamp_ns=frame.source_timestamp_ns,
        received_timestamp_ns=frame.received_timestamp_ns,
        receiver_instance_id=frame.receiver_instance_id,
        receiver_frame_sequence=frame.receiver_frame_sequence,
        mapping_version=("legacy_pico_corrected_palm_v1" if use_corrected_skeleton else "legacy_pico_pose_v1"),
        frame_association_id=frame.association_id,
        source_sequence=frame.sequence,
        source_instance_id=frame.receiver_instance_id,
    )


__all__ = [
    "BUTTON_FLAG",
    "LEFT_HAND_POINT",
    "MAGIC",
    "PACKET_SIZES",
    "RIGHT_HAND_POINT",
    "ROTATIONS_VALID_FLAG",
    "SKELETON_VALID_FLAG",
    "legacy_palm_observation",
    "parse_legacy_pico_packet",
    "select_legacy_palm_pose",
]
