"""Isolated reference-compatible TJVR protocol adapter.

Matches pico_teleop_protocol.cpp at c022b177 for the VR+Manus route.
The historical decoder and its serialized schema remain unchanged. Stream
ordering, discontinuity detection and operator event edges belong downstream;
this adapter only decodes one datagram, retaining the complete legacy/raw frame.
"""
from dataclasses import dataclass
import struct

import numpy as np

from .legacy_pico import parse_legacy_pico_packet
from .models import LegacyPicoPalmFrame


@dataclass(frozen=True)
class ReferenceTjvrFrame:
    frame: LegacyPicoPalmFrame
    left_arm_direction: np.ndarray
    right_arm_direction: np.ndarray
    left_arm_direction_valid: bool
    right_arm_direction_valid: bool
    user_button_pressed: bool


def _direction(packet: bytes, offset: int, declared_valid: bool) -> tuple[np.ndarray, bool]:
    if not declared_valid:
        return np.zeros(3), False
    direction = np.array(struct.unpack_from('<3d', packet, offset))
    if not np.isfinite(direction).all() or np.linalg.norm(direction) <= 1e-9:
        return np.zeros(3), False
    # Keep the reference's normalization semantics; do not substitute an elbow
    # vector reconstructed from the skeleton for this independent optional field.
    return direction / np.linalg.norm(direction), True


def parse_reference_tjvr_packet(
    packet: bytes, *, receiver_instance_id: str,
    receiver_frame_sequence: int, received_timestamp_ns: int,
) -> ReferenceTjvrFrame:
    frame = parse_legacy_pico_packet(
        packet, receiver_instance_id=receiver_instance_id,
        receiver_frame_sequence=receiver_frame_sequence,
        received_timestamp_ns=received_timestamp_ns)
    # Validate the raw quaternion, before the legacy normalizer can hide errors.
    offsets = [68, 124]
    if frame.upper_limb_rotations_valid:
        offsets.extend(396 + index * 32 for index in range(8))
    for offset in offsets:
        norm = np.linalg.norm(struct.unpack_from('<4d', frame.raw_packet, offset))
        if not np.isfinite(norm) or abs(norm - 1.0) > 1e-3:
            raise ValueError(f'TJVR reference quaternion at offset {offset} is not unit length')
    left, left_valid = _direction(frame.raw_packet, 156,
        frame.protocol_version >= 2 and bool(frame.flags & (1 << 4)))
    right, right_valid = _direction(frame.raw_packet, 180,
        frame.protocol_version >= 2 and bool(frame.flags & (1 << 5)))
    return ReferenceTjvrFrame(
        frame=frame, left_arm_direction=left, right_arm_direction=right,
        left_arm_direction_valid=left_valid, right_arm_direction_valid=right_valid,
        user_button_pressed=frame.protocol_version == 4 and bool(frame.flags & (1 << 8)))
