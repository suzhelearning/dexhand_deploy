"""Read TJVT recording containers without changing TJVR packet consumption.

This is an offline reader, not a source scheduler: every recorded packet is
preserved. A controller must separately reproduce its latest-frame policy.
The caller must exhaust the iterator to validate the complete recording.
"""
from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import BinaryIO, Iterator
import zlib

from .legacy_pico import PACKET_SIZES


_HEADER = struct.Struct('<4sHHQ')
_TIME = struct.Struct('<q')


@dataclass(frozen=True)
class TjvrRecord:
    relative_receive_ns: int
    packet: bytes


def iter_tjvr_records(stream: BinaryIO) -> Iterator[TjvrRecord]:
    """Validate container framing, packet header/CRC and receive-time order.

    Semantic pose/flag validation belongs to the selected packet decoder.
    Duplicate source sequences/timestamps are deliberately not filtered here.
    """
    header = stream.read(_HEADER.size)
    if len(header) != _HEADER.size:
        raise ValueError('truncated TJVT header')
    magic, version, packet_size, count = _HEADER.unpack(header)
    if magic != b'TJVT' or version != 1 or packet_size not in PACKET_SIZES.values():
        raise ValueError('invalid TJVT header')
    previous_time = 0
    for index in range(count):
        timestamp = stream.read(_TIME.size)
        packet = stream.read(packet_size)
        if len(timestamp) != _TIME.size or len(packet) != packet_size:
            raise ValueError(f'truncated TJVT record {index}')
        relative_ns, = _TIME.unpack(timestamp)
        if relative_ns < previous_time:
            raise ValueError(f'negative/backward receive time at record {index}')
        packet_magic, packet_version, declared_size = struct.unpack_from('<4sHH', packet)
        if (packet_magic != b'TJVR' or declared_size != packet_size or
                PACKET_SIZES.get(packet_version) != packet_size):
            raise ValueError(f'invalid TJVR packet header at record {index}')
        expected_crc, = struct.unpack_from('<I', packet, packet_size - 4)
        if zlib.crc32(packet[:-4]) != expected_crc:
            raise ValueError(f'TJVR CRC mismatch at record {index}')
        previous_time = relative_ns
        yield TjvrRecord(relative_ns, packet)
    if stream.read(1):
        raise ValueError('trailing bytes after TJVT records')
