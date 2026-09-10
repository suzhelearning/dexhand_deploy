import io
import struct
import unittest

from tests.test_legacy_pico_palm import _packet
from tianji_teleop.hand_tracking.tjvr_trace import iter_tjvr_records


def trace(packets, times=None):
    times = times if times is not None else list(range(len(packets)))
    return (struct.pack('<4sHHQ', b'TJVT', 1, 656, len(packets)) +
            b''.join(struct.pack('<q', t) + p for t, p in zip(times, packets)))


class TjvrTraceTest(unittest.TestCase):
    def test_preserves_bytes_order_and_receiver_time(self):
        packets = [_packet(), _packet(flags=0x1CF)]
        rows = list(iter_tjvr_records(io.BytesIO(trace(packets, [0, 125]))))
        self.assertEqual([r.packet for r in rows], packets)
        self.assertEqual([r.relative_receive_ns for r in rows], [0, 125])

    def test_duplicate_timestamps_preserved(self):
        self.assertEqual(len(list(iter_tjvr_records(io.BytesIO(trace([_packet()]*2, [0, 0]))))), 2)

    def test_short_header_rejected(self):
        with self.assertRaisesRegex(ValueError, 'header'):
            list(iter_tjvr_records(io.BytesIO(b'TJVT')))

    def test_invalid_version_and_size_rejected(self):
        for version, size in [(2, 656), (1, 655)]:
            with self.subTest(version=version, size=size), self.assertRaises(ValueError):
                list(iter_tjvr_records(io.BytesIO(struct.pack('<4sHHQ', b'TJVT', version, size, 0))))

    def test_truncated_record_and_trailing_bytes_rejected(self):
        value = trace([_packet()])
        for damaged in (value[:-1], value+b'x'):
            with self.subTest(length=len(damaged)), self.assertRaises(ValueError):
                list(iter_tjvr_records(io.BytesIO(damaged)))

    def test_corrupt_crc_rejected(self):
        p = bytearray(_packet())
        p[99] ^= 1
        with self.assertRaisesRegex(ValueError, 'CRC'):
            list(iter_tjvr_records(io.BytesIO(trace([p]))))

    def test_backward_or_negative_receive_time_rejected(self):
        for times in ([0, -1], [2, 1]):
            with self.subTest(times=times), self.assertRaisesRegex(ValueError, 'time'):
                list(iter_tjvr_records(io.BytesIO(trace([_packet()]*2, times))))

    def test_packet_header_must_match_container(self):
        p = bytearray(_packet())
        struct.pack_into('<H', p, 6, 400)
        with self.assertRaisesRegex(ValueError, 'packet header'):
            list(iter_tjvr_records(io.BytesIO(trace([p]))))


if __name__ == '__main__':
    unittest.main()
