"""Reference protocol semantics, independent of the legacy runtime decoder."""
import struct
import unittest
import zlib

import numpy as np

from tests.test_legacy_pico_palm import _packet
from tianji_teleop.hand_tracking import reference_tjvr
from tianji_teleop.hand_tracking.legacy_pico import parse_legacy_pico_packet


def changed(packet, offset, values):
    result = bytearray(packet)
    struct.pack_into('<' + 'd' * len(values), result, offset, *values)
    struct.pack_into('<I', result, len(result) - 4, zlib.crc32(result[:-4]))
    return bytes(result)


def decode(packet):
    return reference_tjvr.parse_reference_tjvr_packet(
        packet, receiver_instance_id='test', receiver_frame_sequence=1,
        received_timestamp_ns=100)


class ReferenceTjvrTest(unittest.TestCase):
    def test_entry_point_exists(self):
        self.assertTrue(callable(getattr(reference_tjvr, 'parse_reference_tjvr_packet', None)))

    def test_direction_normalization_and_independent_validity(self):
        packet = changed(_packet(flags=0xff), 156, [3., 4., 0.])
        result = decode(changed(packet, 180, [0., 0., 0.]))
        np.testing.assert_allclose(result.left_arm_direction, [.6, .8, 0.])
        self.assertTrue(result.left_arm_direction_valid)
        self.assertFalse(result.right_arm_direction_valid)
        np.testing.assert_array_equal(result.right_arm_direction, [0., 0., 0.])

    def test_missing_flag_ignores_nonfinite_direction(self):
        result = decode(changed(_packet(flags=0xcf), 156, [float('nan'), 0., 0.]))
        self.assertFalse(result.left_arm_direction_valid)
        np.testing.assert_array_equal(result.left_arm_direction, [0., 0., 0.])

    def test_declared_invalid_direction_does_not_reject_pose(self):
        for values in ([float('inf'), 0., 0.], [1e-10, 0., 0.]):
            with self.subTest(values=values):
                result = decode(changed(_packet(flags=0xff), 156, values))
                self.assertFalse(result.left_arm_direction_valid)
                np.testing.assert_array_equal(result.left_arm_direction, [0., 0., 0.])

    def test_button_is_state_not_an_edge(self):
        packet = _packet(flags=0x1ff)
        self.assertTrue(decode(packet).user_button_pressed)
        self.assertTrue(decode(packet).user_button_pressed)
        self.assertFalse(decode(_packet(flags=0xff)).user_button_pressed)

    def test_all_versions_and_raw_frame_retained(self):
        for version, flags in ((1, 0xf), (2, 0x3f), (3, 0x7f), (4, 0xff)):
            with self.subTest(version=version):
                packet = _packet(version=version, flags=flags)
                result = decode(packet)
                self.assertEqual(result.frame.raw_packet, packet)
                self.assertEqual(result.left_arm_direction_valid, version >= 2)
                self.assertEqual(result.frame.tracking_epoch, 11)

    def test_reference_rejects_nonunit_quaternion_without_changing_legacy(self):
        for offset in (68, 124, 396, 620):
            with self.subTest(offset=offset):
                packet = changed(_packet(), offset, [0., 0., 0., 1.01])
                with self.assertRaisesRegex(ValueError, 'quaternion'):
                    decode(packet)
                parse_legacy_pico_packet(packet, receiver_instance_id='legacy',
                    receiver_frame_sequence=1, received_timestamp_ns=100)

    def test_small_quaternion_error_normalized(self):
        result = decode(changed(_packet(), 68, [0., 0., 0., 1.0005]))
        np.testing.assert_array_equal(result.frame.left_pose[3:], [0., 0., 0., 1.])

    def test_absent_rotations_not_validated(self):
        result = decode(changed(_packet(flags=0x4f), 396, [float('nan')] * 4))
        self.assertFalse(result.frame.upper_limb_rotations_valid)
