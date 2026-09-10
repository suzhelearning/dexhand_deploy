from dataclasses import replace
import unittest

from scipy.spatial.transform import Rotation

from tests.test_legacy_pico_palm import _packet
from tests.test_reference_tjvr import decode
from tianji_teleop.hand_tracking import reference_tjvr_stream as stream


def frame(sequence, epoch=9, x=0., angle=0.):
    result = decode(_packet()).frame
    pose = result.left_pose.copy()
    pose[0] += x
    pose[3:] = Rotation.from_rotvec([angle, 0., 0.]).as_quat()
    return replace(result, sequence=sequence, tracking_epoch=epoch, left_pose=pose)


class ReferenceStreamTest(unittest.TestCase):
    def test_entry_point_exists(self):
        self.assertTrue(callable(getattr(stream, 'ReferenceTjvrStreamGate', None)))

    def gate(self):
        return stream.ReferenceTjvrStreamGate(.15, .6)

    def test_first_duplicate_epoch_and_reset(self):
        gate = self.gate()
        self.assertTrue(gate.evaluate(frame(10)).epoch_changed)
        self.assertEqual(gate.evaluate(frame(10)).reason, 'out_of_order')
        self.assertTrue(gate.evaluate(frame(1, epoch=10, x=5.)).epoch_changed)
        self.assertEqual(gate.evaluate(frame(11)).reason, 'epoch_rollback')
        self.assertEqual(gate.evaluate(frame(12, epoch=0)).reason, 'zero_epoch')
        gate.reset()
        self.assertTrue(gate.evaluate(frame(1, epoch=1)).epoch_changed)

    def test_three_stable_jump_frames_and_rejected_sequence(self):
        gate = self.gate()
        gate.evaluate(frame(1))
        self.assertEqual(gate.evaluate(frame(3, x=.30)).reason, 'position_jump')
        self.assertEqual(gate.evaluate(frame(2, x=.31)).reason, 'out_of_order')
        self.assertFalse(gate.evaluate(frame(4, x=.31)).accepted)
        result = gate.evaluate(frame(5, x=.32))
        self.assertTrue(result.accepted)
        self.assertTrue(result.stream_discontinuity)
        self.assertFalse(result.epoch_changed)

    def test_candidate_restart_and_anchor_recovery(self):
        gate = self.gate()
        gate.evaluate(frame(1))
        for seq, x in ((2, .3), (3, .6), (4, .61)):
            self.assertFalse(gate.evaluate(frame(seq, x=x)).accepted)
        self.assertTrue(gate.evaluate(frame(5, x=.62)).stream_discontinuity)
        self.assertFalse(gate.evaluate(frame(6, x=1.)).accepted)
        result = gate.evaluate(frame(7, x=.63))
        self.assertTrue(result.accepted)
        self.assertFalse(result.stream_discontinuity)
        self.assertFalse(gate.evaluate(frame(8, x=1.)).accepted)
        self.assertFalse(gate.evaluate(frame(9, x=1.)).accepted)

    def test_orientation_jump_and_epoch_clears_candidate(self):
        gate = self.gate()
        gate.evaluate(frame(1))
        self.assertEqual(gate.evaluate(frame(2, angle=.601)).reason, 'orientation_jump')
        self.assertTrue(gate.evaluate(frame(1, epoch=10, angle=2.)).epoch_changed)
        result = gate.evaluate(frame(2, epoch=10, angle=2.01))
        self.assertTrue(result.accepted)
        self.assertFalse(result.stream_discontinuity)

    def test_invalid_thresholds(self):
        for value in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                stream.ReferenceTjvrStreamGate(value, .6)
            with self.assertRaises(ValueError):
                stream.ReferenceTjvrStreamGate(.15, value)
