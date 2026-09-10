import unittest
from scripts import pico_sim_smoke as smoke
from tianji_teleop.hand_tracking.pico import parse_pico_packet
from tianji_teleop.hand_tracking.official_pico import pico_official_hand_observations


class PicoOfficialSmokeTest(unittest.TestCase):
    def test_gesture_fixture_classifies_open_and_release_without_changing_arm_pose(self):
        from tianji_teleop.hand_tracking.runtime import pico_frame_observations
        from tianji_teleop.hand_tracking.gesture_recognition import classify_hand
        self.assertTrue(hasattr(smoke, 'gesture_packet'))
        for opened, expected in ((True, 'open'), (False, 'fist')):
            frame = parse_pico_packet(smoke.gesture_packet(.015, 0., opened),
                receiver_instance_id='pico', connection_generation=1,
                receiver_frame_sequence=0, received_timestamp_ns=1000)
            for hand, _ in pico_frame_observations(frame).values():
                self.assertEqual(classify_hand(hand.keypoints_m, valid=hand.valid).gesture, expected)
            self.assertTrue(all(hand.valid for hand in pico_official_hand_observations(frame).values()))
            self.assertAlmostEqual(frame.hands['left'].wrist_pose[0], .315)

    def test_new_smoke_geometry_is_valid_for_both_official_hands(self):
        self.assertTrue(hasattr(smoke, 'official_packet'))
        for d, b in ((0., 0.), (.015, .01)):
            frame = parse_pico_packet(smoke.official_packet(d, b), receiver_instance_id='pico',
                connection_generation=1, receiver_frame_sequence=0, received_timestamp_ns=1000)
            self.assertTrue(all(hand.valid for hand in pico_official_hand_observations(frame).values()))
            self.assertAlmostEqual(frame.hands['left'].wrist_pose[0], .3+d)
