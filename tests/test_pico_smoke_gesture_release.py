import unittest

from scripts import pico_sim_smoke


class PicoSmokeGestureReleaseTest(unittest.TestCase):
    def rows(self):
        return [dict(receiver_frame_sequence=i, received_timestamp_ns=1000 + i * 10,
            receiver_instance_id='pico', connection_generation=1,
            hands={side: dict(available=True, gesture='fist') for side in ('left', 'right')})
            for i in range(3)]

    def test_requires_observed_release_not_elapsed_wall_time(self):
        predicate = pico_sim_smoke.observed_gesture_release
        rows = self.rows()
        self.assertTrue(predicate(rows, since_ns=999, now_ns=1030))
        self.assertFalse(predicate(rows[:1], since_ns=999, now_ns=1030))
        self.assertFalse(predicate(rows, since_ns=1030, now_ns=1040))
        self.assertFalse(predicate(rows, since_ns=999, now_ns=300_000_000))
        rows[-1]['hands']['left']['gesture'] = 'open'
        self.assertFalse(predicate(rows, since_ns=999, now_ns=1030))

    def test_reconnect_or_sequence_gap_cannot_ack_release(self):
        for field, value in (('connection_generation', 2), ('receiver_frame_sequence', 7)):
            with self.subTest(field=field):
                rows = self.rows()
                rows[-1][field] = value
                self.assertFalse(pico_sim_smoke.observed_gesture_release(
                    rows, since_ns=999, now_ns=1030))
