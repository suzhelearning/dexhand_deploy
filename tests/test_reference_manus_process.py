import importlib.util
import sys
import time
import unittest

from tests.test_manus_hand_tracking import _payload


def rawviz_records(side, seq, *, canonical_points=None):
    data = _payload()
    if canonical_points is not None:
        # _payload deliberately permutes semantics. Keep this topology and
        # place the optional meter-scale geometry in the matching array slots.
        order = list(range(21))
        order = order[::2] + order[1::2]
        data['nodes'] = [list(canonical_points[index]) for index in order]
    glove = side + '-glove'
    lines = [f'HAND {glove} {side} {len(data["nodes"])}']
    for node in data['node_semantics']:
        fields = [node[k] for k in ('array_index', 'node_id', 'parent_id', 'chain_type', 'side', 'finger_joint_type')]
        lines.append('NODE ' + glove + ' ' + ' '.join(map(str, fields)))
    values = [value for point in data['nodes'] for value in point + [1, 0, 0, 0]]
    lines.append(f'POSE {glove} {seq} {seq * 1000} 0 ' + ' '.join(map(str, values)))
    return '\n'.join(lines) + '\n'


class ReferenceManusProcessTest(unittest.TestCase):
    def test_nominal_geometry_preserves_semantic_order_and_original_y_reflection(self):
        import numpy as np
        from tests.test_gesture_recognition import hand_points
        points = hand_points()
        records = (rawviz_records('right', 1, canonical_points=points) +
                   rawviz_records('left', 1, canonical_points=points))
        source = self.source(records)
        self.wait_until(lambda: source.pending_count == 1)
        actual = np.asarray(source.try_read().points).reshape(2, 21, 3)
        expected = points.astype(np.float32) * [1, -1, 1]
        for side in actual:
            np.testing.assert_array_equal(side, expected)
            self.assertLess(np.linalg.norm(side - side[0], axis=1).max(), .2)
            self.assertGreater(np.linalg.norm(np.cross(side[5], side[17])), .001)

    def source(self, records, **kwargs):
        name = 'tianji_teleop.hand_tracking.reference_manus_process'
        self.assertIsNotNone(importlib.util.find_spec(name))
        from tianji_teleop.hand_tracking.reference_manus_process import ReferenceManusProcess
        source = ReferenceManusProcess(command=[sys.executable, '-u', '-c',
            'import sys,time; sys.stdout.write(' + repr(records) + '); sys.stdout.flush(); time.sleep(10)'],
            receiver_instance_id='manus-test', **kwargs)
        self.addCleanup(source.close)
        return source

    def wait_until(self, condition):
        deadline = time.monotonic() + 3
        while not condition() and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(condition())

    def test_async_callbacks_keep_both_source_sequences(self):
        source = self.source(rawviz_records('right', 1) + rawviz_records('left', 1) + rawviz_records('right', 2))
        self.wait_until(lambda: source.pending_count == 2)
        first, second = source.try_read(), source.try_read()
        self.assertEqual(first.receiver_instance_id, 'manus-test')
        self.assertEqual(first.sequence, 1)
        self.assertEqual(second.sequence, 2)
        self.assertEqual(second.source_sequences, {'right': 2, 'left': 1})
        self.assertEqual(first.points[:3], (1., -2., 3.))
        self.assertEqual(len(first.points), 126)
        self.assertLessEqual(first.received_timestamp_ns, second.received_timestamp_ns)
        self.assertIsNone(source.try_read())

    def test_overflow_latches_failure_instead_of_silent_callback_drop(self):
        source = self.source(rawviz_records('right', 1) + rawviz_records('left', 1) + rawviz_records('right', 2), capacity=1)
        self.wait_until(lambda: source.failure is not None)
        self.assertIn('overflow', source.failure)
        self.assertIsNone(source.try_read())

    def test_oversized_line_faults_and_close_is_idempotent(self):
        source = self.source('x' * 65537)
        self.wait_until(lambda: source.failure is not None)
        self.assertIn('line', source.failure)
        source.close()
        source.close()

    def test_single_left_preserves_63_values(self):
        source = self.source(rawviz_records('left', 1), sides=('left',))
        self.wait_until(lambda: source.pending_count == 1)
        row = source.try_read()
        self.assertEqual(len(row.points), 63)
        self.assertEqual(row.source_sequences, {'left': 1})

    def test_record_sink_sees_each_original_callback_before_consumption(self):
        captured = []
        source = self.source(rawviz_records('right', 1) + rawviz_records('left', 1) + rawviz_records('right', 2),
                             callback_sink=captured.append)
        self.wait_until(lambda: source.pending_count == 2)
        self.assertEqual(captured, [source.try_read(), source.try_read()])

    def test_close_can_preserve_callbacks_already_accepted_for_shutdown_drain(self):
        source = self.source(rawviz_records('right', 1) + rawviz_records('left', 1))
        self.wait_until(lambda: source.pending_count == 1)
        source.close(clear_pending=False)
        self.assertEqual(source.pending_count, 1)
        pending = source.drain_pending()
        self.assertEqual(len(pending), 1)
        self.assertEqual(pending[0].sequence, 1)
