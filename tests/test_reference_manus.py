import importlib.util
import hashlib
import json
from pathlib import Path
import unittest
import numpy as np

from tests.test_manus_hand_tracking import _payload
from tianji_teleop.hand_tracking.manus import manus_to_mediapipe, parse_manus_payload


class ReferenceManusTest(unittest.TestCase):
    def module(self):
        name = 'tianji_teleop.hand_tracking.reference_manus'
        self.assertIsNotNone(importlib.util.find_spec(name), 'missing reference Manus input boundary')
        from tianji_teleop.hand_tracking import reference_manus
        return reference_manus

    def test_float32_y_flip_without_legacy_wrist_recentering(self):
        ref = self.module()
        payload = _payload()
        points = ref.resolve_wuji2_keypoints(payload['node_semantics'], payload['nodes'])
        self.assertEqual(points.dtype, np.dtype('float32'))
        np.testing.assert_array_equal(points[0], [1, -2, 3])
        raw = parse_manus_payload(payload, receiver_instance_id='test',
                                  receiver_frame_sequence=1, received_timestamp_ns=1)
        legacy = manus_to_mediapipe(raw)
        np.testing.assert_array_equal(legacy.keypoints_m[0], [0, 0, 0])
        np.testing.assert_array_equal(points - points[0], legacy.keypoints_m)

    def test_asynchronous_latest_sides_keep_original_callback_order(self):
        ref = self.module()
        assembler = ref.HandInputAssembler()
        right, left = np.ones((21, 3)), np.ones((21, 3)) * 2
        assembler.update('right', right, 10, 100)
        self.assertIsNone(assembler.take_latest())
        assembler.update('left', left, 20, 200)
        first = assembler.take_latest()
        np.testing.assert_array_equal(first.values, np.concatenate([right.ravel(), left.ravel()]))
        self.assertEqual(first.sequences, {'right': 10, 'left': 20})
        self.assertIsNone(assembler.take_latest())
        assembler.update('right', right * 3, 11, 300)
        self.assertEqual(assembler.take_latest().sequences, {'right': 11, 'left': 20})
        assembler.update('right', right, 10, 400)
        self.assertIsNone(assembler.take_latest())
        assembler.invalidate('left')
        assembler.update('right', right, 12, 500)
        self.assertIsNone(assembler.take_latest())

    def test_incomplete_semantics_rejected(self):
        ref = self.module()
        payload = _payload()
        with self.assertRaises(ValueError):
            ref.resolve_wuji2_keypoints(payload['node_semantics'][:-1], payload['nodes'])

    def test_pinned_source_diff_is_only_relative_import(self):
        ref = self.module()
        root = Path(ref.__file__).parent
        manifest = json.loads((root / 'source_manifest.json').read_text())
        for entry in manifest['files']:
            data = (root / entry['destination']).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), entry['sha256'])
            original = data.replace(b'from .wuji2_hand_input import (', b'from wuji2_hand_input import (')
            self.assertEqual(hashlib.sha256(original).hexdigest(), entry['source_sha256'])
