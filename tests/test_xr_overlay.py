import unittest

import mujoco
import numpy as np

from tianji_teleop.executors.mujoco.xr_overlay import XrRawOverlay
from tianji_teleop.hand_tracking.xr_input import XrControllerState, XrFrame, XrTrackerState


def _frame(sequence=1, receiver="xr-receiver"):
    controllers = {
        "left": XrControllerState(
            "left", [0.1, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0], True, True, 0.0, 0.2, [0.0, 0.0]
        ),
        "right": XrControllerState(
            "right", [0.2, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0], True, True, 0.0, 0.3, [0.0, 0.0]
        ),
    }
    trackers = tuple(
        XrTrackerState(
            serial,
            [position, 0.1, 0.5, 0.0, 0.0, 0.0, 1.0],
            True,
            side,
        )
        for serial, position, side in (
            ("190058", 0.3, "left"),
            ("190046", 0.4, None),
            ("190600", 0.5, "right"),
            ("190023", 0.6, None),
        )
    )
    return XrFrame(
        sequence=sequence,
        source_timestamp_ns=sequence,
        received_timestamp_ns=1000 + sequence,
        hmd_pose=[0.0, 0.0, 1.6, 0.0, 0.0, 0.0, 1.0],
        controllers=controllers,
        trackers=trackers,
        receiver_instance_id=receiver,
    )


class XrOverlayTest(unittest.TestCase):
    def test_overlay_accepts_only_matching_fresh_ordered_xr_frames(self):
        overlay = XrRawOverlay("router")
        frame = _frame()
        self.assertTrue(overlay.ingest({**frame.to_dict(), "router_zid": "router"}, 1001))
        self.assertFalse(overlay.ingest({**frame.to_dict(), "router_zid": "foreign"}, 1002))
        self.assertFalse(overlay.ingest({**frame.to_dict(), "router_zid": "router"}, 1002))
        stale = _frame(sequence=2)
        self.assertFalse(overlay.ingest({**stale.to_dict(), "router_zid": "router"}, 2_000_000_000))
        current, diagnostics = overlay.snapshot(1001)
        self.assertIsNotNone(current)
        self.assertEqual(current.to_dict(), frame.to_dict())
        self.assertEqual(diagnostics["state"], "live")
        self.assertEqual(diagnostics["frames_received"], 1)
        self.assertIsNotNone(diagnostics["error"])

    def test_overlay_renders_hmd_controllers_and_all_trackers_passively(self):
        overlay = XrRawOverlay("router")
        frame = _frame()
        overlay.ingest({**frame.to_dict(), "router_zid": "router"}, 1001)
        markers, bones = overlay.geometry(1001)
        labels = {item["label"] for item in markers}
        self.assertIn("XR HMD", labels)
        self.assertIn("XR left controller", labels)
        self.assertIn("XR right controller", labels)
        self.assertEqual(sum(label.startswith("XR tracker") for label in labels), 4)
        self.assertEqual(bones, [])

        model = mujoco.MjModel.from_xml_string("<mujoco/>")
        data = mujoco.MjData(model)
        before = data.qpos.copy()
        scene = mujoco.MjvScene(model, 3)
        overlay.append(scene, mujoco, 1001)
        self.assertEqual(scene.ngeom, 3)
        np.testing.assert_array_equal(data.qpos, before)


if __name__ == "__main__":
    unittest.main()
