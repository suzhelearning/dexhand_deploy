from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

import h5py
import mujoco
import numpy as np

from pico_body_tianji.regrind_h5 import (
    RegrindQposLayout,
    apply_regrind_frame,
    load_regrind_h5,
)


JOINT_NAMES = (
    "r_thumb_cmc_flex",
    "r_thumb_cmc_abd",
    "r_thumb_mcp",
    "r_thumb_ip",
    "r_index_finger_mcp_flex",
    "r_index_finger_mcp_abd",
    "r_index_finger_pip",
    "r_index_finger_dip",
    "r_middle_finger_mcp_flex",
    "r_middle_finger_mcp_abd",
    "r_middle_finger_pip",
    "r_middle_finger_dip",
    "r_ring_finger_mcp_flex",
    "r_ring_finger_mcp_abd",
    "r_ring_finger_pip",
    "r_ring_finger_dip",
    "r_pinky_mcp_flex",
    "r_pinky_mcp_abd",
    "r_pinky_pip",
    "r_pinky_dip",
)
FRAME_DESCRIPTION = (
    "table centre origin, table top z=0, +x across width, "
    "+y along length, +z up"
)


def _write_recording(path: Path, *, quat_convention: str = "wxyz") -> None:
    frames = 3
    joints = np.arange(frames * len(JOINT_NAMES), dtype=np.float64).reshape(
        frames, len(JOINT_NAMES)
    ) / 100.0
    root_position = np.array(
        [[-0.4, -0.2, 0.2], [-0.3, -0.1, 0.3], [-0.2, 0.0, 0.4]]
    )
    object_position = np.array(
        [[-0.1, 0.2, 0.01], [-0.1, 0.1, 0.01], [-0.1, 0.0, 0.01]]
    )
    quaternions = np.tile([1.0, 0.0, 0.0, 0.0], (frames, 1))
    with h5py.File(path, "w") as h5:
        h5.attrs["frames"] = frames
        h5.attrs["fps"] = 50.0
        h5.attrs["dropped_leading_frames"] = 1
        h5.attrs["frame"] = FRAME_DESCRIPTION
        h5.attrs["joint_order"] = ",".join(JOINT_NAMES)
        h5.attrs["quat_convention"] = quat_convention
        h5.attrs["root_link"] = "r_base"
        h5.create_dataset("regrind_retargeting_joints", data=joints)
        h5.create_dataset("regrind_retargeting_root_pos", data=root_position)
        h5.create_dataset("regrind_retargeting_root_quat", data=quaternions)
        h5.create_dataset("object_pos", data=object_position)
        h5.create_dataset("object_quat", data=quaternions)
        h5.create_dataset("wuji_retargeting_joints", data=np.full_like(joints, 99.0))


class RegrindH5Test(unittest.TestCase):
    def test_loads_only_regrind_dataset_with_declared_contract(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "take.h5"
            _write_recording(path)
            recording = load_regrind_h5(path)

        self.assertEqual(recording.frame_count, 3)
        self.assertEqual(recording.fps, 50.0)
        self.assertAlmostEqual(recording.duration_s, 0.06)
        self.assertEqual(recording.joint_names, JOINT_NAMES)
        self.assertEqual(recording.joints[0, 0], 0.0)
        self.assertNotEqual(recording.joints[0, 0], 99.0)
        self.assertEqual(recording.summary()["dataset"], "regrind_retargeting_*")

    def test_rejects_non_wxyz_quaternions(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "take.h5"
            _write_recording(path, quat_convention="xyzw")
            with self.assertRaisesRegex(ValueError, "WXYZ"):
                load_regrind_h5(path)

    def test_applies_free_roots_and_all_twenty_joints(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "take.h5"
            _write_recording(path)
            recording = load_regrind_h5(path)

        nested = ""
        closing = ""
        for index, name in enumerate(JOINT_NAMES):
            nested += (
                f'<body name="b{index}" pos="0 0 0.01">'
                f'<joint name="{name}" type="hinge" axis="0 0 1"/>'
                '<geom type="sphere" size="0.001"/>'
            )
            closing += "</body>"
        xml = (
            '<mujoco model="regrind_apply_test"><worldbody>'
            '<body name="hand"><freejoint name="hand_root"/>'
            '<geom type="sphere" size="0.001"/>'
            f"{nested}<geom type=\"sphere\" size=\"0.001\"/>{closing}</body>"
            '<body name="object"><freejoint name="object_root"/>'
            '<geom type="sphere" size="0.001"/></body>'
            "</worldbody></mujoco>"
        )
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        hand_joint = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "hand_root"
        )
        object_joint = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "object_root"
        )
        layout = RegrindQposLayout(
            hand_root_address=int(model.jnt_qposadr[hand_joint]),
            object_root_address=int(model.jnt_qposadr[object_joint]),
            joint_addresses=tuple(
                int(
                    model.jnt_qposadr[
                        mujoco.mj_name2id(
                            model, mujoco.mjtObj.mjOBJ_JOINT, name
                        )
                    ]
                )
                for name in JOINT_NAMES
            ),
        )

        apply_regrind_frame(model, data, recording, layout, 1)

        np.testing.assert_allclose(
            data.qpos[layout.hand_root_address : layout.hand_root_address + 7],
            [-0.3, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0],
        )
        np.testing.assert_allclose(
            data.qpos[
                layout.object_root_address : layout.object_root_address + 7
            ],
            [-0.1, 0.1, 0.01, 1.0, 0.0, 0.0, 0.0],
        )
        np.testing.assert_allclose(
            [data.qpos[address] for address in layout.joint_addresses],
            recording.joints[1],
        )


if __name__ == "__main__":
    unittest.main()
