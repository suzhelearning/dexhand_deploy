"""Offline endpoint checks; this does not enable live hand command authority."""
from pathlib import Path
import json
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("c++") and (Path(sys.prefix)/"include/mujoco/mujoco.h").is_file(),
                     "C++ and MuJoCo headers required")
class NativeHandMujocoEndpointTest(unittest.TestCase):
    def test_canonical_bilateral_update_hold_atomic_rejection_and_owner(self):
        import mujoco
        import numpy as np
        from tianji_teleop.protocol.messages import HAND_JOINT_NAMES
        with tempfile.TemporaryDirectory(prefix="native-hand-endpoint-") as folder:
            binary = Path(folder)/"endpoint"
            result = subprocess.run([
                "c++", "-std=c++17", "-pthread", "-Wall", "-Wextra", "-Werror", "-O2",
                "-I"+str(Path(sys.prefix)/"include"),
                str(ROOT/"tests/cpp/native_hand_mujoco_endpoint.cpp"),
                "-L"+str(Path(sys.prefix)/"lib"), "-Wl,-rpath,"+str(Path(sys.prefix)/"lib"),
                "-lmujoco", "-o", str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            for name in ("spark", "mapped_palm"):
                with self.subTest(model=name):
                    path = ROOT/f"src/tianji_teleop/assets/{name}/marvin_m6_wuji2.xml"
                    result = subprocess.run([str(binary), str(path)], capture_output=True, text=True, timeout=15)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    row = json.loads(result.stdout)
                    np.testing.assert_array_equal(row["arms"], [[.1]*7, [-.2]*7])
                    np.testing.assert_array_equal(row["hands"], [[.6]*20, [.01*(21+j) for j in range(20)]])
                    model = mujoco.MjModel.from_xml_path(str(path))
                    # Ensure the native interface's canonical binding matches the protocol.
                    for side in ("left", "right"):
                        self.assertEqual(len(HAND_JOINT_NAMES[side]), 20)
                        for joint in HAND_JOINT_NAMES[side]:
                            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint) < 0:
                                joint = joint.replace("_mcp_", "_finger_mcp_").replace("_pip", "_finger_pip").replace("_dip", "_finger_dip")
                            self.assertGreaterEqual(model.joint(joint).id, 0)
