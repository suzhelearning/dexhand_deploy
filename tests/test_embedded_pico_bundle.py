import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / "vendor" / "pico_tracker"
GENERATED_PARTS = {
    ".pixi",
    ".pytest_cache",
    "__pycache__",
    "build",
    "install",
    "log",
    "recordings",
    "runtime",
}
REQUIRED = (
    ".gitignore",
    "pixi.toml",
    "pixi.lock",
    "src/imu_ros2/package.xml",
    "src/imu_ros2/CMakeLists.txt",
    "src/imu_ros2/src/imu_multi_node.cpp",
    "src/pico_bridge/package.xml",
    "src/pico_bridge/CMakeLists.txt",
    "src/pico_bridge/src/pico_bridge_node.cpp",
    "src/pico_bridge/src/pico_smpl_ground_node.cpp",
    "src/pico_bridge/src/tianji_mujoco_teleop_bridge_node.cpp",
    "src/pico_bridge/launch/start_pico_bridge.launch.py",
    "src/pico_bridge/launch/start_pico_palm_skeleton_filter.launch.py",
    "src/pico_bridge/launch/start_tianji_mujoco_teleop.launch.py",
    "scripts/calibrate_pico_arm.sh",
    "scripts/calibrate_pico_palm_orientation.sh",
    "scripts/record_pico_tremor.sh",
    "scripts/start_pico_driver.sh",
    "scripts/start_pico_m0.sh",
    "scripts/cleanup_tianji_pico_processes.py",
)


class EmbeddedPicoBundleTest(unittest.TestCase):
    def test_complete_runtime_source_set_is_embedded(self):
        missing = [relative for relative in REQUIRED if not (BUNDLE / relative).is_file()]
        self.assertEqual(missing, [], f"missing embedded PICO files: {missing}")

    def test_source_manifest_is_relative_and_hashed(self):
        manifest = json.loads((BUNDLE / "source_manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["schema_version"], 1)
        self.assertTrue(manifest["source_commit"])
        self.assertGreater(len(manifest["files"]), 20)
        paths = []
        for entry in manifest["files"]:
            self.assertEqual(set(entry), {"path", "sha256"})
            path = Path(entry["path"])
            self.assertFalse(path.is_absolute())
            self.assertNotIn("..", path.parts)
            self.assertRegex(entry["sha256"], r"[0-9a-f]{64}")
            target = BUNDLE / path
            self.assertTrue(target.is_file(), entry["path"])
            self.assertEqual(hashlib.sha256(target.read_bytes()).hexdigest(), entry["sha256"])
            paths.append(entry["path"])
        self.assertEqual(paths, sorted(paths))

    def test_nested_manifest_has_locked_pico_build_task(self):
        manifest = (BUNDLE / "pixi.toml").read_text(encoding="utf-8")
        self.assertIn("build-pico-bridge", manifest)
        self.assertIn("--packages-select imu_ros2 pico_bridge", manifest)
        self.assertTrue((BUNDLE / "pixi.lock").is_file())

    def test_embedded_text_has_no_developer_checkout_path(self):
        if not BUNDLE.is_dir():
            return
        for path in BUNDLE.rglob("*"):
            relative = path.relative_to(BUNDLE)
            if (
                not path.is_file()
                or path.name == "pixi.lock"
                or any(part in GENERATED_PARTS for part in relative.parts)
                or path.suffix in {".pyc", ".pyo"}
            ):
                continue
            try:
                content = path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            self.assertNotIn("/home/zj/current_robotics/", content, str(path))

    def test_root_build_task_is_scoped_to_embedded_manifest(self):
        root_manifest = (ROOT / "pixi.toml").read_text(encoding="utf-8")
        build_script = (ROOT / "scripts" / "build_embedded_pico.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("build-embedded-pico", root_manifest)
        self.assertIn("--manifest-path vendor/pico_tracker/pixi.toml", build_script)
        self.assertIn("build-pico-bridge", build_script)
        self.assertNotIn("PICO_TRACKER_ROOT", build_script)

    def test_root_generated_runtime_is_ignored(self):
        gitignore = (ROOT / ".gitignore").read_text(encoding="utf-8")
        for pattern in ("/install/", "/recordings/", "/MUJOCO_LOG.TXT"):
            self.assertIn(pattern, gitignore)
