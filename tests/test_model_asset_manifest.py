from pathlib import Path
import tempfile
import unittest


class ModelAssetManifestTest(unittest.TestCase):
    def collect(self, model, root):
        from tianji_teleop.recording.model_assets import flat_mujoco_asset_files
        return flat_mujoco_asset_files(model, asset_root=root)

    def test_shared_mesh_paths_are_deduplicated_and_do_not_scan_unreferenced_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'models').mkdir()
            (root / 'meshes').mkdir()
            mesh = root / 'meshes/hand.stl'
            mesh.write_bytes(b'fixture')
            (root / 'meshes/unrelated.stl').write_bytes(b'not referenced')
            model = root / 'models/main.xml'
            model.write_text('<mujoco><compiler meshdir="../meshes"/><asset>'
                '<mesh name="a" file="hand.stl"/><mesh name="b" file="hand.stl"/>'
                '</asset></mujoco>')
            self.assertEqual(set(self.collect(model, root)), {model, mesh})

    def test_missing_external_or_unsupported_includes_fail_instead_of_partial_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = root / 'model.xml'
            for contents in (
                '<mujoco><asset><mesh file="missing.stl"/></asset></mujoco>',
                '<mujoco><asset><mesh file="../outside.stl"/></asset></mujoco>',
                '<mujoco><include file="other.xml"/></mujoco>',
                '<mujoco><compiler strippath="true"/></mujoco>',
            ):
                model.write_text(contents)
                with self.subTest(contents=contents), self.assertRaises(ValueError):
                    self.collect(model, root)
