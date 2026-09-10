import hashlib
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'src/tianji_teleop/assets'


class SparkAssetsTest(unittest.TestCase):
    def test_reference_model_and_shared_mesh_closure(self):
        model = ASSETS / 'spark/marvin_m6_wuji2.xml'
        self.assertTrue(model.is_file(), 'SPARK reference model missing')
        # Only the relative mesh directory may differ from the pinned model.
        original = model.read_bytes().replace(b'meshdir="../tianji_wuji2"', b'meshdir="tianji_wuji2"')
        self.assertEqual(hashlib.sha256(original).hexdigest(),
            'dcb3040c7cb5b5d1d897c56f9e5834df0c682d179919baa6a1fef60ce782b4ae')
        tree = ET.fromstring(model.read_text())
        meshdir = model.parent / tree.find('compiler').attrib['meshdir']
        for mesh in tree.findall('asset/mesh'):
            self.assertTrue((meshdir / mesh.attrib['file']).is_file())
        self.assertTrue((model.parent / 'marvin_m6_s_ccs_696_v4_local.urdf').is_file())
