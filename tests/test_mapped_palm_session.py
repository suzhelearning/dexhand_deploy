from copy import deepcopy
from pathlib import Path
import unittest
import yaml
from tianji_teleop.hand_tracking.session_config import resolve_session

ROOT = Path(__file__).resolve().parents[1]
NAME = 'pico_ee_mapped_corrected_palm_velocity_qp'

class MappedPalmSessionTest(unittest.TestCase):
    def test_explicit_backend_leaves_default_spark_unchanged(self):
        source = yaml.safe_load((ROOT / 'src/tianji_teleop/config/sessions/pico_vr_manus_sim.yaml').read_text())
        default = resolve_session(source)
        mapped = deepcopy(source)
        mapped['ik_backend'] = NAME
        resolved = resolve_session(mapped)
        self.assertEqual(resolved['retarget_owner'], 'mapped_palm')
        self.assertEqual(resolved['hand_retarget_backend'], default['hand_retarget_backend'])
        self.assertEqual(resolve_session(source), default)

    def test_bare_hand_input_rejected(self):
        source = yaml.safe_load((ROOT / 'src/tianji_teleop/config/sessions/pico2_hands_sim.yaml').read_text())
        source['ik_backend'] = NAME
        with self.assertRaises(ValueError):
            resolve_session(source)
