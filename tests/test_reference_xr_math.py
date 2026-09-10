import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'src/tianji_teleop/tianji_teleop/hand_tracking/reference_xr'
REFERENCE = Path(os.environ.get('XR_REFERENCE_ROOT', ''))

# Same poses and call order in two isolated interpreters; no SDK/ROS or robot.
TRACE = '''
import json
import numpy as np
from scipy.spatial.transform import Rotation
config = TianjiConfig.load(use_ros=False)
controller = IncrementalController(config, rate=90., min_cutoff=1., beta=.7, elbow_min_cutoff=.3)
roles = ('pico_left_wrist','pico_right_wrist','pico_left_arm','pico_right_arm')
initial = {role: [.1,-.2,.3,0.,0.,0.,1.] for role in roles}
result = []
for segment in range(2):
 controller.initialize(initial, [.1,.2,.3,0.,0.,0.,1.])
 for i in range(80):
  t = i / 79.
  quat = Rotation.from_euler('xyz', [t*.4,-t*.3,t*.6]).as_quat()
  pose = np.r_[np.array([.1,-.2,.3]) + np.array([.2,-.1,.3])*t, quat]
  for role in roles:
   position, orientation = controller.compute_target_pose(pose, role)
   result.append([position.tolist(), orientation.tolist()])
  for side in ('left','right'):
   wrist = np.array([.5,0.,0.]) if i != 20 else np.zeros(3)
   elbow = np.array([.2,.001,.001]) if i % 20 == 0 else np.array([.2,.3*np.sin(t),.2])
   direction, projected = controller.compute_elbow_direction(np.zeros(3), wrist, elbow, side)
   result.append([direction.tolist(), projected.tolist()])
  result.append(controller.compute_hmd_world_pose(pose).tolist())
 controller.reset()
 assert controller.compute_target_pose(initial[roles[0]], roles[0]) == (None, None)
print(json.dumps(result, allow_nan=False))
'''


class ReferenceXrMathTest(unittest.TestCase):
    def test_source_closure_has_only_declared_import_rewrites(self):
        self.assertTrue((DEST / 'source_manifest.json').is_file())
        manifest = json.loads((DEST / 'source_manifest.json').read_text())
        self.assertEqual(manifest['source_commit'], '59028470b8be4abfdd052ff35a06ca73413d9590')
        for entry in manifest['files']:
            data = (DEST / entry['destination']).read_text()
            for old, new in reversed(entry.get('replacements', [])):
                data = data.replace(new, old)
            self.assertEqual(hashlib.sha256(data.encode()).hexdigest(), entry['source_sha256'], entry['destination'])

    @unittest.skipUnless(os.environ.get('XR_REFERENCE_TEST'), 'optional independent XR source oracle')
    def test_increment_filter_elbow_and_reset_match_reference_exactly(self):
        self.assertTrue(DEST.is_dir(), 'XR math closure missing')
        self.assertIsNotNone(importlib.util.find_spec('tianji_teleop.hand_tracking.reference_xr.incremental_controller'))
        reference_imports = (
            'import sys\n'
            f'sys.path[:0] = {[str(REFERENCE / "src/input_devices/pico_input"), str(REFERENCE / "src/output_devices/tianji_world_output")]!r}\n'
            'from pico_input.incremental_controller import IncrementalController\n'
            'from tianji_world_output.config_loader import TianjiConfig\n')
        migrated_imports = (
            'from tianji_teleop.hand_tracking.reference_xr.incremental_controller import IncrementalController\n'
            'from tianji_teleop.hand_tracking.reference_xr.config_loader import TianjiConfig\n')
        outputs = []
        for imports in (reference_imports, migrated_imports):
            result = subprocess.run([sys.executable, '-c', imports + TRACE],
                                     capture_output=True, text=True, timeout=20, check=True)
            outputs.append(json.loads(result.stdout))
        self.assertEqual(outputs[0], outputs[1])
