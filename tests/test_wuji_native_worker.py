import importlib.util
import json
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional official hand environment and data')
class WujiNativeWorkerTest(unittest.TestCase):
    def test_ready_includes_lazy_rotation_dependency_without_fake_callbacks(self):
        program = '''
import io, json, runpy, sys
namespace = runpy.run_path('scripts/wuji_hand_worker.py')
bridge_type = namespace['OfficialWujiHand2Bridge']
def forbidden(*args, **kwargs):
    raise AssertionError('startup must not advance retarget/filter state')
bridge_type.retarget = forbidden
original = sys.stdout
class ReadyOutput(io.StringIO):
    def write(self, text):
        if 'wuji_worker_ready' in text:
            assert 'scipy.spatial.transform' in sys.modules, 'rotation dependency still lazy at ready'
        return super().write(text)
sys.stdout = output = ReadyOutput()
sys.stdin = io.TextIOWrapper(io.BytesIO(b''))
sys.argv = ['wuji_hand_worker.py', '--startup-handshake']
namespace['main']()
sys.stdout = original
print(output.getvalue(), end='')
'''
        result = subprocess.run([str(ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python'),
            '-c', program], cwd=ROOT, text=True, capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['callbacks'], 0)

    def run_worker(self, payload):
        worker = ROOT / 'scripts/wuji_hand_worker.py'
        self.assertTrue(worker.is_file(), 'missing isolated official hand worker')
        return subprocess.run([str(ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python'),
                               str(worker)], input=payload, text=True, capture_output=True, timeout=30)

    def test_original_first_frame_and_sequence_gap_reset(self):
        # Decode trusted NumPy recording in its pinned environment, never use
        # unrestricted pickle.load in a device-facing worker.
        python = ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python'
        program = """
import json, sys
sys.path.insert(0, 'scripts')
from reference_manus_trace import RecordingUnpickler
with open('vendor/reference_inputs/remote-192.168.110.210-20260908/manus_gjy_dual_20260827.pkl', 'rb') as f:
    row = RecordingUnpickler(f).load()[0]
print(json.dumps(row['right_fingers'].reshape(-1).tolist() + row['left_fingers'].reshape(-1).tolist()))
"""
        points = json.loads(subprocess.check_output([str(python), '-c', program], cwd=ROOT, text=True))
        payload = ''.join(json.dumps(dict(schema_version=1, kind='wuji_hand_input',
            callback_sequence=sequence, timestamp_ns=sequence, points=points)) + '\n' for sequence in (1, 3))
        result = self.run_worker(payload)
        self.assertEqual(result.returncode, 0, result.stderr)
        output = [json.loads(line) for line in result.stdout.splitlines()]
        baseline = json.loads((ROOT / 'vendor/reference_inputs/remote-192.168.110.210-20260908/original_hand_trace.json').read_text())
        self.assertEqual(len(output), 2)
        for sequence, row in zip((1, 3), output):
            self.assertEqual(row['callback_sequence'], sequence)
            self.assertEqual(row['algorithm'], 'official_wuji_hand2')
            self.assertEqual(row['left']['joint_names'] + row['right']['joint_names'], baseline['joint_names'])
            self.assertEqual(row['left']['position_rad'] + row['right']['position_rad'], baseline['position_rad'][0])

    def test_bad_input_emits_no_joint_command(self):
        result = self.run_worker('{"kind":"wuji_hand_input","points":[NaN]}\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')
