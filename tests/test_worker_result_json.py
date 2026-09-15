"""Native audit encoding must retain every existing Python diagnostic field."""
import json
from pathlib import Path
import random
import struct
import subprocess
import tempfile
import unittest

from tianji_teleop.hand_tracking.native_binary_results import _SPECS, _STRUCTS, decode_result

ROOT = Path(__file__).resolve().parents[1]


class WorkerResultJsonTest(unittest.TestCase):
    def test_full_audit_schema_matches_python(self):
        header = ROOT / 'native/control/worker_result_json.hpp'
        self.assertTrue(header.is_file(), 'native complete worker audit encoder is missing')
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / 'audit'
            subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                            '-I', str(ROOT), str(ROOT / 'tests/cpp/worker_result_json_fixture.cpp'),
                            '-o', str(binary)], check=True, capture_output=True)
            rng = random.Random(20260914)
            cases, expected = [], []
            for prefix in ('spark', 'mapped_palm'):
                for iteration in range(32):
                    fields = []
                    for key, fmt in _SPECS[prefix]:
                        if fmt.endswith('d'):
                            fields.extend(rng.uniform(-5, 5) for _ in range(int(fmt[:-1] or '1')))
                        elif fmt == 'B':
                            fields.append(iteration % 2 if key == '_height_present' else rng.randrange(2))
                        elif fmt == 'i':
                            fields.append(rng.randrange(-2147483648, 2147483648))
                        else:
                            fields.append(rng.randrange(2**63, 2**64))
                    data = struct.pack('<4sBBH', b'TJBR', 1, 2 if prefix == 'mapped_palm' else 1,
                                       _STRUCTS[prefix].size) + _STRUCTS[prefix].pack(*fields)
                    cases.append(dict(bytes=list(data), mapped=prefix == 'mapped_palm'))
                    expected.append(decode_result(data, prefix))
            result = subprocess.run([str(binary)], input='\n'.join(map(json.dumps, cases)),
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual([json.loads(line) for line in result.stdout.splitlines()], expected)
            # Kind 3 adds X offsets after the original height fields; kind 2 stays byte-identical.
            original=bytearray(cases[-1]['bytes'])
            offset=8
            for key,fmt in _SPECS['mapped_palm']:
                offset+=struct.calcsize('<'+fmt)
                if key=='target_height_offsets_m': break
            xz=original[:offset]+struct.pack('<2d',.1,.2)+original[offset:]
            xz[5]=3
            struct.pack_into('<H',xz,6,len(xz)-8)
            result=subprocess.run([str(binary)],input=json.dumps(dict(bytes=list(xz),mapped=True)),text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads(result.stdout),dict(expected[-1],target_x_offsets_m=[.1,.2]))
            nonfinite = bytearray(cases[0]['bytes'])
            offset = 8
            for _, fmt in _SPECS['spark']:
                if fmt.endswith('d'):
                    break
                offset += struct.calcsize('<' + fmt)
            struct.pack_into('<d', nonfinite, offset, float('nan'))
            for invalid in (dict(cases[0], mapped=True), dict(cases[0], bytes=cases[0]['bytes'][:-1]),
                            dict(cases[0], bytes=list(nonfinite)),
                            dict(cases[0], bytes=cases[0]['bytes'][:8] + [2] + cases[0]['bytes'][9:])):
                result = subprocess.run([str(binary)], input=json.dumps(invalid),
                                        text=True, capture_output=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(result.stdout, '')
