from pathlib import Path
import subprocess
import tempfile
import unittest
import numpy as np
from tests.test_reference_manus_process import rawviz_records
from tianji_teleop.hand_tracking.reference_manus import HandInputAssembler, RawvizHandInputProcessor

ROOT = Path(__file__).resolve().parents[1]


class ManusIngressTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix='native-manus-ingress-')
        cls.addClassCleanup(cls.directory.cleanup)
        cls.binary = Path(cls.directory.name)/'ingress'
        subprocess.run(['c++', '-std=c++17', '-pthread', '-O2', '-Wall', '-Wextra', '-Werror',
                        str(ROOT/'tests/cpp/native_manus_ingress.cpp'),
                        str(ROOT/'native/hand/manus_input.cpp'), '-o', str(cls.binary)], check=True)

    def test_driver_output_matches_reference_under_fragmentation(self):
        data = rawviz_records('right',1)+rawviz_records('left',1)+rawviz_records('right',2)
        expected=[]
        processor=RawvizHandInputProcessor(HandInputAssembler(True,True),expected.append)
        for line in data.splitlines():processor.process_line(line)
        self.assertGreater(len(expected), 0)
        for mode in ('whole','fragment'):
            p=subprocess.run([str(self.binary),mode],input=data,text=True,capture_output=True,timeout=5)
            self.assertEqual(p.returncode,0,p.stderr)
            rows=[line.split() for line in p.stdout.splitlines()]
            self.assertEqual(len(rows),len(expected))
            for i,(row,reference) in enumerate(zip(rows,expected),1):
                self.assertEqual(list(map(int,row[:6])),[i,1000,1,3,reference.sequences['right'],reference.sequences['left']])
                np.testing.assert_array_equal(np.array(row[6:],float),reference.values.astype(float))

    def test_invalid_stream_or_failed_sink_is_visible(self):
        for mode,data,reason in [('whole','x'*65537,'bound'),('whole','POSE unfinished','truncated'),
                                 ('whole','bad\0line\n','NUL'),('reject','diagnostic\n','sink')]:
            p=subprocess.run([str(self.binary),mode],input=data,text=True,capture_output=True,timeout=5)
            self.assertNotEqual(p.returncode,0)
            self.assertIn(reason,p.stderr)

    def test_exclusive_pipe_receiver_eof_and_cancellation(self):
        binary=Path(self.directory.name)/'receiver'
        subprocess.run(['c++','-std=c++17','-pthread','-O2','-Wall','-Wextra','-Werror',
                        str(ROOT/'tests/cpp/native_manus_receiver.cpp'),
                        str(ROOT/'native/hand/manus_input.cpp'),'-o',str(binary)],check=True)
        for mode,data in [('idle',''),('partial','POSE unfinished'),
                          ('stream',rawviz_records('right',1)+rawviz_records('left',1))]:
            p=subprocess.run([str(binary),mode],input=data,text=True,capture_output=True,timeout=5)
            self.assertEqual(p.returncode,0,p.stderr)
