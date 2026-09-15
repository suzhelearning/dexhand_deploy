import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
from tests.test_reference_manus_process import rawviz_records
from tianji_teleop.hand_tracking.reference_manus import HandInputAssembler, RawvizHandInputProcessor

ROOT = Path(__file__).resolve().parents[1]


class NativeManusParserTest(unittest.TestCase):
    @unittest.skipUnless((ROOT/'build/hand-native/tianji_hand_native_scheduler').is_file(),
                         'native Hand2 scheduler build required')
    def test_native_parser_feeds_actual_native_hand_scheduler(self):
        import time
        from tianji_teleop.hand_tracking.native_manus_parser import NativeManusProcessor
        from tianji_teleop.producers.native_hand_scheduler import NativeHandSchedulerClient
        from tianji_teleop.protocol.messages import SessionState,HAND_JOINT_NAMES
        from tests.test_gesture_recognition import hand_points
        frames=[];parser=NativeManusProcessor(frames.append)
        try:
            for side in ('right','left'):
                for line in rawviz_records(side,1,canonical_points=hand_points()).splitlines():
                    parser.process_line(line)
            self.assertEqual(len(frames),1)
            client=NativeHandSchedulerClient(
                python=ROOT/'tools/wuji_hand_native/.pixi/envs/default/bin/python',
                startup_handshake=True,timeout_seconds=5.)
            try:
                stamp=time.monotonic_ns()
                self.assertTrue(client.update_session(SessionState(1,1,stamp,'teleop','test',
                    'coordinator',None,'coord','router')))
                result=client.retarget(frames[0].values.tolist(),sequence=1,timestamp_ns=time.monotonic_ns())
                for side in ('left','right'):
                    self.assertTrue(result[side]['valid'])
                    self.assertEqual(result[side]['joint_names'],list(HAND_JOINT_NAMES[side]))
                    self.assertTrue(np.isfinite(result[side]['position_rad']).all())
                self.assertEqual(result['native_scheduler']['phase'],1)
            finally:client.close()
        finally:parser.close()

    def test_binding_and_resource_limit_are_explicit(self):
        from tianji_teleop.hand_tracking.native_manus_parser import NativeManusProcessor
        with tempfile.TemporaryDirectory() as directory:
            library=Path(directory)/'parser.so'
            subprocess.run(['c++','-std=c++17','-O2','-shared','-fPIC','-Wall','-Wextra','-Werror',
                str(ROOT/'native/hand/manus_input.cpp'),'-o',str(library)],check=True)
            expected,actual=[],[]
            reference=RawvizHandInputProcessor(HandInputAssembler(False,True),expected.append,
                                              left_glove='right-glove')
            native=NativeManusProcessor(actual.append,sides=('left',),left_glove='right-glove',library=library)
            try:
                for line in rawviz_records('right',1).splitlines():
                    reference.process_line(line);native.process_line(line)
                self.assertEqual(actual[0].sequences,{'left':1})
                np.testing.assert_array_equal(actual[0].values,expected[0].values)
                with self.assertRaisesRegex(RuntimeError,'bound'):native.process_line('HAND bad right 513')
                with self.assertRaisesRegex(RuntimeError,'65536'):native.process_line('x'*65537)
            finally:native.close()

    def test_receive_process_can_select_native_parser(self):
        from tests.test_reference_manus_process import ReferenceManusProcessTest
        fixture=ReferenceManusProcessTest()
        with tempfile.TemporaryDirectory() as directory:
            library=Path(directory)/'parser.so'
            subprocess.run(['c++','-std=c++17','-O2','-shared','-fPIC','-Wall','-Wextra','-Werror',
                str(ROOT/'native/hand/manus_input.cpp'),'-o',str(library)],check=True)
            try:
                source=fixture.source(rawviz_records('right',1)+rawviz_records('left',1),
                    parser_backend='cpp',parser_library=library)
                fixture.wait_until(lambda:source.pending_count==1)
                self.assertEqual(source.try_read().source_sequences,{'right':1,'left':1})
                source.close();source.close()
            finally: fixture.doCleanups()

    def test_parser_matches_reference_callbacks(self):
        name = 'tianji_teleop.hand_tracking.native_manus_parser'
        self.assertIsNotNone(importlib.util.find_spec(name), 'native Manus parser adapter missing')
        from tianji_teleop.hand_tracking.native_manus_parser import NativeManusProcessor
        with tempfile.TemporaryDirectory() as directory:
            library = Path(directory) / 'parser.so'
            subprocess.run(['c++','-std=c++17','-O2','-shared','-fPIC','-Wall','-Wextra','-Werror',
                str(ROOT/'native/hand/manus_input.cpp'),'-o',str(library)],check=True)
            for sides in (('right','left'),('left',),('right',)):
                expected, actual = [], []
                reference = RawvizHandInputProcessor(HandInputAssembler('right' in sides,'left' in sides), expected.append)
                native = NativeManusProcessor(actual.append,sides=sides,library=library)
                try:
                    lines = (rawviz_records('right',1)+rawviz_records('left',1)+
                             rawviz_records('right',2)+rawviz_records('right',2)+
                             'POSE left-glove 3 3000 0 1 2 3 1 0 0 0\n'+
                             rawviz_records('right',4)+rawviz_records('left',4)+
                             rawviz_records('left',5).replace('1.0 2.0 3.0','1e999 2.0 3.0')+
                             rawviz_records('right',6)+rawviz_records('left',6)+
                             rawviz_records('left',7).replace('1.0 2.0 3.0','0x1p0 2.0 3.0')+
                             rawviz_records('left',8).replace('1.0 2.0 3.0','1_0.0 2.0 3.0')+
                             rawviz_records('left',10).replace('left-glove 10 10000','left-glove 1_0 10_000')+
                             rawviz_records('left',11).replace('11000 0 ','11000 18446744073709551615 ')+
                             'unrelated SDK message\nNODE bad\n').splitlines()
                    for line in lines:
                        reference.process_line(line); native.process_line(line)
                        self.assertEqual(len(actual),len(expected))
                    for a,b in zip(actual,expected):
                        np.testing.assert_array_equal(a.values,b.values)
                        self.assertEqual(a.sequences,b.sequences)
                        self.assertEqual(a.source_timestamps_ns,b.source_timestamps_ns)
                finally: native.close()
