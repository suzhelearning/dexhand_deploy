from dataclasses import replace
import importlib.util
import time
import unittest

from tests import test_official_pico_input as pico_fixture
from tests.test_pico_official_backend import SideClient
from tests.test_hand_retarget_loop import Source
from tianji_teleop.producers.pico_official_hand import PicoOfficialHandBackend, PicoOfficialHandProducer
from tianji_teleop.producers.hand_retarget_loop import HandRetargetLoop
from tianji_teleop.protocol.messages import SessionState


class PicoHandLoopTest(unittest.TestCase):
    def test_shared_loop_handles_pico_frames_without_manus_array_conversion(self):
        self.assertIsNotNone(importlib.util.find_spec('tianji_teleop.producers.retarget_input'))
        from tianji_teleop.producers.retarget_input import pico_frame_input
        backend = PicoOfficialHandBackend({side: SideClient(side) for side in ('left', 'right')},
                                         receiver_instance_id='pico', connection_generation=3)
        producer = PicoOfficialHandProducer(backend, publisher_instance_id='hand', router_zid='router',
            coordinator_instance_id='coord', receiver_instance_id='pico', freshness_ns=100)
        source, published = Source(), []
        loop = HandRetargetLoop(producer, source, publish=published.append, clock=lambda: 1000,
                               input_adapter=pico_frame_input)
        self.addCleanup(loop.close)
        loop.update_session(SessionState(1, 1, 1000, 'teleop', 'test', 'coordinator', None, 'coord', 'router'))
        frame = pico_fixture.OfficialPicoInputTest().frame()
        source.rows.append(frame)
        deadline = time.monotonic() + 2
        while not published and not loop.failure and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertEqual(len(published), 1, loop.failure)
        self.assertEqual(published[0]['left'].sequence, 8)
        self.assertEqual(loop.processed_callbacks, 1)
        source.rows.append(replace(frame, connection_generation=4, receiver_frame_sequence=0))
        deadline = time.monotonic() + 2
        while not loop.failure and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertIn('generation changed', loop.failure)
        self.assertEqual(len(published), 1)
