import importlib.util
from dataclasses import replace
from threading import Event
import time
import unittest

from tests.test_hand_producer_authority import Backend
from tianji_teleop.producers.hand_retarget import HandRetargetProducer
from tianji_teleop.protocol.messages import SessionState
from tianji_teleop.hand_tracking.reference_manus_process import ManusCallback


class Source:
    failure = None
    def __init__(self):
        self.rows = []
    def try_read(self):
        return self.rows.pop(0) if self.rows else None


class HandRetargetLoopTest(unittest.TestCase):
    def wait_until(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate() and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(predicate())

    def setup_loop(self, backend=None):
        module = 'tianji_teleop.producers.hand_retarget_loop'
        self.assertIsNotNone(importlib.util.find_spec(module))
        from tianji_teleop.producers.hand_retarget_loop import HandRetargetLoop
        producer = HandRetargetProducer(backend or Backend(), publisher_instance_id='hand',
            router_zid='router', coordinator_instance_id='coord', receiver_instance_id='source',
            freshness_ns=1_000_000_000)
        source, published = Source(), []
        loop = HandRetargetLoop(producer, source, publish=published.append, clock=lambda: 1_000_000_000)
        self.addCleanup(loop.close)
        return loop, source, published

    def state(self, mode, seq):
        return SessionState(1, seq, 1_000_000_000, mode, 'test', 'coordinator', None, 'coord', 'router')

    def row(self):
        return ManusCallback('source', 1, 1_000_000_000, (0.,) * 126,
                             {'right': 1, 'left': 1}, {'right': 10, 'left': 20})

    def test_actual_callback_processed_once_and_never_repeated(self):
        loop, source, published = self.setup_loop()
        loop.update_session(self.state('teleop', 1))
        source.rows.append(self.row())
        self.wait_until(lambda: len(published) == 1)
        self.assertEqual(set(published[0]), {'left', 'right'})
        time.sleep(.03)
        self.assertEqual(len(published), 1)

    def test_session_stop_during_retarget_prevents_late_publication(self):
        entered, release = Event(), Event()
        self.addCleanup(release.set)
        class SlowBackend(Backend):
            def retarget(self, *args, **kwargs):
                entered.set()
                release.wait(1)
                return super().retarget(*args, **kwargs)
        loop, source, published = self.setup_loop(SlowBackend())
        loop.update_session(self.state('teleop', 1))
        source.rows.append(self.row())
        self.assertTrue(entered.wait(1))
        loop.update_session(self.state('returning', 2))
        release.set()
        self.wait_until(lambda: loop.processed_callbacks == 1)
        self.assertEqual(published, [])

    def test_source_failure_latches_loop_and_drops_queued_input(self):
        loop, source, published = self.setup_loop()
        source.failure = 'reader exited'
        self.wait_until(lambda: loop.failure is not None)
        source.rows.append(self.row())
        self.assertEqual(published, [])

    def test_readiness_needs_fresh_valid_required_sides(self):
        loop, source, _ = self.setup_loop()
        self.assertTrue(hasattr(loop, 'status'))
        self.assertFalse(loop.status(1_000_000_000, ('left', 'right')).ready)
        source.rows.append(self.row())
        self.wait_until(lambda: loop.processed_callbacks == 1)
        status = loop.status(1_000_000_000, ('left', 'right'))
        self.assertTrue(status.ready)
        self.assertTrue(status.healthy)
        self.assertIn('max_retarget_duration_ns', status.diagnostics)
        stale = loop.status(2_000_000_001, ('left', 'right'))
        self.assertFalse(stale.ready)
        self.assertTrue(stale.healthy)

    def test_stale_callback_reports_measured_age_without_relaxing_gate(self):
        loop, source, published = self.setup_loop()
        loop._clock = lambda: 3_000_000_000
        source.rows.append(self.row())
        self.wait_until(lambda: loop.failure is not None)
        self.assertIn('age_ns=2000000000', loop.failure)
        self.assertEqual(published, [])

    def test_new_callback_after_control_tick_cutoff_does_not_hide_previous_fresh_input(self):
        loop, source, _ = self.setup_loop()
        source.rows.append(self.row())
        self.wait_until(lambda: loop.processed_callbacks == 1)
        loop._clock = lambda: 1_000_000_010
        source.rows.append(replace(self.row(), sequence=2, received_timestamp_ns=1_000_000_010))
        self.wait_until(lambda: loop.processed_callbacks == 2)
        # Arm cycle sampled its clock before the hand thread completed frame 2.
        status = loop.status(1_000_000_005, ('left', 'right'))
        self.assertTrue(status.ready, status.to_dict())
        self.assertEqual(status.diagnostics['input_timestamp_ns'], 1_000_000_000)
        self.assertEqual(loop.status(1_000_000_010, ('left', 'right')).diagnostics['input_timestamp_ns'],
                         1_000_000_010)

    def test_cutoff_history_does_not_make_stale_input_fresh(self):
        loop, source, _ = self.setup_loop()
        source.rows.append(self.row())
        self.wait_until(lambda: loop.processed_callbacks == 1)
        loop._clock = lambda: 2_000_000_010
        source.rows.append(replace(self.row(), sequence=2, received_timestamp_ns=2_000_000_010))
        self.wait_until(lambda: loop.processed_callbacks == 2)
        self.assertFalse(loop.status(2_000_000_005, ('left', 'right')).ready)
        self.assertFalse(loop.status(999_999_999, ('left', 'right')).ready)
