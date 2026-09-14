from threading import Event
from types import SimpleNamespace
import unittest

from tianji_teleop.producers.spark.live_control import SparkControlLoop
from tianji_teleop.producers.spark.mapped_height_simulation import MappedHeightControlLoop


class BarrierScheduleTest(unittest.TestCase):
    def run_barrier(self, loop_type, *, calibration=False, state='idle'):
        now = [1.]
        waits = []

        class Stop(Event):
            def wait(self, timeout=None):
                waits.append(timeout)
                self.set()
                return True

        stop = Stop()
        calls = [0]

        def barrier():
            calls[0] += 1
            now[0] += .26
            if calls[0] > 1:
                stop.set()
            return {'execution_epoch': 2}

        core = SimpleNamespace(
            coordinator=SimpleNamespace(state=SimpleNamespace(state=state)),
            height_events=[], rearm_at_home=barrier,
            finish_height_calibration=lambda: bool(barrier()) if calibration else False)
        loop = loop_type(core, None, stop_event=stop, period_s=.005,
            failure_fn=lambda: None, snapshot_sink=SimpleNamespace(submit=lambda _: True),
            clock=lambda: now[0])
        if not calibration:
            loop.submit('rearm')
        process_actions = loop._process_actions
        def bounded_actions():
            if calls[0]:
                stop.set()
                return True
            return process_actions()
        loop._process_actions = bounded_actions
        loop._run()
        return loop, waits

    def test_calibration_reanchors_after_successful_idle_barrier(self):
        loop, waits = self.run_barrier(MappedHeightControlLoop, calibration=True)
        self.assertEqual(len(waits), 1)
        self.assertAlmostEqual(waits[0], .005)
        self.assertEqual(loop.late_cycles, 0)
        self.assertIsNone(loop.failure)

    def test_explicit_home_rearm_also_reanchors(self):
        loop, waits = self.run_barrier(MappedHeightControlLoop)
        self.assertEqual(len(waits), 1)
        self.assertAlmostEqual(waits[0], .005)
        self.assertIsNone(loop.failure)

    def test_original_spark_does_not_reanchor(self):
        loop, waits = self.run_barrier(SparkControlLoop)
        self.assertEqual(waits, [])
        self.assertGreater(loop.late_cycles, 0)
        self.assertIsNone(loop.failure)

    def test_does_not_reanchor_when_actions_have_entered_teleop(self):
        loop, waits = self.run_barrier(MappedHeightControlLoop, calibration=True, state='teleop')
        self.assertEqual(waits, [])
        self.assertGreater(loop.late_cycles, 0)
        self.assertIsNone(loop.failure)
