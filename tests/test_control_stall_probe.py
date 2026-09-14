import sys
import unittest
import time
from threading import Event, Thread
from tianji_teleop.producers.spark.stall_probe import ControlStallProbe


class StallProbeTest(unittest.TestCase):
    def test_bounded_deduplicated_samples_and_threshold(self):
        state = [('actions', 1., 'idle', 3)]
        probe = ControlStallProbe(lambda: (next(iter(sys._current_frames())), state[0]), capacity=2)
        probe.sample(1.01)
        self.assertEqual(probe.samples, [])
        probe.sample(1.10)
        probe.sample(1.20)
        self.assertEqual(len(probe.samples), 1)
        self.assertEqual(probe.samples[0]['phase'], 'actions')
        self.assertTrue(probe.samples[0]['stack'])
        for start in (2., 3.):
            state[0] = ('core_step', start, 'teleop', 4)
            probe.sample(start + .1)
        self.assertEqual(len(probe.samples), 2)
        self.assertEqual(probe.dropped, 1)

    def test_no_live_thread_and_close_before_start(self):
        probe = ControlStallProbe(lambda: (None, ('waiting', 0., 'idle', 0)))
        probe.sample(1.)
        self.assertEqual(probe.samples, [])
        probe.close()

    def test_background_probe_captures_blocked_thread_without_releasing_it(self):
        entered, release = Event(), Event()
        def blocked_control():
            entered.set()
            release.wait(2)
        worker = Thread(target=blocked_control)
        worker.start()
        self.assertTrue(entered.wait(1))
        state = ('actions', time.monotonic() - .1, 'idle', 7)
        probe = ControlStallProbe(lambda: (worker.ident, state))
        try:
            probe.start()
            deadline = time.monotonic() + 1
            while not probe.samples and time.monotonic() < deadline:
                time.sleep(.01)
            probe.close()
            self.assertEqual(len(probe.samples), 1)
            self.assertTrue(worker.is_alive())
            self.assertTrue(any(frame['function'] == 'blocked_control'
                                for frame in probe.samples[0]['stack']))
        finally:
            release.set()
            worker.join(1)
            probe.close()
