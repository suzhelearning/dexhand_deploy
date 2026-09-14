"""Bounded best-effort Python stack sampling; never a control authority."""
import sys
from threading import Event, Thread


class ControlStallProbe:
    def __init__(self, snapshot, *, capacity=32):
        self.snapshot = snapshot
        self.capacity = capacity
        self.samples = []
        self.dropped = 0
        self.error = None
        self._last = None
        self._stop = Event()
        self._thread = None

    def sample(self, now):
        ident, state = self.snapshot()
        phase, started, session_state, tick = state
        if ident is None or phase == 'stopped' or now - started < .05 or state == self._last:
            return
        self._last = state
        if len(self.samples) >= self.capacity:
            self.dropped += 1
            return
        frame = sys._current_frames().get(ident)
        if frame is None:
            return
        stack = []
        try:
            # Do not retain frames, locals, or source text; no disk IO here.
            while frame is not None and len(stack) < 16:
                stack.append(dict(file=frame.f_code.co_filename,
                                  line=frame.f_lineno, function=frame.f_code.co_name))
                frame = frame.f_back
        finally:
            del frame
        self.samples.append(dict(phase=phase, state=session_state, tick=tick,
            observed_monotonic_s=now, elapsed_ms=(now-started)*1000, stack=stack))

    def start(self):
        if self._thread is not None:
            raise RuntimeError('stall probe already started')
        self._thread = Thread(target=self._run, name='control-stall-probe', daemon=True)
        self._thread.start()

    def _run(self):
        import time
        try:
            while not self._stop.wait(.02):
                self.sample(time.monotonic())
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1)
