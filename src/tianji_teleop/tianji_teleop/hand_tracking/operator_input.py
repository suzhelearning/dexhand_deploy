"""Explicitly bound operator edges; events are requests, never authorization.

No subscriptions or device imports. The owner must bind a fresh instance for
each connection epoch and route requests through the existing session machine.
Replay may retain observations but cannot generate live events.
"""
from dataclasses import dataclass
import math


ACTIONS = frozenset({'start_request', 'pause_request', 'home_request',
                     'calibrate_request', 'clutch_press', 'clutch_release'})


def _nonnegative_integer(value):
    return type(value) is int and 0 <= value <= 2**63 - 1


@dataclass(frozen=True)
class OperatorObservation:
    source: str
    side: str
    sequence: int
    epoch: int
    receive_time_ns: int
    valid: bool
    available: bool
    action: str
    pressed: bool
    confidence: float

    def __post_init__(self):
        if (not isinstance(self.source, str) or not self.source.strip() or
                self.side not in ('left', 'right', 'both', 'none') or self.action not in ACTIONS or
                any(not _nonnegative_integer(v) for v in
                    (self.sequence, self.epoch, self.receive_time_ns)) or
                any(type(v) is not bool for v in (self.valid, self.available, self.pressed)) or
                type(self.confidence) not in (int, float) or
                not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1):
            raise ValueError('invalid operator observation')


@dataclass(frozen=True)
class OperatorEvent:
    source: str
    side: str
    sequence: int
    epoch: int
    receive_time_ns: int
    action: str
    edge: str
    confidence: float


class OperatorEdgeFilter:
    """Release-before-press debounce for one explicit source/side/action/epoch.

    Stability uses local receive time, not an unsynchronized device clock.
    Missing, invalid, replayed or stale input disarms rather than queuing a
    request for later. Duplicate/foreign input cannot advance filter state.
    This class has no session authority and does not execute its events.
    """
    def __init__(self, *, source, side, action, epoch, freshness_ns, stable_ns,
                 minimum_confidence=.8):
        OperatorObservation(source, side, 0, epoch, 0, True, True, action, False, 1.)
        if (not _nonnegative_integer(freshness_ns) or freshness_ns == 0 or
                not _nonnegative_integer(stable_ns)):
            raise ValueError('explicit freshness and stability intervals required')
        if (type(minimum_confidence) not in (float, int) or
                not math.isfinite(minimum_confidence) or not 0 <= minimum_confidence <= 1):
            raise ValueError('minimum confidence must be finite in 0..1')
        self.binding = (source, side, action, epoch)
        self.freshness_ns = freshness_ns
        self.stable_ns = stable_ns
        self.minimum_confidence = minimum_confidence
        self._sequence = None
        self._received = None
        self._released = False
        self._press_since = None

    def _disarm(self):
        self._released = False
        self._press_since = None

    def invalidate(self):
        """Lose the pending edge while preserving sequence/replay watermarks."""
        self._disarm()

    def update(self, observation, *, now_ns, replay=False):
        if (not isinstance(observation, OperatorObservation) or
                not _nonnegative_integer(now_ns) or type(replay) is not bool):
            raise ValueError('validated observation and local monotonic time required')
        o = observation
        if (o.source, o.side, o.action, o.epoch) != self.binding:
            return None
        if self._sequence is not None and o.sequence <= self._sequence:
            return None
        gap = (self._sequence is not None and o.sequence != self._sequence + 1)
        clock_gap = (self._received is not None and
                     not 0 < o.receive_time_ns - self._received <= self.freshness_ns)
        self._sequence, self._received = o.sequence, o.receive_time_ns
        if (replay or gap or clock_gap or not o.valid or not o.available or
                o.confidence < self.minimum_confidence or
                not 0 <= now_ns - o.receive_time_ns <= self.freshness_ns):
            self._disarm()
            return None
        if not o.pressed:
            self._released = True
            self._press_since = None
            return None
        if not self._released:
            return None
        if self._press_since is None:
            self._press_since = o.receive_time_ns
        if o.receive_time_ns - self._press_since < self.stable_ns:
            return None
        self._disarm()
        return OperatorEvent(o.source, o.side, o.sequence, o.epoch, o.receive_time_ns,
                             o.action, 'rising', o.confidence)
