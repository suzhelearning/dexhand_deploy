"""One owning clock for SPARK proposals and coordinator disposition.

The caller supplies input, executor feedback and explicit operator intents.
This cycle never authorizes a session and never substitutes actual feedback
for the native model reference. It must not be called concurrently.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class CycleResult:
    commands: dict
    native_result: dict | None
    receipt_accepted: bool
    native_attempt: dict | None = None


class SparkCoordinatorCycle:
    def __init__(self, coordinator, producer):
        self.coordinator = coordinator
        self.producer = producer
        self._last_ns = 0

    def step(self, now_ns):
        if type(now_ns) is not int or not self._last_ns < now_ns < 2**63:
            raise ValueError('control clock must be increasing positive int64')
        self._last_ns = now_ns
        co, producer = self.coordinator, self.producer
        producer.update_session(co.state)
        pair = producer.tick(now_ns)
        if pair is not None and not co.update_bilateral_proposal(pair, received_ns=now_ns):
            producer.guard.pause('coordinator rejected paired proposal')
        # Include native/ingress failure BEFORE disposition, not next tick.
        co.update_component(producer.status(now_ns), received_ns=now_ns)
        commands = co.tick(now_ns=now_ns)
        accepted = False
        if pair is not None and not producer.paused:
            accepted = producer.observe_execution(co.last_bilateral_receipt, now_ns)
        attempt = producer.last_attempt
        if attempt is not None and attempt['timestamp_ns'] != now_ns:
            attempt = None
        return CycleResult(commands, producer.last_result if pair is not None else None, accepted, attempt)
