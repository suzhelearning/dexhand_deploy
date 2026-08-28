"""Coordinator-facing session lifecycle client for canonical sources.

The client is deliberately a small composition primitive: it observes the
coordinator's state/latches and publishes intents, but never publishes
SessionState itself.  Every message is decoded through the protocol package;
unknown or foreign coordinator instances are ignored and cannot authorize
motion.
"""
from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable

from ...protocol import topics
from ...protocol.messages import LatchedBool, ProtocolError, SessionIntent, SessionState
from ...zenoh_util import ZenohPub
from .target_publisher import SequenceAllocator


class SessionClient:
    """Subscribe/query coordinator state before a source may move."""

    def __init__(
        self,
        session: Any,
        *,
        source: str,
        publisher_instance_id: str,
        router_zid: str,
        expected_coordinator_instance_id: str | None = None,
        snapshot_timeout_s: float = 1.0,
        clock: Callable[[], int] = time.monotonic_ns,
        allocator: SequenceAllocator | None = None,
    ) -> None:
        if not expected_coordinator_instance_id:
            raise ValueError("expected_coordinator_instance_id is required")
        if not source or not publisher_instance_id or not router_zid:
            raise ValueError("source, publisher_instance_id and router_zid are required")
        if snapshot_timeout_s <= 0.0:
            raise ValueError("snapshot_timeout_s must be positive")
        self._session = session
        self.source = source
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.expected_coordinator_instance_id = expected_coordinator_instance_id
        self._snapshot_timeout_s = float(snapshot_timeout_s)
        self._clock = clock
        self._allocator = allocator or SequenceAllocator()
        self._intent_publisher = ZenohPub(session, topics.SESSION_INTENT)
        self._resources: list[Any] = []
        self._lock = threading.RLock()
        self._state_event = threading.Event()
        # Each latched key has an independent query completion.  Subscriber
        # traffic is deliberately not allowed to satisfy this barrier: a
        # source must have a snapshot for all three authority keys before it
        # can connect or move.
        self._snapshot_event = threading.Event()
        self._query_complete = {
            "state": False,
            "at_home": False,
            "return_complete": False,
        }
        self._query_reply_count = {
            "state": 0,
            "at_home": 0,
            "return_complete": 0,
        }
        self._started = False
        self._snapshot_started_at = 0.0
        self._snapshot_timed_out = False
        self._state: SessionState | None = None
        self._at_home: LatchedBool | None = None
        self._return_complete: LatchedBool | None = None
        self._coordinator_identity: str | None = None
        # Coordinator sequence is publisher-instance global, not per topic.
        # A control tick may publish state and both latches with one sequence,
        # so the same sequence is accepted once per channel but never rolls
        # back globally.
        self._coordinator_sequence_baseline = -1
        self._accepted_coordinator_messages: set[tuple[str, int]] = set()
        self._pending_action: str | None = None
        self._pending_intent_sequence: int | None = None
        self._pending_deadline = 0.0
        self._intent_baselines: dict[int, tuple[int, int, int]] = {}
        self._invalid_coordinator = False

    @property
    def state(self) -> SessionState | None:
        with self._lock:
            return self._state

    @property
    def at_home(self) -> bool | None:
        with self._lock:
            return None if self._at_home is None else self._at_home.value

    @property
    def return_complete(self) -> bool | None:
        with self._lock:
            return None if self._return_complete is None else self._return_complete.value
    @property
    def return_complete_sequence(self) -> int:
        with self._lock:
            return -1 if self._return_complete is None else self._return_complete.sequence

    @property
    def at_home_sequence(self) -> int:
        with self._lock:
            return -1 if self._at_home is None else self._at_home.sequence

    @property
    def return_authorized(self) -> bool:
        with self._lock:
            return (
                self._pending_action == "return"
                and self._state is not None
                and self._state.intent_sequence == self._pending_intent_sequence
                and self._state.state in {"returning", "idle"}
            )

    @property
    def return_completion_fresh(self) -> bool:
        with self._lock:
            intent = self._pending_intent_sequence
            if intent is None:
                return False
            baseline = self._intent_baselines.get(intent)
            if baseline is None:
                return False
            return (
                self.return_authorized
                and self._at_home is not None
                and self._at_home.value
                and self._at_home.sequence > baseline[1]
                and self._return_complete is not None
                and self._return_complete.value
                and self._return_complete.sequence > baseline[2]
            )
    @property
    def return_intent_baseline(self) -> tuple[int, int, int] | None:
        with self._lock:
            if self._pending_intent_sequence is None:
                return None
            return self._intent_baselines.get(self._pending_intent_sequence)
    @property
    def pending_intent_sequence(self) -> int | None:
        with self._lock:
            return self._pending_intent_sequence

    @property
    def start_authorized(self) -> bool:
        with self._lock:
            return self._authorized_state("teleop", "start")

    @property
    def startup_ready(self) -> bool:
        """True only after all three independent query snapshots completed."""
        with self._lock:
            self._poll_timeout_locked()
            return (
                self._snapshot_event.is_set()
                and not self._snapshot_timed_out
                and not self._invalid_coordinator
            )

    @property
    def snapshot_timed_out(self) -> bool:
        with self._lock:
            self._poll_timeout_locked()
            return self._snapshot_timed_out

    @property
    def coordinator_instance_id(self) -> str | None:
        with self._lock:
            return self._coordinator_identity

    @property
    def snapshot_complete(self) -> bool:
        with self._lock:
            self._poll_timeout_locked()
            return self._snapshot_event.is_set() and not self._snapshot_timed_out

    def reconnect(self) -> None:
        """Drop coordinator identity/baselines and perform a fresh startup query."""
        for resource in self._resources:
            try:
                resource.undeclare()
            except Exception:
                pass
        self._resources.clear()
        with self._lock:
            self._started = False
            self._state = None
            self._at_home = None
            self._return_complete = None
            self._coordinator_identity = None
            self._coordinator_sequence_baseline = -1
            self._accepted_coordinator_messages.clear()
            self._query_complete = {
                "state": False,
                "at_home": False,
                "return_complete": False,
            }
            self._query_reply_count = {
                "state": 0,
                "at_home": 0,
                "return_complete": 0,
            }
            self._snapshot_event.clear()
            self._snapshot_timed_out = False
            self._pending_action = None
            self._pending_intent_sequence = None
            self._invalid_coordinator = False
        self.start()

    def start(self) -> None:
        """Declare subscribers first, then request one-shot coordinator snapshots."""
        with self._lock:
            if self._started:
                raise RuntimeError("SessionClient already started")
            self._started = True
            self._snapshot_started_at = time.monotonic()
            self._snapshot_timed_out = False
            self._snapshot_event.clear()
            self._query_complete = {
                "state": False,
                "at_home": False,
                "return_complete": False,
            }
            self._query_reply_count = {
                "state": 0,
                "at_home": 0,
                "return_complete": 0,
            }
        # Subscriber declaration intentionally precedes every query.
        self._resources.extend(
            [
                self._session.declare_subscriber(topics.SESSION_STATE, self._on_state_sample),
                self._session.declare_subscriber(topics.AT_HOME, self._on_at_home_sample),
                self._session.declare_subscriber(topics.RETURN_COMPLETE, self._on_return_complete_sample),
            ]
        )
        self._query(topics.SESSION_STATE, self._on_state_reply)
        self._query(topics.AT_HOME, self._on_at_home_reply)
        self._query(topics.RETURN_COMPLETE, self._on_return_complete_reply)

    def _query(self, key: str, callback: Callable[[Any], None]) -> None:
        try:
            self._session.get(key, callback, timeout=self._snapshot_timeout_s)
        except TypeError:
            # Small fakes and older zenoh-python versions may not expose timeout.
            self._session.get(key, callback)
        except Exception:
            # A missing snapshot is fail-closed; the timer below keeps startup blocked.
            return
    @staticmethod
    def _payload(value: Any) -> bytes | None:
        if isinstance(value, (bytes, bytearray, memoryview)):
            return bytes(value)
        if isinstance(value, dict):
            return json.dumps(value, separators=(",", ":")).encode("utf-8")
        payload = getattr(value, "payload", None)
        if payload is not None:
            return bytes(payload)
        result = getattr(value, "result", None)
        payload = getattr(result, "payload", None)
        return None if payload is None else bytes(payload)

    def _mark_query(self, channel: str, *, success: bool) -> None:
        with self._lock:
            count = self._query_reply_count[channel] + 1
            self._query_reply_count[channel] = count
            if count != 1 or not success:
                self._invalid_coordinator = True
                self._query_complete[channel] = False
            else:
                self._query_complete[channel] = True
            if all(self._query_complete.values()) and not self._invalid_coordinator:
                self._snapshot_event.set()

    def _on_state_sample(self, sample: Any) -> None:
        payload = self._payload(sample)
        if payload:
            self._on_state_payload(payload)

    def _on_at_home_sample(self, sample: Any) -> None:
        payload = self._payload(sample)
        if payload:
            self._on_latched_payload(payload, is_home=True)

    def _on_return_complete_sample(self, sample: Any) -> None:
        payload = self._payload(sample)
        if payload:
            self._on_latched_payload(payload, is_home=False)

    def _on_state_reply(self, reply: Any) -> None:
        payload = self._payload(reply)
        if getattr(reply, "ok", True) is False or not payload:
            self._mark_query("state", success=False)
            return
        self._on_state_payload(payload, query_channel="state")

    def _on_at_home_reply(self, reply: Any) -> None:
        payload = self._payload(reply)
        if getattr(reply, "ok", True) is False or not payload:
            self._mark_query("at_home", success=False)
            return
        self._on_latched_payload(payload, is_home=True, query_channel="at_home")

    def _on_return_complete_reply(self, reply: Any) -> None:
        payload = self._payload(reply)
        if getattr(reply, "ok", True) is False or not payload:
            self._mark_query("return_complete", success=False)
            return
        self._on_latched_payload(
            payload, is_home=False, query_channel="return_complete"
        )
    def _accept_coordinator(
        self,
        instance: str,
        sequence: int,
        router: str,
        channel: str,
        *,
        origin: str = "sample",
    ) -> bool:
        if router != self.router_zid:
            self._invalid_coordinator = True
            return False
        if instance != self.expected_coordinator_instance_id:
            self._invalid_coordinator = True
            return False
        if self._coordinator_identity is None:
            self._coordinator_identity = instance
        elif self._coordinator_identity != instance:
            self._invalid_coordinator = True
            return False
        if sequence < self._coordinator_sequence_baseline:
            return False
        if sequence > self._coordinator_sequence_baseline:
            self._coordinator_sequence_baseline = sequence
            self._accepted_coordinator_messages.clear()
        token = (f"{origin}:{channel}", sequence)
        if token in self._accepted_coordinator_messages:
            return False
        self._accepted_coordinator_messages.add(token)
        return True

    def _on_state_payload(
        self, payload: bytes, *, query_channel: str | None = None
    ) -> bool:
        try:
            state = SessionState.from_dict(json.loads(payload.decode("utf-8")))
        except (UnicodeDecodeError, json.JSONDecodeError, ProtocolError, TypeError, ValueError):
            if query_channel is not None:
                self._mark_query(query_channel, success=False)
            return False
        with self._lock:
            accepted = self._accept_coordinator(
                state.publisher_instance_id,
                state.sequence,
                state.router_zid,
                "state",
                origin="query" if query_channel is not None else "sample",
            )
            if accepted:
                self._state = state
                if (
                    self._pending_action == "start"
                    and state.intent_sequence == self._pending_intent_sequence
                    and state.state != "teleop"
                ):
                    self._pending_action = None
                    self._pending_intent_sequence = None
                self._state_event.set()
        if query_channel is not None:
            self._mark_query(query_channel, success=accepted)
        return accepted

    def _on_latched_payload(
        self,
        payload: bytes,
        *,
        is_home: bool,
        query_channel: str | None = None,
    ) -> bool:
        try:
            latch = LatchedBool.from_dict(json.loads(payload.decode("utf-8")))
        except (UnicodeDecodeError, json.JSONDecodeError, ProtocolError, TypeError, ValueError):
            if query_channel is not None:
                self._mark_query(query_channel, success=False)
            return False
        with self._lock:
            accepted = self._accept_coordinator(
                latch.publisher_instance_id,
                latch.sequence,
                latch.router_zid,
                "at_home" if is_home else "return_complete",
                origin="query" if query_channel is not None else "sample",
            )
            if accepted:
                if is_home:
                    self._at_home = latch
                else:
                    self._return_complete = latch
        if query_channel is not None:
            self._mark_query(query_channel, success=accepted)
        return accepted

    def _authorized_state(self, state: str, action: str) -> bool:
        current = self._state
        pending = self._pending_intent_sequence
        return (
            current is not None
            and current.state == state
            and pending is not None
            and current.intent_sequence == pending
            and self._pending_action == action
            and self._coordinator_identity is not None
            and not self._invalid_coordinator
        )

    def _poll_timeout_locked(self) -> None:
        if self._started and not self._snapshot_event.is_set() and time.monotonic() - self._snapshot_started_at >= self._snapshot_timeout_s:
            self._snapshot_timed_out = True
        if self._pending_action is not None and time.monotonic() >= self._pending_deadline:
            authorized = (
                self._state is not None
                and self._state.intent_sequence == self._pending_intent_sequence
                and (
                    (self._pending_action == "start" and self._state.state == "teleop")
                    or (self._pending_action == "return" and self._state.state in {"returning", "idle"})
                )
            )
            if not authorized:
                self._pending_action = None
                self._pending_intent_sequence = None

    def poll(self) -> None:
        with self._lock:
            self._poll_timeout_locked()

    def _request(self, action: str, reason: str, timeout_s: float) -> int:
        if action not in ("start", "return", "shutdown"):
            raise ValueError("unsupported session action")
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        with self._lock:
            self._poll_timeout_locked()
            if not self._started:
                raise RuntimeError("SessionClient must be started before requesting intents")
            baseline = (
                -1 if self._state is None else self._state.sequence,
                -1 if self._at_home is None else self._at_home.sequence,
                -1 if self._return_complete is None else self._return_complete.sequence,
            )
            sequence = self._allocator.next()
            timestamp_ns = int(self._clock())
            intent = SessionIntent(
                schema_version=1,
                sequence=sequence,
                timestamp_ns=timestamp_ns,
                source=self.source,
                action=action,
                reason=str(reason),
                publisher_instance_id=self.publisher_instance_id,
                router_zid=self.router_zid,
            )
            self._intent_baselines[sequence] = baseline
            self._pending_action = action
            self._pending_intent_sequence = sequence
            self._pending_deadline = time.monotonic() + float(timeout_s)
        self._intent_publisher.put_json(intent.to_dict())
        return sequence

    def request_start(self, reason: str = "operator", timeout_s: float = 1.0) -> int:
        return self._request("start", reason, timeout_s)

    def request_return(self, reason: str = "source_return", timeout_s: float = 1.0) -> int:
        return self._request("return", reason, timeout_s)

    def request_shutdown(self, reason: str = "source_shutdown", timeout_s: float = 1.0) -> int:
        return self._request("shutdown", reason, timeout_s)

    def wait_for_state(self, state: str, *, intent_sequence: int | None = None, timeout_s: float = 1.0) -> bool:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            with self._lock:
                self._poll_timeout_locked()
                current = self._state
                expected_intent = self._pending_intent_sequence if intent_sequence is None else intent_sequence
                if (
                    current is not None
                    and current.state == state
                    and current.intent_sequence == expected_intent
                    and not self._invalid_coordinator
                ):
                    return True
            self._state_event.wait(min(0.01, max(0.0, deadline - time.monotonic())))
            self._state_event.clear()
        return False

    def close(self) -> None:
        for resource in self._resources:
            try:
                resource.undeclare()
            except Exception:
                try:
                    resource.close()
                except Exception:
                    pass
        self._resources.clear()
        self._intent_publisher.close()
        with self._lock:
            self._started = False
            self._pending_action = None
            self._pending_intent_sequence = None
