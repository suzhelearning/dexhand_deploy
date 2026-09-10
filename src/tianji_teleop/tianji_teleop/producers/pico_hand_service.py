"""Simulation-only PICO hand component using the existing raw/session topics.

The managed launcher owns workers, router/domain guards and liveliness. This
component owns only subscriptions and the shared retarget thread; it does not
start a second PICO receiver or any executor. Session.put must be bounded, as
required by HandRetargetLoop's publication contract.
"""
from dataclasses import replace
import json
from threading import RLock
import time

from ..hand_tracking.pico_raw_input import PicoRawInputQueue
from ..protocol import topics
from ..protocol.messages import strict_loads
from .hand_retarget_loop import HandRetargetLoop
from .pico_official_hand import PicoOfficialHandProducer
from .retarget_input import pico_frame_input


def _payload(value):
    payload = getattr(value, 'payload', value)
    return payload.to_bytes() if hasattr(payload, 'to_bytes') else payload


class PicoHandService:
    def __init__(self, session, producer, *, required_capability, clock=time.monotonic_ns,
                 authority_guard=None, audit_processed_inputs=False):
        if required_capability != 'simulation':
            raise ValueError('PICO official hand service is simulation-only until separate real validation')
        if type(audit_processed_inputs) is not bool:
            raise ValueError('audit_processed_inputs must be boolean')
        if (not isinstance(producer, PicoOfficialHandProducer) or
                producer.receiver_instance_id != producer.backend.receiver_instance_id):
            raise ValueError('explicit consistently bound PICO hand producer required')
        self._session, self._clock = session, clock
        self._authority_guard = authority_guard
        self._lock = RLock()
        self._closed = False
        self._fault = None
        self._subscriptions = []
        self._producer = producer
        self._audit_sequence = 0
        self._source = PicoRawInputQueue(receiver_instance_id=producer.receiver_instance_id,
            router_zid=producer.router_zid, connection_generation=producer.backend.connection_generation)
        self._loop = HandRetargetLoop(producer, self._source, publish=self._publish_hands,
                                      clock=clock, input_adapter=pico_frame_input,
                                      processed_input_sink=self._processed_input if audit_processed_inputs else None)
        try:
            self._subscriptions.append(session.declare_subscriber(topics.RAW_PICO_HAND_TRACKING, self._on_raw))
            self._subscriptions.append(session.declare_subscriber(topics.SESSION_STATE, self._on_state))
        except BaseException:
            self.close()
            raise

    @property
    def failure(self):
        with self._lock:
            fault = self._fault
        return fault or self._authority_failure() or self._source.failure or self._loop.failure

    def _authority_failure(self):
        return self._authority_guard.failure if self._authority_guard is not None else None

    def _on_raw(self, sample):
        with self._lock:
            if self._closed or self._fault:
                return
        self._source.ingest(_payload(sample))

    def _on_state(self, sample):
        with self._lock:
            if self._closed or self._fault:
                return
        try:
            value = _payload(sample)
            if isinstance(value, (bytes, bytearray, str)):
                value = strict_loads(value)
            self._loop.update_session(value)
        except (ValueError, TypeError) as exc:
            with self._lock:
                self._fault = 'PICO hand session input failed: ' + str(exc)

    def _send(self, topic, value):
        self._session.put(topic, json.dumps(value, allow_nan=False, separators=(',', ':')).encode(),
                          encoding='application/json')

    def _processed_input(self, row, snapshot):
        from ..hand_tracking.pico_retarget_audit import TOPIC, validate_consumed
        frame = row.payload
        self._audit_sequence += 1
        value = dict(schema_version=1, kind='pico_hand_retarget_consumed',
            router_zid=self._producer.router_zid, publisher_instance_id=self._producer.publisher_instance_id,
            receiver_instance_id=frame.receiver_instance_id, connection_generation=frame.connection_generation,
            receiver_frame_sequence=frame.receiver_frame_sequence, frame_association_id=frame.association_id,
            worker_sequence=row.sequence, processing_sequence=self._audit_sequence,
            received_timestamp_ns=frame.received_timestamp_ns, processed_timestamp_ns=self._clock(),
            valid_sides=snapshot['valid_sides'])
        self._send(TOPIC, validate_consumed(value))

    def _publish_hands(self, commands):
        with self._lock:
            if self._closed or self._fault or self._authority_failure():
                return
            for side, command in commands.items():
                self._send(topics.hand_command(side), command.to_dict())

    def tick(self):
        with self._lock:
            if self._closed:
                raise RuntimeError('PICO hand service closed')
        status = self._loop.status(self._clock(), ('left', 'right'), allow_tracking_hold=True)
        failure = self.failure
        if failure:
            status = replace(status, phase='fault', ready=False, healthy=False, error=failure)
        self._send(topics.PRODUCER_STATUS, status.to_dict())
        return status

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
        errors = []
        for subscription in reversed(self._subscriptions):
            try:
                subscription.undeclare()
            except Exception as exc:
                errors.append(exc)
        self._source.close()
        try:
            self._loop.close()
        except Exception as exc:
            errors.append(exc)
        if errors:
            raise RuntimeError('PICO hand component shutdown failed: ' + str(errors[0]))
