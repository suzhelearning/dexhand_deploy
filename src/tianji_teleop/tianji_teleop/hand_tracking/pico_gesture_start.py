"""Opt-in bilateral-open START request; never toggle, home or authorize directly."""
from dataclasses import asdict
import json
from threading import RLock
import time

from ..protocol.messages import strict_loads
from .operator_input import OperatorEdgeFilter, OperatorObservation
from .pico_gestures import TOPIC, validate_observation

RESULT_TOPIC = 'tianji/observation/operator/pico_start_result'


def validate_result(value):
    keys = {'schema_version', 'kind', 'router_zid', 'publisher_instance_id', 'timestamp_ns',
            'event', 'request_forwarded', 'reason', 'quality_kind'}
    if (not isinstance(value, dict) or set(value) != keys or
            type(value['schema_version']) is not int or value['schema_version'] != 1 or
            value['kind'] != 'pico_gesture_start_result' or
            type(value['request_forwarded']) is not bool or
            value['quality_kind'] != 'geometry_available_only' or
            value['reason'] != ('forwarded_to_session_gate' if value['request_forwarded'] else 'session_gate_rejected')):
        raise ValueError('invalid PICO gesture request result')
    for key in ('router_zid', 'publisher_instance_id'):
        if not isinstance(value[key], str) or not value[key].strip():
            raise ValueError('gesture result requires explicit identities')
    event = value['event']
    if (not isinstance(event, dict) or set(event) != {'source', 'side', 'sequence', 'epoch',
            'receive_time_ns', 'action', 'edge', 'confidence'} or
            event['edge'] != 'rising' or event['action'] != 'start_request' or event['side'] != 'both'):
        raise ValueError('invalid gesture start event')
    observation = OperatorObservation(**{k: v for k, v in event.items() if k != 'edge'},
        valid=True, available=True, pressed=True)
    if (type(value['timestamp_ns']) is not int or
            not observation.receive_time_ns <= value['timestamp_ns'] < 2**63):
        raise ValueError('invalid gesture result timestamp')
    return value


def bind_from_environment(environment, *, session, node):
    mode = environment.get('TIANJI_PICO_OPERATOR_INPUT', 'keyboard')
    if mode == 'keyboard':
        return None
    if (mode != 'gesture' or environment.get('TIANJI_REQUIRED_CAPABILITY') != 'simulation' or
            environment.get('TIANJI_REQUIRED_OBSERVATION_PROFILE') != 'pico'):
        raise ValueError('gesture START binding requires explicit managed PICO simulation')
    resolved = strict_loads(environment.get('TIANJI_RESOLVED_DUAL_SESSION', '{}'))
    if (not isinstance(resolved, dict) or resolved.get('profile') != 'pico2_hands_sim' or
            not isinstance(resolved.get('config'), dict) or
            resolved['config'].get('operator_input') != 'gesture' or
            resolved['config'].get('input_mode') != 'pico2_hands'):
        raise ValueError('gesture binding requires matching resolved PICO session')
    return PicoGestureStartBinding(session,
        router_zid=environment.get('TIANJI_ROUTER_ZID'),
        publisher_instance_id=environment.get('TIANJI_COMPONENT_INSTANCE_ID'),
        observation_instance_id=environment.get('TIANJI_OBSERVATION_PUBLISHER_INSTANCE_ID'),
        connection_generation=1,
        request_start=lambda: node.request_start(reason='pico_gesture_bilateral_open'),
        armed=lambda: node.phase == 'armed')


class PicoGestureStartBinding:
    def __init__(self, session, *, router_zid, publisher_instance_id, observation_instance_id,
                 connection_generation, request_start, armed, clock=time.monotonic_ns):
        for identity in (router_zid, publisher_instance_id, observation_instance_id):
            if not isinstance(identity, str) or not identity.strip() or '/' in identity:
                raise ValueError('explicit gesture session identities required')
        if not callable(request_start) or not callable(armed) or not callable(clock):
            raise ValueError('explicit session request, armed predicate and clock required')
        self._filter = OperatorEdgeFilter(source=observation_instance_id, side='both',
            action='start_request', epoch=connection_generation, freshness_ns=200_000_000,
            stable_ns=800_000_000)
        self._session, self._clock = session, clock
        self._router, self._publisher = router_zid, publisher_instance_id
        self._observation, self._generation = observation_instance_id, connection_generation
        self._request, self._armed = request_start, armed
        self._lock = RLock()
        self._closed = False
        self._subscription = session.declare_subscriber(TOPIC, self._receive)

    def _receive(self, sample):
        with self._lock:
            if self._closed:
                return
            try:
                value = getattr(sample, 'payload', sample)
                if hasattr(value, 'to_bytes'):
                    value = value.to_bytes()
                if isinstance(value, (str, bytes, bytearray)):
                    value = strict_loads(value)
                value = validate_observation(value)
            except (ValueError, TypeError):
                self._filter.invalidate()
                return
            if (value['router_zid'] != self._router or
                    value['publisher_instance_id'] != self._observation or
                    value['receiver_instance_id'] != self._observation or
                    value['connection_generation'] != self._generation):
                return
            hands = value['hands'].values()
            available = all(hand['available'] for hand in hands)
            observation = OperatorObservation(source=self._observation, side='both',
                sequence=value['receiver_frame_sequence'], epoch=self._generation,
                receive_time_ns=value['received_timestamp_ns'], valid=bool(self._armed()),
                available=available, action='start_request',
                pressed=all(hand['gesture'] == 'open' for hand in hands),
                confidence=1. if available else 0.)  # availability, not a learned probability
            now = self._clock()
            event = self._filter.update(observation, now_ns=now)
            if event is None:
                return
            # The callback is the existing node.request_start: it checks armed,
            # input/calibration readiness and forwards to coordinator authority.
            forwarded = bool(self._request())
            result = dict(schema_version=1, kind='pico_gesture_start_result',
                router_zid=self._router, publisher_instance_id=self._publisher,
                timestamp_ns=now, event=asdict(event), request_forwarded=forwarded,
                reason='forwarded_to_session_gate' if forwarded else 'session_gate_rejected',
                quality_kind='geometry_available_only')
            self._session.put(RESULT_TOPIC, json.dumps(result, allow_nan=False).encode(),
                              encoding='application/json')

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self._subscription.undeclare()
