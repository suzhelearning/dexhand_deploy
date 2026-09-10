"""Opt-in passive PICO gesture observations; no operation bindings or authority."""
from dataclasses import asdict
import math

from .gesture_recognition import classify_hand
from .runtime import ObservationRuntime, pico_frame_observations

TOPIC = 'tianji/observation/operator/pico_gestures'
VERSION = 'pico21_geometry_v1'


def validate_observation(value):
    keys = {'schema_version', 'kind', 'algorithm', 'router_zid', 'publisher_instance_id',
            'receiver_instance_id', 'connection_generation', 'receiver_frame_sequence',
            'received_timestamp_ns', 'frame_association_id', 'hands'}
    if not isinstance(value, dict) or set(value) != keys:
        raise ValueError('invalid PICO gesture observation fields')
    if (type(value['schema_version']) is not int or value['schema_version'] != 1 or
            value['kind'] != 'pico_gestures' or value['algorithm'] != VERSION):
        raise ValueError('unsupported PICO gesture observation version')
    for key in ('router_zid', 'publisher_instance_id', 'receiver_instance_id', 'frame_association_id'):
        if not isinstance(value[key], str) or not value[key].strip():
            raise ValueError('gesture observation requires explicit identities')
    for key in ('connection_generation', 'receiver_frame_sequence', 'received_timestamp_ns'):
        if type(value[key]) is not int or not 0 <= value[key] < 2**63:
            raise ValueError('invalid gesture observation clock/sequence')
    if not isinstance(value['hands'], dict) or set(value['hands']) != {'left', 'right'}:
        raise ValueError('both gesture sides required')
    for hand in value['hands'].values():
        if (not isinstance(hand, dict) or set(hand) !=
                {'gesture', 'available', 'pinch_ratio', 'finger_extension'} or
                hand['gesture'] not in ('unknown', 'open', 'fist', 'pinch') or
                type(hand['available']) is not bool):
            raise ValueError('invalid hand gesture observation')
        if not hand['available']:
            if hand['gesture'] != 'unknown' or hand['pinch_ratio'] is not None or hand['finger_extension'] != []:
                raise ValueError('unavailable gesture must not contain fabricated metrics')
            continue
        metrics = hand['finger_extension']
        if not isinstance(metrics, list) or len(metrics) != 4:
            raise ValueError('four finger extension ratios required')
        if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0
               for v in [hand['pinch_ratio'], *metrics]):
            raise ValueError('finite nonnegative gesture metrics required')
    return value


class PicoGestureRuntime(ObservationRuntime):
    """Separate opt-in runtime, leaving original observation defaults intact."""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._gesture_previous = {}
        self._gesture_frame = None

    def ingest_pico(self, frame):
        result = super().ingest_pico(frame)
        identity = (frame.receiver_instance_id, frame.connection_generation)
        previous = self._gesture_frame
        if (previous is None or previous[:2] != identity or
                frame.receiver_frame_sequence != previous[2]+1 or
                not 0 < frame.received_timestamp_ns-previous[3] <= 200_000_000):
            self._gesture_previous.clear()
        self._gesture_frame = (*identity, frame.receiver_frame_sequence, frame.received_timestamp_ns)
        hands = {}
        for side, (hand, _) in pico_frame_observations(frame).items():
            gesture = classify_hand(hand.keypoints_m, valid=hand.valid,
                previous=self._gesture_previous.get(side, 'unknown'))
            self._gesture_previous[side] = gesture.gesture
            hands[side] = asdict(gesture)
            hands[side]['finger_extension'] = list(gesture.finger_extension)
        value = dict(schema_version=1, kind='pico_gestures', algorithm=VERSION,
            router_zid=self.router_zid, publisher_instance_id=self.publisher_instance_id,
            receiver_instance_id=frame.receiver_instance_id, connection_generation=frame.connection_generation,
            receiver_frame_sequence=frame.receiver_frame_sequence,
            received_timestamp_ns=frame.received_timestamp_ns, frame_association_id=frame.association_id,
            hands=hands)
        self._publish(TOPIC, validate_observation(value))
        return result
