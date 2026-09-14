"""Reference receiver core and versioned, raw-preserving transport envelope."""
import base64
import binascii
from dataclasses import dataclass
from threading import Lock
from typing import Callable

from .reference_tjvr import ReferenceTjvrFrame, parse_reference_tjvr_packet
from .reference_tjvr_stream import ReferenceTjvrStreamGate, StreamDecision


def _integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError(f'{name} must be an integer >= {minimum}')
    return value


@dataclass(frozen=True)
class ReceivedTjvrFrame:
    observation: ReferenceTjvrFrame
    stream_discontinuity: bool = False
    resynchronization_generation: int = 0

    def __post_init__(self):
        if not isinstance(self.observation, ReferenceTjvrFrame):
            raise ValueError('observation must be a reference TJVR frame')
        if type(self.stream_discontinuity) is not bool:
            raise ValueError('stream_discontinuity must be boolean')
        _integer(self.resynchronization_generation, 'resynchronization_generation')

    def to_dict(self):
        f = self.observation.frame
        return dict(schema_version=1, kind='tjvr_upper_limb_observation',
            raw_packet_base64=base64.b64encode(f.raw_packet).decode('ascii'),
            received_timestamp_ns=f.received_timestamp_ns,
            receiver_instance_id=f.receiver_instance_id,
            receiver_frame_sequence=f.receiver_frame_sequence,
            stream_discontinuity=self.stream_discontinuity,
            resynchronization_generation=self.resynchronization_generation)

    @classmethod
    def from_dict(cls, value):
        fields = {'schema_version', 'kind', 'raw_packet_base64', 'received_timestamp_ns',
            'receiver_instance_id', 'receiver_frame_sequence', 'stream_discontinuity',
            'resynchronization_generation'}
        if not isinstance(value, dict) or set(value) != fields:
            raise ValueError('invalid TJVR observation fields')
        if type(value['schema_version']) is not int or value['schema_version'] != 1:
            raise ValueError('unsupported TJVR observation schema')
        if value['kind'] != 'tjvr_upper_limb_observation':
            raise ValueError('invalid TJVR observation kind')
        _integer(value['received_timestamp_ns'], 'received_timestamp_ns', 1)
        _integer(value['receiver_frame_sequence'], 'receiver_frame_sequence', 1)
        encoded = value['raw_packet_base64']
        if not isinstance(encoded, str) or len(encoded) > 876:
            raise ValueError('invalid raw TJVR packet encoding')
        try:
            packet = base64.b64decode(encoded, validate=True)
        except (ValueError, binascii.Error) as exc:
            raise ValueError('invalid raw TJVR packet encoding') from exc
        observation = parse_reference_tjvr_packet(packet,
            receiver_instance_id=value['receiver_instance_id'],
            receiver_frame_sequence=value['receiver_frame_sequence'],
            received_timestamp_ns=value['received_timestamp_ns'])
        return cls(observation, value['stream_discontinuity'],
                   value['resynchronization_generation'])


class ReferenceTjvrReceiver:
    """Single ingestion order, latest-only consumption, no device authority.

    raw_sink runs after decoding and before stream acceptance, matching the
    original recorder boundary. Sink errors propagate to the owner rather than
    silently continuing an incomplete recording. The optional decision sink
    runs once per decoded packet, before exposing new latest input. Both sinks
    must be bounded/nonblocking; after either fails the owner must stop this
    receiver. Use a new instance on restart.
    """
    def __init__(self, receiver_instance_id: str, max_position_jump_m: float,
                 max_orientation_jump_rad: float,
                 raw_sink: Callable[[bytes, int], None] | None = None,
                 raw_frame_sink: Callable[[ReferenceTjvrFrame], None] | None = None,
                 decision_sink=None, target_source='packet'):
        if target_source not in ('packet', 'mapped_corrected_palm'):
            raise ValueError('unknown TJVR target source')
        self._target_source = target_source
        if not isinstance(receiver_instance_id, str) or not receiver_instance_id.strip():
            raise ValueError('receiver_instance_id must be nonempty')
        self._instance = receiver_instance_id
        self._gate = ReferenceTjvrStreamGate(max_position_jump_m, max_orientation_jump_rad)
        self._sink = raw_sink
        self._frame_sink = raw_frame_sink
        self._decision_sink = decision_sink
        # Serialize ingest calls and stream-gate state without extending the
        # state lock across user callbacks.  The UDP source has one ingest
        # thread today, but keeping this boundary explicit preserves the
        # direct receiver API's ordering if callers use it concurrently.
        self._ingest_lock = Lock()
        self._lock = Lock()
        self._latest = None
        self._generation = 0
        self._stats = dict(datagrams=0, malformed=0, accepted=0, rejected=0, superseded=0)

    def ingest(self, packet: bytes, received_timestamp_ns: int) -> StreamDecision | None:
        _integer(received_timestamp_ns, 'received_timestamp_ns', 1)
        with self._ingest_lock:
            with self._lock:
                self._stats['datagrams'] += 1
                receiver_frame_sequence = self._stats['datagrams']
            try:
                observation = parse_reference_tjvr_packet(packet,
                    receiver_instance_id=self._instance,
                    receiver_frame_sequence=receiver_frame_sequence,
                    received_timestamp_ns=received_timestamp_ns)
            except ValueError:
                with self._lock:
                    self._stats['malformed'] += 1
                return None

            # These callbacks may perform a deep copy, enqueue recording data,
            # or publish diagnostics.  They must not stop the control owner
            # from consuming the previous latest frame.
            if self._sink is not None:
                self._sink(observation.frame.raw_packet, received_timestamp_ns)
            if self._frame_sink is not None:
                self._frame_sink(observation)
            gate_frame = observation.frame
            if self._target_source == 'mapped_corrected_palm':
                from .mapped_palm_input import select_mapped_palm_frame
                try:
                    gate_frame = select_mapped_palm_frame(gate_frame)
                except ValueError:
                    with self._lock:
                        self._stats['malformed'] += 1
                    return None
            decision = self._gate.evaluate(gate_frame)
            with self._lock:
                generation = self._generation + int(decision.stream_discontinuity)
            if self._decision_sink is not None:
                self._decision_sink(observation, decision, generation)
            if not decision.accepted:
                with self._lock:
                    self._stats['rejected'] += 1
                return decision
            with self._lock:
                if decision.stream_discontinuity:
                    self._generation += 1
                if self._latest is not None:
                    self._stats['superseded'] += 1
                self._latest = ReceivedTjvrFrame(observation, decision.stream_discontinuity,
                                                self._generation)
                self._stats['accepted'] += 1
                return decision

    def try_read_latest(self) -> ReceivedTjvrFrame | None:
        with self._lock:
            latest, self._latest = self._latest, None
            return latest

    def stats(self) -> dict[str, int]:
        with self._lock:
            return self._stats.copy()
