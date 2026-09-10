"""Explicit PICO geometry to independent official single-side workers.

The caller owns worker startup/close and session authorization. Each worker
must be configured for its bound side. Using 63-point calls means a missing
side never advances its filter on fabricated zeros or cached geometry. The
original bridge observes the real receiver sequence gap when that side returns.
No Manus callback or device sequence is invented for recording.
"""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import math

from ..hand_tracking.models import PicoRawFrame
from ..hand_tracking.official_pico import (
    PICO_OFFICIAL_HAND2_GEOMETRY_SCALE,
    pico_official_hand2_retarget_input,
    pico_official_hand_observations,
)
from ..protocol.messages import HAND_JOINT_NAMES
from .hand_retarget import HandRetargetProducer


class PicoOfficialHandBackend:
    def __init__(self, clients, *, receiver_instance_id, connection_generation):
        if (not isinstance(clients, dict) or set(clients) != {'left', 'right'} or
                clients['left'] is clients['right']):
            raise ValueError('distinct explicit left/right official workers required')
        if (not isinstance(receiver_instance_id, str) or not receiver_instance_id.strip() or
                type(connection_generation) is not int or not 0 <= connection_generation < 2**63):
            raise ValueError('explicit PICO receiver and connection generation required')
        self.clients = dict(clients)
        self.receiver_instance_id = receiver_instance_id
        self.connection_generation = connection_generation
        # The two official processes have independent filters and IPC pipes.
        # Run their transactions concurrently so a bilateral frame pays the
        # slower side's latency, not the sum of both side latencies.
        self._worker_executor = ThreadPoolExecutor(
            max_workers=2, thread_name_prefix='pico-official-hand')
        self._closed = False
        self._sequence = -1
        self._timestamp = 0
        self._failure = None

    def retarget(self, frame, *, sequence=None, timestamp_ns=None):
        if self._closed:
            raise RuntimeError('PICO official hand backend closed')
        if self._failure:
            raise RuntimeError(self._failure)
        if (not isinstance(frame, PicoRawFrame) or
                frame.receiver_instance_id != self.receiver_instance_id or
                frame.connection_generation != self.connection_generation):
            raise ValueError('PICO source/generation mismatch; explicit new worker state required')
        if (not self._sequence < frame.receiver_frame_sequence < 2**63 - 1 or
                not 0 < frame.received_timestamp_ns < 2**63 or frame.received_timestamp_ns < self._timestamp):
            raise ValueError('PICO receive ordinal must increase and receive clock must not roll back')
        if sequence is not None or timestamp_ns is not None:
            if (type(sequence) is not int or sequence != frame.receiver_frame_sequence + 1 or
                    type(timestamp_ns) is not int or timestamp_ns != frame.received_timestamp_ns):
                raise ValueError('PICO producer metadata does not match the actual frame')
        observations = pico_official_hand_observations(frame)
        sequence = frame.receiver_frame_sequence + 1  # worker protocol requires positive sequences
        result = dict(schema_version=1, kind='pico_official_hand_result', algorithm='official_wuji_hand2',
            callback_sequence=sequence, timestamp_ns=frame.received_timestamp_ns,
            frame_association_id=frame.association_id, source='pico2')
        try:
            futures = {}
            for side in ('left', 'right'):
                result[side] = dict(valid=False, joint_names=list(HAND_JOINT_NAMES[side]), position_rad=[0.] * 20)
                if not observations[side].valid:
                    continue
                worker_points = pico_official_hand2_retarget_input(
                    observations[side].keypoints_m,
                    geometry_scale=PICO_OFFICIAL_HAND2_GEOMETRY_SCALE)
                futures[side] = self._worker_executor.submit(
                    self.clients[side].retarget, worker_points.ravel().tolist(),
                    sequence=sequence, timestamp_ns=frame.received_timestamp_ns)
            for side in ('left', 'right'):
                if side not in futures:
                    continue
                row = futures[side].result()
                other = 'right' if side == 'left' else 'left'
                hand = row[side]
                if (row['callback_sequence'] != sequence or row['timestamp_ns'] != frame.received_timestamp_ns or
                        hand['valid'] is not True or row[other]['valid'] is not False or
                        hand['joint_names'] != list(HAND_JOINT_NAMES[side]) or
                        len(hand['position_rad']) != 20 or any(type(v) not in (int, float) or not math.isfinite(v)
                                                             for v in hand['position_rad'])):
                    raise ValueError('official side worker returned an unassociated or wrong-side result')
                result[side] = deepcopy(hand)
        except Exception as exc:
            # One worker may already have advanced. Never retry the same frame
            # or emit a partially solved pair after a transport/backend error.
            self._failure = 'PICO official hand backend failed: ' + str(exc)
            raise RuntimeError(self._failure) from exc
        self._sequence, self._timestamp = frame.receiver_frame_sequence, frame.received_timestamp_ns
        return result

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._worker_executor.shutdown(wait=True)


class PicoOfficialHandProducer(HandRetargetProducer):
    """PICO frame adapter reusing the common hand authorization/command gate."""
    def update_frame(self, frame, *, now_ns):
        if not isinstance(frame, PicoRawFrame):
            raise TypeError('PICO producer requires a PicoRawFrame')
        return self.update_input(frame, sequence=frame.receiver_frame_sequence + 1,
            timestamp_ns=frame.received_timestamp_ns, receiver_instance_id=frame.receiver_instance_id,
            now_ns=now_ns)

    def update_input(self, frame, *, sequence, timestamp_ns, receiver_instance_id, now_ns):
        if not isinstance(frame, PicoRawFrame):
            raise TypeError('PICO producer requires a PicoRawFrame')
        if (type(sequence) is not int or sequence != frame.receiver_frame_sequence + 1 or
                type(timestamp_ns) is not int or timestamp_ns != frame.received_timestamp_ns or
                receiver_instance_id != frame.receiver_instance_id):
            return False
        if (frame.receiver_instance_id == self.receiver_instance_id and
                frame.connection_generation != self.backend.connection_generation):
            # Check before the ordinary sequence gate: TCP reconnection resets
            # receiver ordinals to zero, but cannot keep an old pending command.
            self.healthy = False
            self.reason = 'PICO connection generation changed; explicit algorithm state recreation required'
            self._pending = None
            return False
        return super().update_input(frame, sequence=sequence, timestamp_ns=timestamp_ns,
            receiver_instance_id=receiver_instance_id, now_ns=now_ns)
