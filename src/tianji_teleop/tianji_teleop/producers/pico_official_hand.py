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
from ..protocol.messages import HAND_JOINT_NAMES, SessionState
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
        # The two Python official processes have independent filters and IPC
        # pipes. Run their synchronous transactions concurrently. Native
        # scheduler clients own their reader threads and do not need this
        # executor at all.
        self._worker_executor = None
        self._closed = False
        self._sequence = -1
        self._timestamp = 0
        self._failure = None
        self._last_session_state = None
        self._native_epoch = 1
        self._native_clients = tuple(client for client in self.clients.values()
                                      if callable(getattr(client, 'set_execution_epoch', None)))
        if self._native_clients and len(self._native_clients) != len(self.clients):
            raise ValueError('both PICO official sides must use the same native hand scheduler')
        self.async_retarget = all(bool(getattr(client, 'async_retarget', False)) and
                                  callable(getattr(client, 'submit_retarget', None)) and
                                  callable(getattr(client, 'poll_retarget', None))
                                  for client in self.clients.values())
        if not self.async_retarget:
            self._worker_executor = ThreadPoolExecutor(
                max_workers=2, thread_name_prefix='pico-official-hand')
        self._async_pending = None
        self._async_waiting = None
        self._async_side_results = {'left': None, 'right': None}
        self._native_epoch_set_since_session = False

    def _validate_frame(self, frame, *, sequence=None, timestamp_ns=None):
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
        return (pico_official_hand_observations(frame), frame.receiver_frame_sequence + 1,
                frame.received_timestamp_ns)

    @staticmethod
    def _empty_result(sequence, timestamp_ns, frame):
        result = dict(schema_version=1, kind='pico_official_hand_result', algorithm='official_wuji_hand2',
            callback_sequence=sequence, timestamp_ns=frame.received_timestamp_ns,
            frame_association_id=frame.association_id, source='pico2')
        for side in ('left', 'right'):
            result[side] = dict(valid=False, joint_names=list(HAND_JOINT_NAMES[side]), position_rad=[0.] * 20)
        return result

    @staticmethod
    def _side_points(observation):
        return pico_official_hand2_retarget_input(
            observation.keypoints_m, geometry_scale=PICO_OFFICIAL_HAND2_GEOMETRY_SCALE).ravel().tolist()

    @staticmethod
    def _validate_side_result(row, side, sequence, timestamp_ns):
        other = 'right' if side == 'left' else 'left'
        hand = row[side]
        if (row['callback_sequence'] != sequence or row['timestamp_ns'] != timestamp_ns or
                hand['valid'] is not True or row[other]['valid'] is not False or
                hand['joint_names'] != list(HAND_JOINT_NAMES[side]) or
                len(hand['position_rad']) != 20 or any(type(v) not in (int, float) or not math.isfinite(v)
                                                     for v in hand['position_rad'])):
            raise ValueError('official side worker returned an unassociated or wrong-side result')

    def retarget(self, frame, *, sequence=None, timestamp_ns=None):
        if self.async_retarget:
            raise RuntimeError('PICO async hand backend requires submit_retarget/poll_retarget')
        observations, sequence, timestamp_ns = self._validate_frame(
            frame, sequence=sequence, timestamp_ns=timestamp_ns)
        result = self._empty_result(sequence, timestamp_ns, frame)
        try:
            futures = {}
            for side in ('left', 'right'):
                if not observations[side].valid:
                    continue
                futures[side] = self._worker_executor.submit(
                    self.clients[side].retarget, self._side_points(observations[side]),
                    sequence=sequence, timestamp_ns=timestamp_ns)
            for side in ('left', 'right'):
                if side not in futures:
                    continue
                row = futures[side].result()
                self._validate_side_result(row, side, sequence, timestamp_ns)
                result[side] = deepcopy(row[side])
        except Exception as exc:
            # One worker may already have advanced. Never retry the same frame
            # or emit a partially solved pair after a transport/backend error.
            self._failure = 'PICO official hand backend failed: ' + str(exc)
            raise RuntimeError(self._failure) from exc
        self._sequence, self._timestamp = frame.receiver_frame_sequence, timestamp_ns
        return result

    def submit_retarget(self, frame, *, sequence=None, timestamp_ns=None):
        """Admit a PICO frame and let both native side schedulers solve later."""
        if not self.async_retarget:
            raise RuntimeError('PICO Python hand backend is synchronous')
        observations, sequence, timestamp_ns = self._validate_frame(
            frame, sequence=sequence, timestamp_ns=timestamp_ns)
        valid_sides = [side for side in ('left', 'right') if observations[side].valid]
        pending = dict(frame=frame, sequence=sequence, timestamp_ns=timestamp_ns,
                       valid_sides=valid_sides, observations=observations)
        try:
            if self._async_pending is None:
                self._dispatch_async(pending)
            else:
                # Both workers must finish the same frame. Coalesce only the
                # waiting input, never supersede one side of an in-flight pair.
                self._async_waiting = pending
        except Exception as exc:
            self._failure = 'PICO official hand backend failed: ' + str(exc)
            raise RuntimeError(self._failure) from exc
        self._sequence, self._timestamp = frame.receiver_frame_sequence, timestamp_ns
        return dict(sequence=sequence, timestamp_ns=timestamp_ns, valid_sides=valid_sides)

    def _dispatch_async(self, pending):
        for side in pending['valid_sides']:
            self.clients[side].submit_retarget(
                self._side_points(pending['observations'][side]),
                sequence=pending['sequence'], timestamp_ns=pending['timestamp_ns'])
        self._async_pending = pending

    def poll_retarget(self):
        """Coalesce native side results and emit only a matched PICO frame."""
        if not self.async_retarget:
            return None
        if self._closed:
            raise RuntimeError('PICO official hand backend closed')
        if self._failure:
            raise RuntimeError(self._failure)
        pending = self._async_pending
        if pending is None:
            # Still drain both pipes so a session transition or a temporarily
            # invalid PICO side cannot leave an old native result queued behind
            # the next valid frame. There is no frame to associate it with.
            for side in ('left', 'right'):
                self.clients[side].poll_retarget()
                self._async_side_results[side] = None
            return None
        try:
            for side in ('left', 'right'):
                row = self.clients[side].poll_retarget()
                if row is not None:
                    self._async_side_results[side] = row
            for side in pending['valid_sides']:
                row = self._async_side_results[side]
                if (row is None or row['callback_sequence'] != pending['sequence'] or
                        row['timestamp_ns'] != pending['timestamp_ns']):
                    return None
            result = self._empty_result(pending['sequence'], pending['timestamp_ns'], pending['frame'])
            stale_pair = any(self._async_side_results[side].get('native_scheduler', {}).get('status') == 3
                             for side in pending['valid_sides'])
            for side in pending['valid_sides']:
                row = self._async_side_results[side]
                if stale_pair:
                    continue
                self._validate_side_result(row, side, pending['sequence'], pending['timestamp_ns'])
                result[side] = deepcopy(row[side])
            side_scheduler = [self._async_side_results[side].get('native_scheduler')
                              for side in pending['valid_sides']]
            if side_scheduler:
                if any(not isinstance(item, dict) or
                       any(item.get(key) is None or item.get(key) != side_scheduler[0].get(key)
                           for key in ('phase', 'epoch')) or
                       item.get('status') not in (1, 2, 3)
                       for item in side_scheduler):
                    raise ValueError('native scheduler pair execution metadata mismatch')
                result['native_scheduler'] = {
                    key: side_scheduler[0][key] for key in ('phase', 'status', 'epoch')}
                if stale_pair:
                    result['native_scheduler']['status'] = 3
            for side in ('left', 'right'):
                if side in pending['valid_sides']:
                    self._async_side_results[side] = None
            self._async_pending = None
            waiting, self._async_waiting = self._async_waiting, None
            if waiting is not None:
                self._dispatch_async(waiting)
            return result
        except Exception as exc:
            self._failure = 'PICO official hand backend failed: ' + str(exc)
            raise RuntimeError(self._failure) from exc

    def update_session(self, state):
        """Forward the validated coordinator heartbeat to optional workers.

        The legacy OfficialHandClient has no session method and is therefore
        unchanged. Native scheduler clients use this hook to gate their
        fixed-rate output with the same session sequence as the Python owner.
        A return-to-idle transition is also the hand-side rearm barrier for
        the PICO2 component: advance the native epoch exactly once before the
        idle heartbeat reaches the scheduler. This clears native filter state
        without changing the Python worker lifecycle.
        """
        value = state if isinstance(state, SessionState) else SessionState.from_dict(state)
        next_epoch = self._native_epoch
        if (self._native_clients and self._last_session_state is not None and
                value.state == 'idle' and self._last_session_state != 'idle' and
                not self._native_epoch_set_since_session):
            next_epoch += 1
        if next_epoch != self._native_epoch:
            for client in self._native_clients:
                client.set_execution_epoch(next_epoch)
            self._native_epoch = next_epoch
        self._native_epoch_set_since_session = False
        for client in self.clients.values():
            update = getattr(client, 'update_session', None)
            if callable(update) and not update(value):
                return False
        changed = self._last_session_state != value.state
        self._last_session_state = value.state
        if changed and self.async_retarget:
            self._async_pending = None
            self._async_waiting = None
            self._async_side_results = {'left': None, 'right': None}
        return True

    def set_execution_epoch(self, execution_epoch):
        """Set the next native hand epoch at an arm-side rearm barrier.

        Python official workers intentionally do not expose this operation.
        Keeping this adapter method a no-op for them lets the shared live
        simulation call the barrier without changing the legacy path.
        """
        if type(execution_epoch) is not int or not 0 < execution_epoch < 2**63:
            raise ValueError('execution_epoch must be a positive int64')
        if not self._native_clients:
            return
        if execution_epoch < self._native_epoch:
            raise ValueError('execution_epoch cannot move backwards')
        for client in self._native_clients:
            client.set_execution_epoch(execution_epoch)
        if execution_epoch != self._native_epoch and self.async_retarget:
            self._async_pending = None
            self._async_waiting = None
            self._async_side_results = {'left': None, 'right': None}
        self._native_epoch = execution_epoch
        self._native_epoch_set_since_session = True

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._worker_executor is not None:
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
