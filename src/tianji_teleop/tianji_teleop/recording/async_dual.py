"""One isolated disk process and a bounded queue for new live sessions.

Overflow or disk failure invalidates the recording. The session owner observes
failure and stops the simulation; recording never obtains control authority.
No changes to legacy recorder scheduling or default file format.
"""
from copy import deepcopy
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread

from .writer_process import ProcessSessionWriter


class AsyncDualRecorder:
    _METHODS = frozenset({'append_dual_audit', 'append_raw_reference_tjvr', 'append_manus_callback',
        'append_arm_command', 'append_arm_state', 'append_hand_command', 'append_hand_state',
        'append_session_state', 'append_raw_pico'})

    def __init__(self, path, *, router_zid, metadata, capacity=4096,
                 source_type='vr_manus_sim', robot_model=None):
        if source_type not in ('vr_manus_sim', 'pico2_hands_sim'):
            raise ValueError('live recording requires an explicit simulation source')
        if robot_model is None and source_type == 'vr_manus_sim':
            robot_model = 'spark/marvin_m6_wuji2'
        if not isinstance(robot_model, str) or not robot_model.strip():
            raise ValueError('explicit robot_model required for this recording source')
        self._source_type = source_type
        if type(capacity) is not int or not 0 < capacity <= 65536:
            raise ValueError('record queue capacity must be in 1..65536')
        self._queue = Queue(maxsize=capacity)
        self._lock = Lock()
        self._stop = Event()
        self._closed = False
        self._failure = None
        self._complete = True
        self._writer = ProcessSessionWriter(path, source_type=source_type, robot_model=robot_model,
            router_zid=router_zid, schema_version='1.2', metadata=metadata, flush_interval_s=.5)
        self._thread = Thread(target=self._run, name='dual-session-recorder', daemon=True)
        try:
            self._thread.start()
        except BaseException:
            self._writer.abort()
            raise

    @property
    def failure(self):
        with self._lock:
            return self._failure

    def append(self, method, *args, **kwargs):
        if method not in self._METHODS:
            raise ValueError('unsupported live recording method')
        if ((self._source_type == 'pico2_hands_sim' and
             method in ('append_manus_callback', 'append_raw_reference_tjvr')) or
                (self._source_type == 'vr_manus_sim' and method == 'append_raw_pico')):
            raise ValueError('recording input mode does not match raw stream')
        # The owner may mutate model/message buffers as soon as this returns.
        item = (method, deepcopy(args), deepcopy(kwargs))
        with self._lock:
            if self._failure or self._closed:
                raise RuntimeError(self._failure or 'recording closed')
            try:
                self._queue.put_nowait(item)
            except Full as exc:
                self._failure = 'recording queue overflow; incomplete capture'
                self._stop.set()
                raise RuntimeError(self._failure) from exc

    def _run(self):
        pending = None
        try:
            while pending is not None or not self._stop.is_set() or not self._queue.empty():
                if self.failure:
                    break
                try:
                    item = pending if pending is not None else self._queue.get(timeout=.05)
                    pending = None
                except Empty:
                    self._writer.flush()
                    continue
                method, args, kwargs = item
                batch = [item]
                # Drain only an already available, consecutive audit prefix.
                # Never wait for a batch, cross another stream, or put an item
                # back at the queue tail (which would reorder the recording).
                if self._batchable_audit(item):
                    while len(batch) < 64:
                        try:
                            next_item = self._queue.get_nowait()
                        except Empty:
                            break
                        if not self._batchable_audit(next_item):
                            pending = next_item
                            break
                        batch.append(next_item)
                if len(batch) == 1:
                    getattr(self._writer, method)(*args, **kwargs)
                else:
                    self._writer.append_dual_audit_batch([
                        (entry[1][0], entry[1][1], entry[2]['received_timestamp_ns']) for entry in batch])
        except Exception as exc:
            with self._lock:
                self._failure = self._failure or f'recording write failed: {exc}'
        finally:
            try:
                if self._complete and not self.failure:
                    self._writer.close()
                else:
                    self._writer.abort()
            except Exception as exc:
                with self._lock:
                    self._failure = self._failure or f'recording close failed: {exc}'
                try:
                    self._writer.abort()
                except Exception:
                    # Persistent disk failure cannot be repaired here. Keep
                    # the original failure visible; never report success.
                    pass

    @staticmethod
    def _batchable_audit(item):
        method, args, kwargs = item
        return method == 'append_dual_audit' and len(args) == 2 and set(kwargs) == {'received_timestamp_ns'}

    def close(self, *, complete=True):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._complete = bool(complete)
            self._stop.set()
        self._thread.join(timeout=30)
        if self._thread.is_alive():
            # Do not race HDF5/Connection close from another thread. Terminate
            # only the owned peer to release a blocked send/recv, then let the
            # dispatcher abort and reap its own transport resources.
            with self._lock:
                self._failure = 'recording drain timeout; capture completion unverified'
                self._complete = False
            self._writer.interrupt()
            self._thread.join(timeout=5)
            raise RuntimeError(self._failure)
        if self.failure:
            raise RuntimeError(self.failure)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close(complete=exc_type is None)
