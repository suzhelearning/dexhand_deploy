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
        'append_session_state', 'append_raw_pico', 'append_live_cycle_snapshot',
        'append_live_cycle_snapshot_batch'})

    def __init__(self, path, *, router_zid, metadata, capacity=16384,
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
        self._high_water = 0
        self._accepted = 0
        self._processed = 0
        writer_type = ProcessSessionWriter
        if source_type == 'vr_manus_sim':
            from .native_session_h5 import NativeSessionH5Writer
            writer_type = NativeSessionH5Writer
        self._writer = writer_type(path, source_type=source_type, robot_model=robot_model,
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

    @property
    def statistics(self):
        with self._lock:
            return dict(backend='native_cpp' if self._source_type == 'vr_manus_sim' else 'python',
                accepted=self._accepted, processed=self._processed,
                queue_depth=self._queue.qsize(), queue_high_water=self._high_water,
                queue_capacity=self._queue.maxsize,
                blocks_written=getattr(self._writer, 'blocks_written', None),
                bytes_written=getattr(self._writer, 'bytes_written', None),
                max_block_latency_s=getattr(self._writer, 'max_block_latency_s', None))

    def _enqueue(self, method, args, kwargs, *, copy_payload):
        if method not in self._METHODS:
            raise ValueError('unsupported live recording method')
        if ((self._source_type == 'pico2_hands_sim' and
             method in ('append_manus_callback', 'append_raw_reference_tjvr')) or
                (self._source_type == 'vr_manus_sim' and method == 'append_raw_pico')):
            raise ValueError('recording input mode does not match raw stream')
        # The owner may mutate model/message buffers as soon as this returns.
        item = (method, deepcopy(args), deepcopy(kwargs)) if copy_payload else (method, args, kwargs)
        with self._lock:
            if self._failure or self._closed:
                raise RuntimeError(self._failure or 'recording closed')
            try:
                self._queue.put_nowait(item)
                self._accepted += 1
                self._high_water = max(self._high_water, self._queue.qsize())
            except Full as exc:
                self._failure = 'recording queue overflow; incomplete capture'
                self._stop.set()
                raise RuntimeError(self._failure) from exc

    def append(self, method, *args, **kwargs):
        """Copy an externally owned payload before it enters the writer queue."""
        self._enqueue(method, args, kwargs, copy_payload=True)

    def append_owned(self, method, *args, **kwargs):
        """Transfer an already immutable/owned payload without another copy.

        Live control snapshots are never mutated after publication.  This
        boundary is restricted to callers that own the payload lifetime;
        ordinary acquisition callbacks must continue using ``append``.
        """
        self._enqueue(method, args, kwargs, copy_payload=False)

    def append_live_cycle_snapshot(self, snapshot, *, run_id):
        """Write one complete live control snapshot in one writer RPC.

        The snapshot is already owned by the side-effect worker.  Keeping the
        object intact until the HDF5 child handles it avoids several parent
        thread pickle/response round trips per control cycle.
        """
        if not isinstance(run_id, str) or not run_id.strip():
            raise ValueError('live cycle recording run_id must be nonempty')
        self._enqueue('append_live_cycle_snapshot', (snapshot,),
                      {'run_id': run_id}, copy_payload=False)

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
                elif self._batchable_live_cycle(item):
                    while len(batch) < 16:
                        try:
                            next_item = self._queue.get_nowait()
                        except Empty:
                            break
                        if not self._batchable_live_cycle(next_item):
                            pending = next_item
                            break
                        batch.append(next_item)
                if len(batch) == 1:
                    getattr(self._writer, method)(*args, **kwargs)
                elif self._batchable_audit(batch[0]):
                    self._writer.append_dual_audit_batch([
                        (entry[1][0], entry[1][1], entry[2]['received_timestamp_ns']) for entry in batch])
                else:
                    self._writer.append_live_cycle_snapshot_batch([
                        (entry[1][0], entry[2]['run_id']) for entry in batch])
                with self._lock:
                    self._processed += len(batch)
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

    @staticmethod
    def _batchable_live_cycle(item):
        method, args, kwargs = item
        return method == 'append_live_cycle_snapshot' and len(args) == 1 and set(kwargs) == {'run_id'}

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
