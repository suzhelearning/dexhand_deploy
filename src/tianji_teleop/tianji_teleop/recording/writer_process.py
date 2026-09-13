"""Private spawned HDF5 owner; no device, router or control authority.

The pipe is exclusively inherited by our own child, never a network endpoint.
Only the fixed recording method allowlist is dispatched. Spawn avoids forking
the existing Zenoh/native threads. A failed write latches incomplete state.
"""
import multiprocessing
import os


_METHODS = frozenset({'append_dual_audit', 'append_dual_audit_batch',
    'append_raw_reference_tjvr', 'append_manus_callback', 'append_arm_command',
    'append_arm_state', 'append_hand_command', 'append_hand_state',
    'append_session_state', 'append_raw_pico', 'append_live_cycle_snapshot',
    'append_live_cycle_snapshot_batch',
    'flush', 'close', 'abort'})


def _writer_main(connection, path, options):
    from .session_h5 import SessionH5Writer
    writer = None
    failed = False
    try:
        try:
            writer_type = SessionH5Writer
            if options.get('source_type') == 'vr_manus_sim':
                from .buffered_session_h5 import BufferedSessionH5Writer
                writer_type = BufferedSessionH5Writer
            writer = writer_type(path, **options)
        except Exception as exc:
            connection.send((False, type(exc).__name__, str(exc)))
            return
        connection.send((True, os.getpid()))
        while True:
            method, args, kwargs = connection.recv()
            try:
                if method not in _METHODS:
                    raise ValueError('unsupported recording operation')
                if failed and method != 'abort':
                    raise RuntimeError('recording previously failed; abort required')
                getattr(writer, method)(*args, **kwargs)
            except Exception as exc:
                failed = True
                connection.send((False, type(exc).__name__, str(exc)))
            else:
                connection.send((True, None))
                if method in ('close', 'abort'):
                    return
    except (EOFError, BrokenPipeError, OSError):
        pass
    finally:
        if writer is not None:
            writer.abort()  # idempotent after successful close
        connection.close()


class ProcessSessionWriter:
    """Serialized RPC, owned by AsyncDualRecorder's disk dispatcher thread."""
    def __init__(self, path, **options):
        context = multiprocessing.get_context('spawn')
        self._connection, child = context.Pipe()
        self._process = context.Process(target=_writer_main, args=(child, path, options),
                                        name='dual-hdf5-writer', daemon=True)
        self._closed = False
        self._failure = None
        try:
            self._process.start()
            child.close()
            response = self._receive()
            if not response[0]:
                error = FileExistsError if response[1] == 'FileExistsError' else RuntimeError
                raise error('recording startup failed: ' + response[2])
            self.worker_pid = response[1]
        except BaseException:
            child.close()
            self._shutdown()
            raise

    def _receive(self):
        if not self._connection.poll(20):
            raise TimeoutError('recording worker response timeout')
        return self._connection.recv()

    def _call(self, method, *args, **kwargs):
        if self._closed or (self._failure and method != 'abort'):
            raise RuntimeError(self._failure or 'recording worker closed')
        try:
            if not self._process.is_alive():
                raise RuntimeError('recording worker exited')
            self._connection.send((method, args, kwargs))
            response = self._receive()
            if not response[0]:
                raise RuntimeError(response[1] + ': ' + response[2])
        except Exception as exc:
            self._failure = 'recording write failed: ' + str(exc)
            raise RuntimeError(self._failure) from exc

    def __getattr__(self, method):
        if method not in _METHODS:
            raise AttributeError(method)
        return lambda *args, **kwargs: self._call(method, *args, **kwargs)

    def interrupt(self):
        """Cancel a stuck dispatcher without touching its Connection or HDF5.

        Only the owned child is killed. Closing the peer releases a blocked
        Pipe.send/recv; the dispatcher remains responsible for transport close
        and process reap. Capture integrity is unverified after interruption.
        """
        if self._closed:
            return
        self._failure = self._failure or 'recording interrupted; capture completion unverified'
        if self._process.is_alive():
            self._process.kill()

    def _shutdown(self):
        self._closed = True
        self._connection.close()
        if self._process.pid is not None:
            self._process.join(2)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(2)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(2)

    def close(self):
        if self._closed:
            return
        # A failed close leaves the process available for explicit abort.
        self._call('close')
        self._shutdown()

    def abort(self):
        if self._closed:
            return
        try:
            if self._process.is_alive():
                self._call('abort')
        finally:
            self._shutdown()
