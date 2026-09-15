"""Cold-path owner of schema and disk process; no per-frame Python recording."""
from pathlib import Path
import socket
import subprocess
import tempfile
import time

import h5py

from .session_h5 import SessionH5Writer


class NativeRecordingOwner:
    def __init__(self, path, *, root, router_zid, robot_model, metadata):
        self.path = Path(path).resolve()
        binary = Path(root) / 'build/hdf5_recorder/tianji_hdf5_recorder'
        if not binary.is_file():
            raise RuntimeError('build-hdf5-recorder required for native recording')
        self.origin_ns = time.monotonic_ns()
        self._closed = False
        self._socket = self._process = self._errors = None
        self._failure = None
        # Exclusive creation; an existing recording is never touched.
        SessionH5Writer(self.path, source_type='vr_manus_sim', robot_model=robot_model,
                        router_zid=router_zid, schema_version='1.2', metadata=dict(metadata,
                        recording_adapter='cpp', recording_origin_ns=self.origin_ns)).abort()
        parent, child = socket.socketpair()
        try:
            self._errors = tempfile.TemporaryFile()
            self._process = subprocess.Popen([str(binary), str(self.path)], stdin=child,
                stdout=child, stderr=self._errors, close_fds=True)
            self._socket = parent
        except BaseException:
            parent.close()
            if self._errors is not None: self._errors.close()
            raise
        finally:
            child.close()

    def fileno(self):
        if self._socket is None: raise RuntimeError('recording descriptor already transferred')
        return self._socket.fileno()

    def release_descriptor(self):
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    @property
    def failure(self):
        if self._failure: return self._failure
        code = self._process.poll()
        return f'native recorder exited with status {code}' if code not in (None, 0) else None

    @property
    def statistics(self):
        return dict(backend='native_cpp', adapter='cpp', complete=self._closed and not self._failure)

    def close(self, *, complete):
        if self._closed: return
        self.release_descriptor()
        error = None
        try:
            try:
                code = self._process.wait(timeout=10 if complete else 2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                code = self._process.wait(timeout=5)
                error = 'native recorder did not close within deadline'
            if complete and code != 0:
                error = error or f'native recorder exited with status {code}'
            # Read only after the disk process exits. Never manufacture success;
            # external failure may revoke a disk-level complete marker.
            with h5py.File(self.path, 'r+') as file:
                if complete and not file.attrs.get('complete', False):
                    error = error or 'native recorder did not acknowledge complete file'
                if not complete or error:
                    file.attrs['complete'] = False
        except BaseException as exc:
            self._failure = str(exc) or 'recording close failed'
            raise
        finally:
            self._closed = True
            self._errors.close()
        self._failure = error or (None if complete else 'recording incomplete')
        if error: raise RuntimeError(error)
