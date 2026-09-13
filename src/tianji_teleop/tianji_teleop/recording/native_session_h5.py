"""Canonical validation/columns with an exclusive C++ streaming HDF5 owner.

Python creates the empty schema once, then closes HDF5 before native startup.
No h5py calls occur per sample. Only bounded column blocks cross the pipe.
"""
from dataclasses import dataclass
import json
import os
from pathlib import Path
import selectors
import struct
import subprocess
import time

import h5py
import numpy as np

from .buffered_session_h5 import BufferedSessionH5Writer
from .session_h5 import SessionH5Writer
from ..worker_environment import isolated_worker_environment


def native_recorder_path():
    return Path(__file__).resolve().parents[4] / 'build/hdf5_recorder/tianji_hdf5_recorder'


@dataclass
class _Column:
    name: str
    dtype: object
    shape: tuple


class _Group:
    def __init__(self, group):
        self.name = group.name
        self.attrs = dict(group.attrs)
        self.children = {}
        for name, child in group.items():
            self.children[name] = (_Group(child) if isinstance(child, h5py.Group)
                                   else _Column(child.name, child.dtype, child.shape))

    def __getitem__(self, key):
        node = self
        for part in key.strip('/').split('/'):
            node = node.children[part]
        return node

    def __contains__(self, key):
        try:
            self[key]
            return True
        except KeyError:
            return False


class _Process(subprocess.Popen):
    # Same owned-process lifecycle vocabulary as the previous spawned writer.
    def is_alive(self):
        return self.poll() is None

    def join(self, timeout=None):
        try:
            self.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pass


class NativeSessionH5Writer(BufferedSessionH5Writer):
    def __init__(self, path, *, executable=None, **options):
        executable = Path(executable) if executable else native_recorder_path()
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise RuntimeError('native HDF5 recorder missing; run pixi run build-hdf5-recorder')
        if options.get('source_type') != 'vr_manus_sim' or options.get('overwrite', False):
            raise ValueError('native recording requires exclusive vr_manus_sim capture')
        self._columns = {}
        self._attributes = {}
        self._buffer_full = False
        self._buffer_bytes = 0
        self._failure = None
        self._process = None
        self._native_finalized = False
        self.blocks_written = 0
        self.bytes_written = 0
        self.max_block_latency_s = 0.
        # Reuse schema construction, not BufferedSessionH5Writer's HDF5 cache.
        SessionH5Writer.__init__(self, path, **options)
        file = self._file
        try:
            self._file = _Group(file)
        finally:
            file.close()
        try:
            self._process = _Process([str(executable), str(self.path.resolve())],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0,
                env=isolated_worker_environment())
            self.worker_pid = self._process.pid
            os.set_blocking(self._process.stdin.fileno(), False)
            os.set_blocking(self._process.stdout.fileno(), False)
            self._exchange(b'', expected=b'READY 1\n')
        except BaseException:
            self._shutdown()
            raise

    def _append(self, dataset, value):
        if self._closed or self._failure:
            raise RuntimeError(self._failure or 'recording closed')
        super()._append(dataset, value)
        self._buffer_bytes += len(value) * 4 if isinstance(value, str) else np.asarray(value).nbytes
        self._buffer_full |= self._buffer_bytes >= 8 * 1024 * 1024

    def _set_names(self, group, names, logical_id):
        value = json.dumps(list(names), ensure_ascii=False, separators=(',', ':'), allow_nan=False)
        for key, text in (('joint_names', value), ('names', value), ('logical_id', logical_id)):
            if group.attrs.get(key) != text:
                group.attrs[key] = text
                self._attributes[(group.name, key)] = text

    @staticmethod
    def _text(value):
        encoded = value.encode('utf-8')
        if b'\0' in encoded:
            raise ValueError('NUL in HDF5 string')
        return struct.pack('<I', len(encoded)) + encoded

    def flush(self):
        if self._closed:
            return
        if self._failure:
            raise RuntimeError(self._failure)
        columns = [(column, rows) for column, rows in self._columns.values() if rows]
        if columns or self._attributes:
            block = bytearray(b'B' + struct.pack('<I', len(columns)))
            for column, rows in columns:
                block.extend(self._text(column.name))
                block.extend(struct.pack('<I', len(rows)))
                vlen = h5py.check_dtype(vlen=column.dtype)
                if vlen in (str, bytes):
                    for row in rows:
                        block.extend(self._text(row))
                elif vlen is not None:
                    for row in rows:
                        array = np.asarray(row, dtype=vlen)
                        block.extend(struct.pack('<I', array.size))
                        block.extend(array.tobytes())
                else:
                    array = np.asarray(rows, dtype=column.dtype)
                    if array.shape != (len(rows),) + column.shape[1:]:
                        raise ValueError('native column shape mismatch: ' + column.name)
                    block.extend(array.tobytes())
            block.extend(struct.pack('<I', len(self._attributes)))
            for (path, key), value in self._attributes.items():
                block.extend(self._text(path) + self._text(key) + self._text(value))
            self._exchange(block)
            for _, rows in columns:
                rows.clear()
            self._attributes.clear()
            self._buffer_full = False
            self._buffer_bytes = 0
        # Batch append and durable HDF5 flush have independent cadence.
        if time.monotonic() - self._last_flush >= self._flush_interval_s:
            self._exchange(b'F')
            self._last_flush = time.monotonic()

    def _exchange(self, payload, *, expected=b'OK\n'):
        if len(payload) > 64 * 1024 * 1024:
            raise ValueError('native recording block exceeds 64 MiB')
        request = struct.pack('<I', len(payload)) + payload if payload else b''
        started = time.monotonic()
        deadline = started + 20
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(self._process.stdin, selectors.EVENT_WRITE)
                offset = 0
                while offset < len(request):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or not selector.select(remaining):
                        raise TimeoutError('native recorder send timeout')
                    try:
                        count = os.write(self._process.stdin.fileno(), memoryview(request)[offset:])
                    except BlockingIOError:
                        continue
                    if not count:
                        raise RuntimeError('native recorder pipe closed')
                    offset += count
                selector.unregister(self._process.stdin)
                selector.register(self._process.stdout, selectors.EVENT_READ)
                response = bytearray()
                while b'\n' not in response:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or not selector.select(remaining):
                        raise TimeoutError('native recorder response timeout')
                    chunk = os.read(self._process.stdout.fileno(), 256)
                    if not chunk:
                        raise RuntimeError('native recorder exited')
                    response.extend(chunk)
                    if len(response) > 256:
                        raise RuntimeError('oversized native recorder response')
                if response != expected:
                    raise RuntimeError('native recorder rejected block')
        except Exception as exc:
            self._failure = str(exc)
            raise
        self.blocks_written += bool(payload and payload[0] == ord('B'))
        self.bytes_written += len(payload)
        self.max_block_latency_s = max(self.max_block_latency_s, time.monotonic() - started)

    def interrupt(self):
        self._failure = 'native recording interrupted; incomplete capture'
        if self._process and self._process.poll() is None:
            self._process.kill()

    def _shutdown(self):
        self._closed = True
        if self._process is not None:
            if self._process.poll() is None and not self._native_finalized:
                self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2)
            self._process.stdin.close()
            self._process.stdout.close()

    def close(self):
        if self._closed:
            return
        try:
            self.flush()
            self._exchange(b'C')
            self._native_finalized = True
            self._complete = True
        finally:
            self._shutdown()

    def abort(self):
        if self._closed:
            return
        try:
            if not self._failure:
                self.flush()
                self._exchange(b'A')
                self._native_finalized = True
        finally:
            self._shutdown()
