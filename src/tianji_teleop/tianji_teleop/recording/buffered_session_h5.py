"""Bounded column buffering for the VR/Manus disk process only.

The canonical writer still validates messages and defines every dataset. Flush
whole column prefixes at message boundaries instead of resizing every dataset
for every 200 Hz control tick. No samples are resampled or discarded.
"""
from copy import deepcopy

import h5py
import numpy as np

from .session_h5 import SessionH5Writer, SessionH5Error


class _CachedGroup:
    """Keep fixed-layout HDF5 handles open in this one disk owner.

    Reopening dozens of datasets for every control tick otherwise dominates
    even after row writes are buffered. The recording layout never changes.
    """
    def __init__(self, group):
        self._group = group
        self._children = {}

    def __getitem__(self, key):
        if key not in self._children:
            child = self._group[key]
            self._children[key] = _CachedGroup(child) if isinstance(child, h5py.Group) else child
        return self._children[key]

    def __contains__(self, key):
        return key in self._children or key in self._group

    def __getattr__(self, name):
        return getattr(self._group, name)


class BufferedSessionH5Writer(SessionH5Writer):
    def __init__(self, *args, **kwargs):
        self._columns = {}
        self._buffer_full = False
        super().__init__(*args, **kwargs)
        self._file = _CachedGroup(self._file)

    def _append(self, dataset, value):
        key = dataset.name
        if key not in self._columns:
            self._columns[key] = (dataset, [])
        rows = self._columns[key][1]
        rows.append(deepcopy(value))
        self._buffer_full |= len(rows) >= 128

    def _maybe_flush(self):
        if self._buffer_full:
            self.flush()
        else:
            super()._maybe_flush()

    def flush(self):
        if self._closed:
            return
        for dataset, rows in self._columns.values():
            if not rows:
                continue
            offset = dataset.shape[0]
            dataset.resize(offset + len(rows), axis=0)
            # Numeric variable-length rows (raw UDP bytes/Manus point arrays)
            # require a 1-D object array, even when lengths happen to match.
            vlen = h5py.check_dtype(vlen=dataset.dtype)
            if vlen is not None and vlen not in (str, bytes):
                for index, row in enumerate(rows):
                    # h5py expands equally sized vlen arrays to 2-D during
                    # slice assignment. Keep these few packet columns scalar;
                    # resize once, while all fixed-width columns stay batched.
                    dataset[offset + index] = np.asarray(row, dtype=vlen)
            else:
                values = np.asarray(rows, dtype=dataset.dtype)
                dataset[offset:offset + len(rows)] = values
            rows.clear()
        self._buffer_full = False
        super().flush()

    def append_dual_audit_batch(self, rows):
        if not isinstance(rows, (list, tuple)) or not 0 < len(rows) <= 64:
            raise SessionH5Error('audit batch must contain 1..64 rows')
        encoded = []
        for row in rows:
            if not isinstance(row, (list, tuple)) or len(row) != 3:
                raise SessionH5Error('audit row requires kind, payload and timestamp')
            kind, payload, stamp = row
            encoded.append((kind, self._encode_dual_audit(kind, payload, stamp), stamp))
        group = self._file['meta/dual_audit']
        for kind, payload, stamp in encoded:
            for name, value in (('time_ns', self._record_time(stamp)),
                                ('received_timestamp_ns', stamp),
                                ('kind', kind), ('payload_json', payload)):
                self._append(group[name], value)
        self._maybe_flush()
