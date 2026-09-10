"""Bounded per-dataset batching for the new PICO recorder's single disk owner."""
from copy import deepcopy

from .session_h5 import SessionH5Error, SessionH5Writer


class BufferedSessionWriter(SessionH5Writer):
    def __init__(self, *args, **kwargs):
        self._rows = {}
        super().__init__(*args, **kwargs)

    def _append(self, dataset, value):
        stored_dataset, rows = self._rows.setdefault(dataset.name, (dataset, []))
        rows.append(deepcopy(value))
        if len(rows) >= 256:
            self._write_rows(stored_dataset, rows)

    @staticmethod
    def _write_rows(dataset, rows):
        if rows:
            offset = dataset.shape[0]
            dataset.resize(offset + len(rows), axis=0)
            dataset[offset:] = rows
            rows.clear()

    def append_dual_audit_batch(self, rows):
        if not isinstance(rows, (list, tuple)) or not 0 < len(rows) <= 256:
            raise SessionH5Error('dual audit batch must contain 1..256 rows')
        encoded = []
        for row in rows:
            if not isinstance(row, (list, tuple)) or len(row) != 3:
                raise SessionH5Error('dual audit row requires kind, payload and timestamp')
            kind, payload, stamp = row
            encoded.append((kind, self._encode_dual_audit(kind, payload, stamp), stamp))
        group = self._file['meta/dual_audit']
        for kind, payload, stamp in encoded:
            self._append(group['time_ns'], self._record_time(stamp))
            self._append(group['received_timestamp_ns'], stamp)
            self._append(group['kind'], kind)
            self._append(group['payload_json'], payload)
        self._maybe_flush()

    def flush(self):
        if not self._closed:
            for dataset, rows in self._rows.values():
                self._write_rows(dataset, rows)
            super().flush()

    def close(self):
        # A failed buffered write must not set the complete attribute first.
        self.flush()
        super().close()
