"""Distinct wire identity over the existing bounded bilateral IPC transport."""
from .input_modes import MAPPED_PALM_BACKEND
from .spark_worker_client import SparkWorkerClient
import math


class MappedPalmWorkerClient(SparkWorkerClient):
    ALGORITHM = MAPPED_PALM_BACKEND
    WIRE_PREFIX = 'mapped_palm'

    def configure_height(self, offsets):
        with self._lock:
            if self._closed or self._tick != 0:
                raise ValueError('height configuration requires an open worker at tick zero')
            if (not isinstance(offsets, (list, tuple)) or len(offsets) != 2 or
                    any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1 for v in offsets)):
                raise ValueError('height requires two finite offsets within 1 m')
            values = list(map(float, offsets))
            try:
                ack = self._exchange(('TJMH1 ' + ' '.join(map(repr,values)) + '\n').encode('ascii'),
                                     'height calibration')
                if (ack != dict(kind='mapped_palm_height_ack', target_height_offsets_m=values) or
                        any(type(v) not in (int,float) for v in ack['target_height_offsets_m'])):
                    raise ValueError('height acknowledgement differs from requested offsets')
                return ack
            except BaseException:
                self._shutdown()
                raise
