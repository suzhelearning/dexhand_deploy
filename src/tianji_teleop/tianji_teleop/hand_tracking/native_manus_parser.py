"""Single-owner post-driver C++ rawviz parser; Python reference is unchanged.

The owning receiver joins its input thread before close. No SDK/driver I/O.
"""
import ctypes as C
from pathlib import Path

import numpy as np
from .reference_manus.manus_hand_input import HandInputFrame


class NativeManusProcessor:
    def __init__(self, publish, *, sides=('right','left'), right_glove=None, left_glove=None, library=None):
        if not sides or len(set(sides)) != len(sides) or set(sides)-{'right','left'}:
            raise ValueError('explicit Manus sides required')
        if not callable(publish): raise ValueError('Manus callback required')
        self._sides=tuple(side for side in ('right','left') if side in sides)
        path=Path(library or Path(__file__).resolve().parents[4]/'build/hand-native/libtianji_hand_manus.so')
        if not path.is_file(): raise RuntimeError('native Manus parser missing; run pixi run build-native-hand')
        self._lib=C.CDLL(str(path.resolve()))
        self._lib.tianji_manus_create.argtypes=[C.c_uint,C.c_char_p,C.c_char_p]
        self._lib.tianji_manus_create.restype=C.c_void_p
        self._lib.tianji_manus_destroy.argtypes=[C.c_void_p]
        self._lib.tianji_manus_destroy.restype=None
        self._lib.tianji_manus_line.argtypes=[C.c_void_p,C.c_char_p,C.POINTER(C.c_float),
            C.POINTER(C.c_int64),C.POINTER(C.c_int64),C.c_char_p,C.c_size_t]
        self._lib.tianji_manus_line.restype=C.c_int
        flags=sum(1<<i for i,s in enumerate(('right','left')) if s in sides)
        self._handle=self._lib.tianji_manus_create(flags,(right_glove or '').encode(),(left_glove or '').encode())
        if not self._handle: raise ValueError('invalid native Manus parser configuration')
        self._publish=publish
        self._points=(C.c_float*126)();self._sequences=(C.c_int64*2)();self._times=(C.c_int64*2)()
        self._error=C.create_string_buffer(1024)

    def process_line(self, line):
        if not self._handle: raise RuntimeError('native Manus parser closed')
        if '\0' in line: raise ValueError('NUL in rawviz input')
        self._error.value=b''
        count=self._lib.tianji_manus_line(self._handle,line.encode('utf-8'),self._points,
            self._sequences,self._times,self._error,len(self._error))
        if count<0: raise RuntimeError(self._error.value.decode('utf-8',errors='replace') or 'native Manus parse failed')
        if count:
            self._publish(HandInputFrame(np.ctypeslib.as_array(self._points)[:count].copy(),
                {s:int(self._sequences[i]) for i,s in enumerate(('right','left')) if s in self._sides},
                {s:int(self._times[i]) for i,s in enumerate(('right','left')) if s in self._sides}))

    def close(self):
        if self._handle:
            self._lib.tianji_manus_destroy(self._handle);self._handle=None
