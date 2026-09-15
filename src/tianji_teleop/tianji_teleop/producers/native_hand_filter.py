"""Optional native LPFilter adapter; no device access or command authorization."""
import ctypes
import hashlib
from pathlib import Path
import weakref
import numpy as np


def library_path():
    return Path(__file__).resolve().parents[4] / 'build/hand-native/libtianji_hand_filter.so'


def recording_metadata(environment):
    backend = environment.get('TIANJI_HAND_FILTER_BACKEND', 'python')
    if backend not in ('python', 'cpp'): raise ValueError('TIANJI_HAND_FILTER_BACKEND must be python or cpp')
    if backend == 'python': return None  # Keep old metadata unchanged.
    path = library_path()
    # Fail before recording readiness when the requested native implementation is absent.
    check = NativeHandLPFilter(.2, library=path)
    check.close()
    root = Path(__file__).resolve().parents[4]
    return dict(backend='cpp', abi=1, library_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        adapter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256((root/'native/hand/lowpass.cpp').read_bytes()).hexdigest())


class NativeHandLPFilter:
    """Single-owner 20-DOF filter. Caller serializes next/reset/close."""
    def __init__(self, alpha, *, library=None):
        self.alpha = float(alpha)
        if not np.isfinite(self.alpha) or not 0 < self.alpha <= 1:
            raise ValueError('filter alpha must be finite in (0, 1]')
        path = Path(library) if library is not None else library_path()
        if not path.is_file():
            raise RuntimeError('native hand filter missing; run pixi run build-native-hand')
        self._library = ctypes.CDLL(str(path.resolve()))
        lib = self._library
        lib.tianji_hand_filter_abi.argtypes = []
        lib.tianji_hand_filter_abi.restype = ctypes.c_int
        if lib.tianji_hand_filter_abi() != 1: raise RuntimeError('incompatible hand filter ABI')
        lib.tianji_hand_filter_create.argtypes = [ctypes.c_double]
        lib.tianji_hand_filter_create.restype = ctypes.c_void_p
        lib.tianji_hand_filter_destroy.argtypes = [ctypes.c_void_p]
        lib.tianji_hand_filter_destroy.restype = None
        lib.tianji_hand_filter_reset.argtypes = [ctypes.c_void_p]
        lib.tianji_hand_filter_reset.restype = ctypes.c_int
        self._pointer_type = ctypes.POINTER(ctypes.c_double)
        lib.tianji_hand_filter_next.argtypes = [ctypes.c_void_p, self._pointer_type, ctypes.c_size_t, self._pointer_type, ctypes.c_int]
        lib.tianji_hand_filter_next.restype = ctypes.c_int
        self._handle = lib.tianji_hand_filter_create(self.alpha)
        if not self._handle: raise RuntimeError('native hand filter allocation failed')
        self._cleanup = weakref.finalize(self, lib.tianji_hand_filter_destroy, self._handle)
        self._dtype = None

    def _check_open(self):
        if not self._cleanup.alive: raise RuntimeError('native hand filter is closed')

    def next(self, value):
        self._check_open()
        value = np.asarray(value)
        single = value.dtype == np.dtype('float32')
        array = np.ascontiguousarray(value, dtype=np.float64)
        if array.shape != (20,): raise ValueError('hand filter requires shape (20,)')
        result = np.empty(20, dtype=np.float64)
        if self._library.tianji_hand_filter_next(self._handle, array.ctypes.data_as(self._pointer_type),
                20, result.ctypes.data_as(self._pointer_type), int(single)) != 0:
            raise ValueError('native hand filter rejected nonfinite input/result')
        self._dtype = np.dtype('float32') if single and (self._dtype is None or self._dtype == np.dtype('float32')) else np.dtype('float64')
        return result.astype(self._dtype, copy=False)

    def reset(self):
        self._check_open()
        if self._library.tianji_hand_filter_reset(self._handle) != 0:
            raise RuntimeError('native hand filter reset failed')
        self._dtype = None

    def close(self):
        self._cleanup()


def install_native_filters(bridge, *, library=None):
    """Called before any callback; preserve the pinned bridge's reset ownership."""
    replacements = {}
    try:
        for side, retargeter in bridge._retargeters.items():
            if retargeter.lp_filter.is_init:
                raise RuntimeError('cannot switch an active hand filter')
            replacements[side] = NativeHandLPFilter(retargeter.lp_filter.alpha, library=library)
    except BaseException:
        for native in replacements.values(): native.close()
        raise
    for side, native in replacements.items(): bridge._retargeters[side].lp_filter = native
