"""Optional native wrist-frame preprocessing; configuration remains cold-path."""
import ctypes
import hashlib
from pathlib import Path
import numpy as np


def library_path():
    return Path(__file__).resolve().parents[4]/'build/hand-native/libtianji_hand_geometry.so'


class NativeHandGeometry:
    def __init__(self, side, signs, rotation, wrist, thumb, *, library=None):
        if side not in ('left', 'right'): raise ValueError('invalid hand side')
        self._left = int(side == 'left')
        arrays = [np.asarray(x, dtype=np.float64) for x in (signs, rotation, wrist, thumb)]
        if [x.shape for x in arrays] != [(3,), (3,3), (3,), (3,)]:
            raise ValueError('invalid geometry configuration shape')
        self._config = np.concatenate([x.ravel() for x in arrays])
        if (not np.isfinite(self._config).all() or not np.isin(arrays[0], [-1,1]).all()
                or not np.allclose(arrays[1]@arrays[1].T, np.eye(3), atol=1e-10, rtol=0)
                or abs(np.linalg.det(arrays[1])-1) > 1e-10):
            raise ValueError('invalid geometry configuration')
        path = Path(library) if library is not None else library_path()
        if not path.is_file(): raise RuntimeError('native hand geometry missing; run pixi run build-native-hand')
        self._library = ctypes.CDLL(str(path.resolve()))
        self._library.tianji_hand_geometry_abi.argtypes = []
        self._library.tianji_hand_geometry_abi.restype = ctypes.c_int
        if self._library.tianji_hand_geometry_abi() != 1: raise RuntimeError('incompatible hand geometry ABI')
        self._pointer = ctypes.POINTER(ctypes.c_double)
        self._call = self._library.tianji_hand_geometry_prepare
        self._call.argtypes = [self._pointer, ctypes.c_size_t, self._pointer,
                              ctypes.c_size_t, ctypes.c_int, self._pointer]
        self._call.restype = ctypes.c_int

    def __call__(self, points):
        points = np.ascontiguousarray(points, dtype=np.float64)
        if points.shape != (21,3): raise ValueError('raw_keypoints must have shape (21, 3)')
        result = np.empty((21,3), dtype=np.float64)
        if self._call(points.ctypes.data_as(self._pointer), 63,
                      self._config.ctypes.data_as(self._pointer), 18, self._left,
                      result.ctypes.data_as(self._pointer)) != 0:
            raise ValueError('native hand geometry rejected invalid or degenerate frame')
        return result


def install_native_geometry(bridge, *, library=None):
    from scipy.spatial.transform import Rotation
    replacements = {}
    for side, retargeter in bridge._retargeters.items():
        if retargeter.optimizer.last_qpos is not None:
            raise RuntimeError('cannot switch active hand geometry')
        rotation = Rotation.from_euler('xyz', [retargeter.rotation_xyz.get(k, 0.)
                                             for k in ('x','y','z')], degrees=True).as_matrix()
        replacements[side] = NativeHandGeometry(side, retargeter.input_axis_sign, rotation,
            retargeter.wrist_offset_m, retargeter.thumb_offset_m, library=library)
    for side, native in replacements.items():
        bridge._retargeters[side]._prepare_keypoints = native


def recording_metadata(environment):
    backend = environment.get('TIANJI_HAND_GEOMETRY_BACKEND', 'python')
    if backend not in ('python','cpp'): raise ValueError('TIANJI_HAND_GEOMETRY_BACKEND must be python or cpp')
    if backend == 'python': return None
    path = library_path()
    NativeHandGeometry('right', [1]*3, np.eye(3), [0]*3, [0]*3, library=path)
    root = Path(__file__).resolve().parents[4]
    return dict(backend='cpp', abi=1, library_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        adapter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256((root/'native/hand/geometry.cpp').read_bytes()).hexdigest())
