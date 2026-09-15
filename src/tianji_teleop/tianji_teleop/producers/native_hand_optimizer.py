"""Opt-in native AdaptiveOptimizerAnalytical; the pinned Python source is untouched.

Construct in the isolated Hand2 worker. Python resolves configuration/model names
once; C++ owns FK, Jacobians, targets, SLSQP callbacks and warm-start state.
"""
import ctypes
import hashlib
from pathlib import Path
import sys
import threading
import weakref
import numpy as np


def library_path():
    return Path(__file__).resolve().parents[4]/'build/hand-native/libtianji_hand_optimizer.so'


def recording_metadata(environment):
    backend = environment.get('TIANJI_HAND_OPTIMIZER_BACKEND', 'python')
    if backend not in ('python','cpp'): raise ValueError('TIANJI_HAND_OPTIMIZER_BACKEND must be python or cpp')
    if backend == 'python': return None
    path = library_path()
    if not path.is_file(): raise RuntimeError('native hand optimizer missing; run pixi run build-native-hand-optimizer')
    # Do not load the pinned Pinocchio/NLopt ABI into the session's Python process.
    # Loadability/ABI validation is owned by the isolated worker before READY.
    root = Path(__file__).resolve().parents[4]
    files = dict(adapter=Path(__file__), source=root/'native/hand/optimizer.cpp',
        dependency_lock=root/'tools/wuji_hand_native/pixi.lock')
    return dict(backend='cpp', abi=1, library_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        **{key+'_sha256':hashlib.sha256(value.read_bytes()).hexdigest() for key,value in files.items()})


class NativeHandOptimizer:
    """Single-thread-owned solver, not a publisher or a hardware driver."""
    def __init__(self, original, *, library=None):
        self._owner = threading.get_ident()
        if type(original).__name__ != 'AdaptiveOptimizerAnalytical':
            raise ValueError('native optimizer only supports AdaptiveOptimizerAnalytical')
        if original.num_joints != 20 or original.last_qpos is not None:
            raise ValueError('native optimizer requires an inactive 20-DOF hand')
        config = original.config
        path = config.get('optimizer', {}).get('urdf_path')
        if not path:
            raise ValueError('native Hand2 optimizer requires explicit optimizer.urdf_path')
        urdf = Path(path)
        if not urdf.is_absolute(): urdf = Path(config['__yaml_dir'])/urdf
        urdf = urdf.resolve(strict=True)
        role_names = [original.origin_link_name]+original.task_link_names+original.link3_names+original.link4_names
        names = [original.robot.model.frames[original.robot.get_link_index(x)].name for x in role_names]
        names += original.robot.dof_joint_names
        params = [original.huber_delta, original.huber_delta_dir, original.norm_delta,
            original.w_pos, original.w_dir, original.scaling, original.w_full_hand,
            float(original.thumb_skip_pip), original.w_hyper, original.soft_min,
            original.w_couple, original.couple_ratio]
        params += list(original.segment_scaling.ravel())+list(original.d1)+list(original.d2)
        params = np.ascontiguousarray(params, dtype=np.float64)
        if params.shape != (35,) or not np.isfinite(params).all(): raise ValueError('invalid optimizer parameters')
        path = Path(library) if library is not None else library_path()
        if not path.is_file(): raise RuntimeError('native hand optimizer missing; run pixi run build-native-hand-optimizer')
        lib = self._library = ctypes.CDLL(str(path.resolve()))
        self._pointer = ctypes.POINTER(ctypes.c_double)
        lib.tianji_hand_optimizer_abi.argtypes = []
        lib.tianji_hand_optimizer_abi.restype = ctypes.c_int
        if lib.tianji_hand_optimizer_abi() != 1: raise RuntimeError('incompatible hand optimizer ABI')
        lib.tianji_hand_optimizer_error.argtypes = []
        lib.tianji_hand_optimizer_error.restype = ctypes.c_char_p
        lib.tianji_hand_optimizer_create.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p),
            ctypes.c_size_t, self._pointer, ctypes.c_size_t]
        lib.tianji_hand_optimizer_create.restype = ctypes.c_void_p
        lib.tianji_hand_optimizer_destroy.argtypes = [ctypes.c_void_p]
        lib.tianji_hand_optimizer_destroy.restype = None
        lib.tianji_hand_optimizer_state.argtypes = [ctypes.c_void_p, self._pointer, self._pointer, ctypes.c_int]
        lib.tianji_hand_optimizer_state.restype = ctypes.c_int
        lib.tianji_hand_optimizer_solve.argtypes = [ctypes.c_void_p, self._pointer, ctypes.c_size_t,
                                                    self._pointer, self._pointer]
        lib.tianji_hand_optimizer_solve.restype = ctypes.c_int
        lib.tianji_hand_optimizer_evaluate.argtypes = [ctypes.c_void_p, self._pointer, self._pointer,
                                                       ctypes.c_size_t, self._pointer, self._pointer]
        lib.tianji_hand_optimizer_evaluate.restype = ctypes.c_int
        name_array = (ctypes.c_char_p*len(names))(*(x.encode() for x in names))
        self._handle = lib.tianji_hand_optimizer_create(str(urdf).encode(), name_array, len(names),
                                                       self._ptr(params), params.size)
        if not self._handle: raise RuntimeError(self._error())
        self._cleanup = weakref.finalize(self, lib.tianji_hand_optimizer_destroy, self._handle)
        # Cold metadata is still available for original bridge limit/order inspection.
        self.robot, self.num_joints, self.config = original.robot, original.num_joints, config

    def _error(self):
        return self._library.tianji_hand_optimizer_error().decode('utf-8', errors='replace')

    def _ptr(self, value):
        return None if value is None else value.ctypes.data_as(self._pointer)

    def _check(self):
        if not self._cleanup.alive: raise RuntimeError('native hand optimizer is closed')

    @staticmethod
    def _array(value, shape):
        value = np.ascontiguousarray(value, dtype=np.float64)
        if value.shape != shape or not np.isfinite(value).all():
            raise ValueError(f'expected finite optimizer array {shape}')
        return value

    @property
    def last_qpos(self):
        self._check()
        result = np.empty(20, dtype=np.float64)
        status = self._library.tianji_hand_optimizer_state(self._handle, None, self._ptr(result), 0)
        if status < 0: raise RuntimeError(self._error())
        return None if status == 1 else result

    @last_qpos.setter
    def last_qpos(self, value):
        self._check()
        value = None if value is None else self._array(value, (20,))
        result = np.empty(20, dtype=np.float64)
        if self._library.tianji_hand_optimizer_state(self._handle, self._ptr(value), self._ptr(result), 1) < 0:
            raise RuntimeError(self._error())

    def solve(self, mediapipe_keypoints, last_qpos=None):
        self._check()
        points = self._array(mediapipe_keypoints, (21,3))
        last = None if last_qpos is None else self._array(last_qpos, (20,))
        result = np.empty(20, dtype=np.float64)
        status = self._library.tianji_hand_optimizer_solve(self._handle, self._ptr(points), 63,
                                                          self._ptr(last), self._ptr(result))
        if status < 0: raise RuntimeError(self._error())
        if status == 1:
            print('native Hand2 NLopt failure; retaining clipped initial solution', file=sys.stderr)
        return result.astype(np.float32)

    def loss_and_gradient(self, qpos, mediapipe_keypoints, last_qpos=None):
        self._check()
        q = self._array(qpos, (20,)); points = self._array(mediapipe_keypoints, (21,3))
        last = None if last_qpos is None else self._array(last_qpos, (20,))
        result = np.empty(21, dtype=np.float64)
        if self._library.tianji_hand_optimizer_evaluate(self._handle, self._ptr(q), self._ptr(points),
                                                        63, self._ptr(last), self._ptr(result)) < 0:
            raise RuntimeError(self._error())
        return float(result[0]), result[1:]

    def compute_cost(self, qpos, mediapipe_keypoints):
        return self.loss_and_gradient(qpos, mediapipe_keypoints)[0]

    def close(self):
        if threading.get_ident() != self._owner:
            raise RuntimeError('optimizer owner thread mismatch')
        self._cleanup()


def install_native_optimizers(bridge, *, library=None):
    replacements = {}
    try:
        for side, retargeter in bridge._retargeters.items():
            replacements[side] = NativeHandOptimizer(retargeter.optimizer, library=library)
    except BaseException:
        for native in replacements.values(): native.close()
        raise
    for side, native in replacements.items(): bridge._retargeters[side].optimizer = native
