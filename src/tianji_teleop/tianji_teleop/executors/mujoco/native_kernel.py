"""Opt-in same-process C++ joint application; no command authority of its own."""
from functools import lru_cache
import importlib.util
from pathlib import Path
import sysconfig


def extension_path():
    return (Path(__file__).resolve().parents[5] / 'build/control-native' /
            ('_tianji_mujoco' + sysconfig.get_config_var('EXT_SUFFIX')))


@lru_cache(maxsize=1)
def load_native():
    path = extension_path()
    if not path.is_file():
        raise RuntimeError('native MuJoCo missing; run: pixi run build-native-mujoco')
    spec = importlib.util.spec_from_file_location('_tianji_mujoco', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class NativeMujocoKernel:
    def __init__(self, model, data, groups):
        self._native = load_native()
        self._handle = self._native.create(model, data, groups)

    def apply(self, groups):
        self._native.apply(self._handle, groups)

    def positions(self, start, count):
        return self._native.positions(self._handle, start, count)
