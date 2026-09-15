"""Opt-in native receipt checks. Startup validation retains the Python contract."""
from functools import lru_cache
import importlib.util
from pathlib import Path
import sysconfig
from types import MappingProxyType

from .execution import ExecutionGuard


def extension_path():
    root = Path(__file__).resolve().parents[5]
    return root / 'build/control-native' / ('_tianji_execution' + sysconfig.get_config_var('EXT_SUFFIX'))


@lru_cache(maxsize=1)
def load_native():
    path = extension_path()
    if not path.is_file():
        raise RuntimeError('missing native control extension; run pixi run build-native-control')
    spec = importlib.util.spec_from_file_location('_tianji_execution', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class NativeExecutionGuard:
    __slots__ = ('_settings', '_module', '_handle')

    def __init__(self, **options):
        reference = ExecutionGuard(**options)
        settings = {key: getattr(reference, key) for key in ('run_id', 'execution_epoch',
            'coordinator_instance_id', 'router_zid', 'maximum_receipt_age_ns', 'max_in_flight')}
        self._settings = MappingProxyType(settings)
        self._module = load_native()
        self._handle = self._module.create(settings)

    def __getattr__(self, name):
        try:
            return self._settings[name]
        except KeyError:
            raise AttributeError(name) from None

    @property
    def reason(self):
        return self._module.reason(self._handle)

    @property
    def paused(self):
        return self.reason is not None

    @property
    def in_flight(self):
        return self._module.in_flight(self._handle)

    def check(self, now_ns):
        return self._module.check(self._handle, now_ns)

    def register(self, tick_id, timestamp_ns, reference_positions):
        return self._module.register(self._handle, tick_id, timestamp_ns, reference_positions)

    def observe(self, receipt, now_ns):
        return self._module.observe(self._handle, receipt, now_ns)

    def pause(self, reason):
        return self._module.pause(self._handle, reason)
