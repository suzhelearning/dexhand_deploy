"""Native numeric operations only; coordinator remains the authority owner."""
from functools import lru_cache
import importlib.util
from pathlib import Path
import sysconfig


def extension_path():
    return (Path(__file__).resolve().parents[4] / 'build/control-native' /
            ('_tianji_command_math' + sysconfig.get_config_var('EXT_SUFFIX')))


@lru_cache(maxsize=1)
def load_native():
    path = extension_path()
    if not path.is_file():
        raise RuntimeError('native command math missing; run: pixi run build-native-control')
    spec = importlib.util.spec_from_file_location('_tianji_command_math', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
