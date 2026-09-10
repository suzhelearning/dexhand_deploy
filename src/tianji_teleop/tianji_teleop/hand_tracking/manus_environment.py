"""Resolve the SDK path for the Manus acquisition child only."""
import os
from pathlib import Path


def manus_environment(rawviz, *, library_dir=None, base_env=None):
    env = dict(os.environ if base_env is None else base_env)
    directory = (Path(library_dir) if library_dir is not None else
                 Path(rawviz).resolve().parent / 'ManusSDK/lib').resolve()
    library = directory / 'libManusSDK_Integrated.so'
    if not library.is_file():
        if library_dir is not None:
            raise ValueError(f'Manus SDK library missing: {library}')
        return env, None
    # Do not export this into the coordinator, native IK or isolated hand worker.
    inherited = [entry for entry in env.get('LD_LIBRARY_PATH', '').split(':') if entry]
    env['LD_LIBRARY_PATH'] = ':'.join([str(directory), *inherited])
    return env, library
