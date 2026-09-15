#!/usr/bin/env python3
"""Build for the active CPython ABI; no SDK, robot library or pip dependency."""
import os
from pathlib import Path
import shlex
import subprocess
import sysconfig
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    destination = ROOT / 'build/control-native'
    destination.mkdir(parents=True, exist_ok=True)
    suffix = sysconfig.get_config_var('EXT_SUFFIX')
    if not suffix:
        raise RuntimeError('active Python does not define its extension ABI suffix')
    for name, source in (('_tianji_execution', 'python_execution_guard.cpp'),
                         ('_tianji_command_math', 'python_command_math.cpp')):
        target = destination / (name + suffix)
        with tempfile.TemporaryDirectory(prefix='compile-', dir=destination) as directory:
            candidate = Path(directory) / target.name
            compiler = shlex.split(os.environ.get('CXX', 'c++'))
            subprocess.run(compiler + ['-std=c++17', '-O3', '-DNDEBUG', '-shared', '-fPIC',
                '-ffp-contract=off', '-Wall', '-Wextra', '-Werror', '-I' + sysconfig.get_paths()['include'],
                str(ROOT / 'native/control' / source), '-o', str(candidate)], check=True)
            candidate.replace(target)
        print(f'Built {target}')


if __name__ == '__main__':
    main()
