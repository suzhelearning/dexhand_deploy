#!/usr/bin/env python3
"""Build against the active pixi environment's CPython and MuJoCo ABI."""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import sysconfig
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    prefix = Path(sys.prefix)
    if not (prefix / 'include/mujoco/mujoco.h').is_file():
        raise RuntimeError('MuJoCo development headers missing in active environment')
    destination = ROOT / 'build/control-native'
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / ('_tianji_mujoco' + sysconfig.get_config_var('EXT_SUFFIX'))
    with tempfile.TemporaryDirectory(prefix='mujoco-compile-', dir=destination) as directory:
        candidate = Path(directory) / target.name
        subprocess.run(shlex.split(os.environ.get('CXX', 'c++')) + [
            '-std=c++17', '-O3', '-DNDEBUG', '-shared', '-fPIC', '-Wall', '-Wextra', '-Werror',
            '-I' + sysconfig.get_paths()['include'], '-I' + str(prefix / 'include'),
            str(ROOT / 'native/control/python_mujoco.cpp'), '-L' + str(prefix / 'lib'),
            '-Wl,-rpath,' + str(prefix / 'lib'), '-lmujoco', '-o', str(candidate)], check=True)
        candidate.replace(target)
    print(f'Built {target}')


if __name__ == '__main__':
    main()
