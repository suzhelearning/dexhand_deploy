#!/usr/bin/env python3
"""Build the opt-in C++ session gateway against the active MuJoCo ABI.

The gateway is deliberately a standalone executable.  It does not link to a
device SDK or to Python; Python remains the cold-path launcher and the
existing live Python route remains the default.
"""

import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def main():
    prefix = Path(sys.prefix)
    eigen_candidates = [
        prefix / 'include/eigen3',
        ROOT / '.pixi/envs/ik-build/include/eigen3',
        ROOT / 'tools/mapped_palm_native/.pixi/envs/default/include/eigen3',
        ROOT / 'tools/wuji_hand_native/.pixi/envs/default/include/eigen3',
        Path('/usr/include/eigen3'),
    ]
    eigen = next((candidate for candidate in eigen_candidates
                  if (candidate / 'Eigen/Core').is_file()), None)
    mujoco_header = prefix / 'include/mujoco/mujoco.h'
    if not mujoco_header.is_file():
        raise RuntimeError('MuJoCo development headers missing in active environment')
    if eigen is None:
        raise RuntimeError('Eigen headers missing; install an existing project native build environment')

    destination = ROOT / 'build/control-native'
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / 'tianji_native_session_gateway'
    sources = [
        ROOT / 'native/control/session_gateway_main.cpp',
        ROOT / 'native/hand/manus_input.cpp',
        ROOT / 'src/tianji_teleop/src/ik/mapped_palm/src/pico_teleop_protocol.cpp',
        ROOT / 'src/tianji_teleop/src/ik/mapped_palm/src/pico_mapped_corrected_palm.cpp',
        ROOT / 'src/tianji_teleop/src/ik/mapped_palm/src/so3.cpp',
    ]
    for source in sources:
        if not source.is_file():
            raise RuntimeError(f'missing native gateway source: {source}')

    compiler = shlex.split(os.environ.get('CXX', 'c++'))
    glfw_include = next((path for path in (prefix / 'include', Path('/usr/include'))
                         if (path / 'GLFW/glfw3.h').is_file()), None)
    command = compiler + [
        '-std=c++17', '-O2', '-DNDEBUG', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
        '-ffp-contract=off', '-pthread',
        '-DZENOHCXX_ZENOHC',
        '-isystem', str(ROOT / 'vendor/zenoh-cpp/include'),
        '-isystem', str(ROOT / 'vendor/zenoh/include'),
        '-I' + str(prefix / 'include'),
        '-I' + str(ROOT / 'src/tianji_teleop/src/ik/mapped_palm/include'),
        '-I' + str(eigen),
    ] + [str(source) for source in sources] + [
        '-L' + str(prefix / 'lib'),
        '-Wl,-rpath,' + str(prefix / 'lib'),
        '-lmujoco', '-pthread',
        '-L' + str(ROOT / 'vendor/zenoh/lib'),
        '-Wl,-rpath,' + str(ROOT / 'vendor/zenoh/lib'),
        '-lzenohc',
    ]
    if glfw_include is not None:
        command += ['-DTIANJI_WITH_GLFW', '-I' + str(glfw_include), '-lglfw']

    # Keep a known-good binary in place if compilation fails halfway through.
    with tempfile.TemporaryDirectory(prefix='session-gateway-', dir=destination) as directory:
        candidate = Path(directory) / target.name
        subprocess.run(command + ['-o', str(candidate)], cwd=ROOT, check=True)
        if not candidate.is_file() or not os.access(candidate, os.X_OK):
            raise RuntimeError('native session gateway build produced no executable')
        candidate.replace(target)
    print(f'Built {target}')


if __name__ == '__main__':
    main()
