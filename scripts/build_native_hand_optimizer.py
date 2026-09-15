#!/usr/bin/env python3
"""Build the opt-in optimizer against the pinned offline Hand2 dependencies."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    prefix = ROOT/'tools/wuji_hand_native/.pixi/envs/default'
    cmake = ROOT/'.pixi/envs/ik-build/bin/cmake'
    if not (prefix/'include/nlopt.h').is_file() or not cmake.is_file():
        raise RuntimeError('install the project ik-build and pinned tools/wuji_hand_native environments first')
    destination = ROOT/'build/hand-native'
    destination.mkdir(parents=True, exist_ok=True)
    # Keep the old library usable if configure, compilation or linking fails.
    with tempfile.TemporaryDirectory(prefix='optimizer-compile-', dir=destination) as directory:
        subprocess.run([str(cmake), '-S', str(ROOT/'native/hand/optimizer'), '-B', directory,
            '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_PREFIX_PATH='+str(prefix),
            '-DCMAKE_BUILD_RPATH='+str(prefix/'lib'),
            '-DCMAKE_CXX_COMPILER='+os.environ.get('CXX', 'c++')], check=True)
        subprocess.run([str(cmake), '--build', directory, '--parallel', '1'], check=True)
        artifacts = ('libtianji_hand_optimizer.so', 'tianji_hand_native_worker',
                     'tianji_hand_native_scheduler')
        for name in artifacts:
            candidate = Path(directory) / name
            if not candidate.is_file():
                raise RuntimeError(f'native Hand2 build did not produce {name}')
            candidate.replace(destination / name)
    for name in artifacts:
        print(f'Built {destination/name}')


if __name__ == '__main__': main()
