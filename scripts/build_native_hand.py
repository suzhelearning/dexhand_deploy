#!/usr/bin/env python3
"""Build optional ABI-independent hand math; no hardware SDK dependencies."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    destination = ROOT / 'build/hand-native'
    destination.mkdir(parents=True, exist_ok=True)
    eigen_candidates = [ROOT/'.pixi/envs/ik-build/include/eigen3',
        ROOT/'tools/wuji_hand_native/.pixi/envs/default/include/eigen3',
        ROOT/'.pixi/envs/default/include/eigen3']
    eigen = next((p for p in eigen_candidates if (p/'Eigen/SVD').is_file()), None)
    if eigen is None:
        raise RuntimeError('Eigen headers missing; install the project ik-build pixi environment')
    with tempfile.TemporaryDirectory(prefix='compile-', dir=destination) as directory:
        candidates = []
        for name, source in (('filter', 'lowpass'), ('geometry', 'geometry'), ('manus','manus_input')):
            target = destination / f'libtianji_hand_{name}.so'
            candidate = Path(directory) / target.name
            subprocess.run(shlex.split(os.environ.get('CXX', 'c++')) + [
                '-std=c++17', '-O3', '-shared', '-fPIC', '-ffp-contract=off',
                '-I'+str(eigen), '-Wall', '-Wextra', '-Werror',
                str(ROOT / f'native/hand/{source}.cpp'), '-o', str(candidate)], check=True)
            candidates.append((candidate, target))
        for candidate, target in candidates:
            candidate.replace(target)
            print(f'Built {target}')


if __name__ == '__main__': main()
