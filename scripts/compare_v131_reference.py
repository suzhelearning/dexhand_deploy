#!/usr/bin/env python3
"""Read-only reference-checkout comparison against its actual controller library.

Run after build_ik_sim.sh. Compiles a separate oracle linked ONLY against the
reference project's library; artifacts go into a new temporary directory.
No reference sources/build products or running sessions are changed.
"""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-root', type=Path, required=True)
    args = parser.parse_args()
    ref = args.reference_root.resolve()
    root = Path(__file__).resolve().parents[1]
    build, env = ref / 'build', ref / '.pixi/envs/default'
    library = build / 'libtianji_qp_ik.a'
    if not library.is_file():
        parser.error('build the original tianji_qp_ik library first')
    inputs = list((ref / 'src').rglob('*.cpp')) + list((ref / 'include').rglob('*.hpp'))
    if any(path.stat().st_mtime > library.stat().st_mtime for path in inputs):
        parser.error('reference library is older than its sources; rebuild it before comparing')
    text = (build / 'build.ninja').read_text()
    block = re.search(r'^build tianji_qp_ik_viewer:.*?(?=\n\n)', text, re.M | re.S)
    libraries = shlex.split(re.search(r'^  LINK_LIBRARIES = (.*)$', block[0], re.M)[1])
    output = Path(tempfile.mkdtemp(prefix='v131-reference-compare-'))
    print(f'Artifacts: {output}', flush=True)
    compiler = env / 'bin/x86_64-conda-linux-gnu-c++'
    command = [str(compiler), '-O2', '-DNDEBUG', '-std=c++17']
    for include in (ref / 'include', env / 'include', env / 'include/eigen3', build / '_deps/ruckig-src/include'):
        command.append(f'-I{include}')
    oracle = output / 'original-controller'
    command += [str(root / 'tests/v131_reference_trace.cpp'), '-o', str(oracle)] + libraries
    command += [f'-Wl,-rpath,{env}/lib', f'-Wl,-rpath-link,{env}/lib']
    subprocess.run(command, cwd=build, check=True)
    report = {
        'reference_commit': subprocess.check_output(['git', '-C', str(ref), 'rev-parse', 'HEAD'], text=True).strip(),
        'reference_library': str(library), 'joint_tolerance_rad': 1e-8,
        'cases': {},
    }
    original_args = [str(oracle), str(ref / 'config/qp_ik_pico_ee_v131_velocity_qp_mujoco.yaml'), str(ref / 'models/marvin_m6_qp_pico_fast.xml')]
    port_args = [str(root / 'build/ik-sim/v131_model_trace'), str(root / 'src/tianji_teleop/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf'), str(root / 'src/tianji_teleop/assets/v131/marvin_m6_qp_pico_fast_kinematics.xml')]
    for case, extra in [('moving', []), ('combined_otg_recovery', ['combined'])]:
        traces = []
        for name, command, cwd in [('original', original_args, ref), ('port', port_args, root)]:
            path = output / f'{case}-{name}.txt'
            with path.open('w') as stream:
                subprocess.run(command + extra, cwd=cwd, stdout=stream, check=True)
            traces.append(np.loadtxt(path))
        a, b = traces
        assert a.shape == b.shape == (600, 17), (a.shape, b.shape)
        accepted_equal = bool(np.array_equal(a[:, :3], b[:, :3]))
        error = float(np.max(np.abs(a[:, 3:] - b[:, 3:])))
        report['cases'][case] = {'frames': 600, 'acceptance_equal': accepted_equal,
            'maximum_joint_error_rad': error, 'original_rejected_ticks': int(np.any(a[:, 1:3] == 0, axis=1).sum()),
            'passed': accepted_equal and error <= report['joint_tolerance_rad']}
    report['passed'] = all(case['passed'] for case in report['cases'].values())
    (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
