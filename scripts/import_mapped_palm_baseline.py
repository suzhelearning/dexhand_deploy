#!/usr/bin/env python3
"""Explicit, mechanical import of the approved Git baseline (not a runtime dependency).

Namespace and model meshdir are the only source transformations. Run against a
read-only source checkout; refuse to overwrite an existing imported file.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
COMMIT = '2bcfe09e2c78a48c7ba63943deef83d04a139ff8'
PORT = ROOT / 'src/tianji_teleop/src/ik/mapped_palm'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True, type=Path)
    args = parser.parse_args()
    def git(*values):
        return subprocess.check_output(['git', '-C', str(args.source), *values])
    files = git('ls-tree', '-r', '--name-only', COMMIT, 'src', 'include').decode().splitlines()
    files += ['config/qp_ik_pico_ee_bandwidth_velocity_qp.yaml',
              'models/marvin_m6_wuji2.xml', 'models/marvin_m6_s_ccs_696_v4_local.urdf']
    records = []
    pending = []
    for name in files:
        original = git('show', f'{COMMIT}:{name}')
        data = original.replace(b'tianji_qp_ik', b'tianji_mapped_palm')
        relative = name.replace('tianji_qp_ik', 'tianji_mapped_palm')
        destination = PORT / relative
        if name.startswith('config/'):
            destination = PORT / 'config/bandwidth.yaml'
        elif name.startswith('models/'):
            destination = ROOT / 'src/tianji_teleop/assets/mapped_palm' / Path(name).name
            data = data.replace(b'meshdir="tianji_wuji2"', b'meshdir="../tianji_wuji2"')
        if destination.exists():
            raise SystemExit(f'refusing to overwrite {destination}')
        records.append(dict(source=name, destination=str(destination.relative_to(ROOT)),
            original_sha256=hashlib.sha256(original).hexdigest(), sha256=hashlib.sha256(data).hexdigest()))
        pending.append((destination, data))
    # Bulk mechanical import; future algorithm edits belong outside imported src/include.
    for destination, data in pending:
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
    (PORT / 'source_manifest.json').write_text(json.dumps(dict(commit=COMMIT,
        algorithm='pico_ee_mapped_corrected_palm_velocity_qp',
        transformations=['namespace tianji_qp_ik -> tianji_mapped_palm', 'relative XML meshdir'],
        files=records), indent=2) + '\n')


if __name__ == '__main__':
    main()
