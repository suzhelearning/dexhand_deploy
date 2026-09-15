#!/usr/bin/env python3
"""Offline same-HDF5 publication encoder A/B; no Zenoh, IK execution or devices.

Measures encode + per-topic JSON serialization only, NOT end-to-end scheduling,
network delivery, Python elimination, HDF5 throughput or hardware acceptance.
"""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))
sys.path.insert(0, str(ROOT))


def load_cycles(path, offset, count):
    import h5py
    with h5py.File(path, 'r') as file:
        if not file.attrs.get('complete', False):
            raise ValueError('benchmark requires a completely closed recording')
        audit = file['meta/dual_audit']
        indices = [i for i, kind in enumerate(audit['kind'].asstr()[:]) if kind == 'native_cycle']
        states = file['meta/session_events']
        feedback = file['joint/state/arm']
        if len(indices) != len(states['state']) or len(indices) != len(feedback['position_rad']):
            raise ValueError('cycle/state/feedback row counts differ; refusing implicit alignment')
        selected = indices[offset:offset + count]
        if not selected:
            raise ValueError('empty selected recording interval')
        rows = []
        for ordinal, index in enumerate(selected, offset):
            if not (audit['time_ns'][index] == states['time_ns'][ordinal] == feedback['time_ns'][ordinal]):
                raise ValueError('cycle/state/feedback timestamps differ')
            item = json.loads(audit['payload_json'].asstr()[index])
            command = item['bilateral_command']; left = command['left']; source = item['source_status']
            q = feedback['position_rad'][ordinal].tolist()
            row = dict(timestamp_ns=left['timestamp_ns'], ticks=left['sequence'],
                       state=states['state'].asstr()[ordinal], reason=states['reason'].asstr()[ordinal],
                       state_epoch=item['execution_epoch'],
                       command={side: command[side]['position_rad'] for side in ('left', 'right')},
                       feedback=dict(left=q[:7], right=q[7:]),
                       source=dict(source['diagnostics'], sequence=source['sequence'],
                                   revision=source['diagnostics'].get('source_revision', 0)),
                       result=item['native'], ik_adopted=item['receipt_accepted'],
                       capture_failed=not source['healthy'])
            # Producer status isn't stored in each audit row. Use the same
            # explicit replay identity/model label on both sides, not a claim
            # to reconstruct missing original session metadata.
            config = dict(run_id=command['run_id'], algorithm='recorded-replay',
                          model=str(file.attrs['robot_model']), home=[[0.] * 7] * 2,
                          source_authority=dict(logical='tjvr', instance=source['publisher_instance_id'],
                                                router=source['router_zid']),
                          coordinator_authority=dict(logical='arm', instance=left['publisher_instance_id'],
                                                     router=left['router_zid']),
                          producer_authority=dict(logical='replay-producer', instance='replay-producer',
                                                  router=left['router_zid']),
                          executor_authority=dict(logical='mujoco',
                              instance=feedback['publisher_instance_id'].asstr()[ordinal], router=left['router_zid']))
            rows.append(dict(cycle=row, manifest=config))
        return rows, len(indices)


def summary(samples):
    values = sorted(value / 1000 for value in samples)
    return dict(count=len(values), mean_us=statistics.mean(values),
                **{name: values[int((len(values) - 1) * quantile)]
                   for name, quantile in (('p50_us', .5), ('p95_us', .95), ('p99_us', .99))},
                max_us=values[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', required=True, type=Path)
    parser.add_argument('--offset', type=int, default=0)
    parser.add_argument('--cycles', type=int, default=1000)
    args = parser.parse_args()
    if args.offset < 0 or not 1 <= args.cycles <= 10000:
        parser.error('offset >= 0 and cycles in 1..10000 required')
    from tests.test_native_session_publication import reference
    with args.recording.open('rb') as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b''): digest.update(block)
    rows, total = load_cycles(args.recording, args.offset, args.cycles)
    with tempfile.TemporaryDirectory(prefix='publication-ab-') as directory:
        binary = Path(directory) / 'encode'
        subprocess.run(['c++', '-std=c++17', '-O2', '-pthread', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(Path(sys.prefix) / 'include'),
                        str(ROOT / 'tests/cpp/session_publication_fixture.cpp'), '-o', str(binary)], check=True)
        result = subprocess.run([str(binary), '--timing'], text=True, capture_output=True, timeout=120, check=True,
                                input=''.join(json.dumps(row) + '\n' for row in rows))
    native = [json.loads(line) for line in result.stdout.splitlines()]
    native_ns = json.loads(result.stderr)['encode_serialize_ns']
    # Warm imports/constructors before timing, matching exclusion of C++ process
    # startup, manifest construction, input parsing and stdout from native times.
    reference(rows[0]['cycle'], rows[0]['manifest'])
    python_ns = []; mismatches = 0
    for item, expected in zip(rows, native):
        begin = time.perf_counter_ns()
        actual = reference(item['cycle'], item['manifest'])
        for _, message in actual: json.dumps(message, separators=(',', ':'), ensure_ascii=False)
        python_ns.append(time.perf_counter_ns() - begin)
        mismatches += actual != expected
    if len(native) != len(rows): raise ValueError('native output count mismatch')
    print(json.dumps(dict(scope='publication_encode_serialize_only', recording=str(args.recording.resolve()),
        sha256=digest.hexdigest(), total_recorded_cycles=total, offset=args.offset, cycles=len(rows),
        mismatching_cycles=mismatches, python=summary(python_ns), cpp=summary(native_ns),
        mean_ratio_python_over_cpp=statistics.mean(python_ns) / statistics.mean(native_ns),
        real_time_qualified=False, limitations=[
            'No network, queue, recording, viewer or end-to-end timing measured',
            'Same recorded commands/feedback; solver not rerun',
            'Unrecorded producer identity and model metadata normalized identically',
            'Single-process-per-backend microbenchmark; repeat runs for CPU/cache variability']), indent=2))
    return 1 if mismatches else 0


if __name__ == '__main__':
    raise SystemExit(main())
