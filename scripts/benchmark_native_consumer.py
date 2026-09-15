#!/usr/bin/env python3
"""Same-recording Python snapshot vs native-consumer summary microbenchmark.

No devices, network, rendering or IK execution. Input is already decoded;
this measures only the Python gateway consumer, NOT end-to-end latency.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))
sys.path.insert(0, str(ROOT))

from scripts.benchmark_native_publication import load_cycles, summary
from tianji_teleop.producers.spark.native_live_runner import _NativeFrameConsumer


def compare(rows, repeats):
    timings = {'snapshot': [], 'summary': []}
    for repeat in range(repeats + 1):
        consumers = {}
        for mode in timings:
            manifest = rows[0]['manifest'] | {
                'publication_backend': 'cpp',
                'viewer_backend': 'cpp' if mode == 'summary' else 'python'}
            consumers[mode] = _NativeFrameConsumer(None, manifest=manifest, prefix='spark',
                output=None, capture=None, overlay=None, mapped_overlay=None,
                receiver_instance_id=manifest['source_authority']['instance'])
        for index, row in enumerate(rows):
            frame = SimpleNamespace(kind='cycle', payload=row['cycle'])
            order = ('summary', 'snapshot') if (index + repeat) % 2 else ('snapshot', 'summary')
            for mode in order:
                start = time.perf_counter_ns()
                consumers[mode]._consume(frame)
                elapsed = time.perf_counter_ns() - start
                if repeat:
                    timings[mode].append(elapsed)
            a, b = consumers.values()
            if a.control_status != b.control_status or a.counters != b.counters:
                raise ValueError(f'consumer status/counter mismatch at row {index}')
    return {mode: summary(values) for mode, values in timings.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--offset', type=int, default=0)
    parser.add_argument('--cycles', type=int, default=1000)
    parser.add_argument('--repeats', type=int, default=3)
    args = parser.parse_args()
    if args.offset < 0 or not 1 <= args.cycles <= 10000 or not 1 <= args.repeats <= 20:
        parser.error('offset>=0, cycles=1..10000 and repeats=1..20 required')
    digest = hashlib.sha256()
    with args.recording.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    rows, total = load_cycles(args.recording, args.offset, args.cycles)
    measured = compare(rows, args.repeats)
    print(json.dumps(dict(scope='python_consumer_only_predecoded_cycles',
        recording=str(args.recording), sha256=digest.hexdigest(), offset=args.offset,
        cycles=len(rows), total_cycles=total, repeats=args.repeats,
        status_counter_mismatches=0, timings=measured,
        hardware_acceptance=False, end_to_end_measured=False), indent=2))


if __name__ == '__main__':
    main()
