#!/usr/bin/env python3
"""Offline A/B of native JSON/binary responses; no router, devices or commands.

Use --trace for recorded live input. Deterministic mode verifies value parity;
normal budgets measure timing (wall-clock QP budget decisions can differ).
"""
import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))
from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND, MAPPED_PALM_BACKEND
from tianji_teleop.hand_tracking.spark_replay import ReplayTick, iter_reference_ticks
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.hand_tracking.tjvr_trace import iter_tjvr_records
from tianji_teleop.producers.spark.backend_assets import bilateral_assets


def summary(values):
    values = sorted(v*1000 for v in values)
    return dict(mean_ms=statistics.mean(values), p95_ms=values[int((len(values)-1)*.95)],
                p99_ms=values[int((len(values)-1)*.99)], max_ms=values[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=(SPARK_BACKEND, MAPPED_PALM_BACKEND), default=MAPPED_PALM_BACKEND)
    parser.add_argument('--cycles', type=int, default=1000)
    parser.add_argument('--trace', type=Path)
    parser.add_argument('--deterministic-test', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.cycles <= 100000:
        parser.error('cycles must be in 1..100000')
    if args.trace:
        with args.trace.open('rb') as stream:
            records = list(iter_tjvr_records(stream))  # validate the entire container
        receiver = ReferenceTjvrReceiver('binary-benchmark', .15, .6,
            **({'target_source': 'mapped_corrected_palm'} if args.backend == MAPPED_PALM_BACKEND else {}))
        from itertools import islice
        ticks = list(islice(iter_reference_ticks(records, receiver=receiver), args.cycles))
    else:
        ticks = [ReplayTick(i+1, 1_000_000_000+i*5_000_000, None) for i in range(args.cycles)]
    selected = bilateral_assets(ROOT, args.backend)
    metrics = {fmt: {} for fmt in ('json', 'binary')}
    mismatches = 0
    with ExitStack() as stack:
        clients = {fmt: stack.enter_context(selected['client'](
            **{k: selected[k] for k in ('worker', 'config', 'model', 'urdf')},
            result_format=fmt, startup_handshake=True, deterministic_test=args.deterministic_test))
            for fmt in metrics}
        for index, tick in enumerate(ticks):
            rows = {}
            # Alternate order to reduce consistent first/second execution bias.
            for fmt in (('json', 'binary') if index % 2 else ('binary', 'json')):
                started = time.perf_counter()
                rows[fmt] = clients[fmt].step(tick)
                elapsed = time.perf_counter()-started
                for name, value in dict(total=elapsed, **clients[fmt].last_timing).items():
                    metrics[fmt].setdefault(name, []).append(value)
            mismatches += rows['json'] != rows['binary']
    print(json.dumps(dict(backend=args.backend, cycles=len(ticks), input='trace' if args.trace else 'no_input',
        deterministic_test=args.deterministic_test, differing_results=mismatches,
        real_time_qualified=False, metrics={fmt: {k: summary(v) for k,v in stages.items()}
            for fmt, stages in metrics.items()}), indent=2))
    return 1 if args.deterministic_test and mismatches else 0


if __name__ == '__main__':
    raise SystemExit(main())
