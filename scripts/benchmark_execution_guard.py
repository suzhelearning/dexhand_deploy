#!/usr/bin/env python3
"""Offline receipt supervision microbenchmark; no devices or publishers."""
import argparse
import json
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))
from tianji_teleop.producers.spark.execution import execution_guard_type


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batches', type=int, default=20)
    parser.add_argument('--cycles', type=int, default=2000)
    args = parser.parse_args()
    if not 1 <= args.batches <= 1000 or not 1 <= args.cycles <= 100000:
        parser.error('batches must be 1..1000; cycles must be 1..100000')
    config = dict(run_id='bench', execution_epoch=1, coordinator_instance_id='coord',
                  router_zid='router', maximum_receipt_age_ns=10_000_000, max_in_flight=1)
    q = dict(left=[.1]*7, right=[-.1]*7)
    row = dict(schema_version=1, kind='arm_bilateral_receipt', run_id='bench', execution_epoch=1,
        tick_id=1, timestamp_ns=1, publisher_instance_id='coord', router_zid='router',
        stage='coordinator_command', accepted=True, reason='accepted', command_position_rad=q)
    types = {name: execution_guard_type(name) for name in ('python', 'cpp')}
    timing = {name: [] for name in types}
    for batch in range(args.batches):
        for name in (('python','cpp') if batch % 2 else ('cpp','python')):
            guard = types[name](**config)
            start = time.perf_counter_ns()
            for tick in range(1,args.cycles+1):
                now = tick*5_000_000
                row['tick_id'], row['timestamp_ns'] = tick, now
                if not guard.check(now):
                    raise RuntimeError(guard.reason)
                guard.register(tick,now,q)
                if not guard.observe(row,now):
                    raise RuntimeError(guard.reason)
            timing[name].append((time.perf_counter_ns()-start)/args.cycles/1000.)
            if guard.paused or guard.in_flight:
                raise RuntimeError('receipt state differs from expected empty/healthy state')
    report = {}
    for name, samples in timing.items():
        samples = sorted(samples)
        report[name] = dict(mean_us=statistics.mean(samples), median_us=statistics.median(samples),
                            max_batch_mean_us=max(samples))
    print(json.dumps(dict(scope='check_register_observe_only', batches=args.batches,
        cycles_per_batch=args.cycles, real_time_qualified=False, timing=report), indent=2))


if __name__ == '__main__':
    main()
