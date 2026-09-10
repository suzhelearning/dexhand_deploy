#!/usr/bin/env python3
"""Offline TJVR -> native SPARK replay. No ROS, Zenoh, UDP or device outputs.

The JSONL file is complete only if its final spark_trace_complete record exists.
The original recording is never overwritten or resampled. A fixed receive-clock
schedule replaces OS scheduling ONLY for this explicitly identified test runner.
"""
import argparse
import hashlib
import json
from pathlib import Path
import selectors
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.hand_tracking.spark_replay import encode_tick, iter_reference_ticks
from tianji_teleop.hand_tracking.tjvr_trace import iter_tjvr_records


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def model_fingerprints(path):
    tree = ET.fromstring(path.read_text())
    compiler = tree.find('compiler')
    meshdir = path.parent / compiler.attrib.get('meshdir', '')
    meshes = {mesh.attrib['file']: sha256(meshdir / mesh.attrib['file'])
              for mesh in tree.findall('asset/mesh')}
    compiler.attrib['meshdir'] = ''  # permitted packaging-only path change
    return (hashlib.sha256(ET.tostring(tree)).hexdigest(),
            hashlib.sha256(json.dumps(meshes, sort_keys=True).encode()).hexdigest())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--worker', type=Path, default=ROOT / 'build/spark-native/spark_native_worker')
    parser.add_argument('--config', type=Path, default=ROOT / 'src/tianji_teleop/config/producers/spark_reference.yaml')
    parser.add_argument('--model', type=Path, default=ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_wuji2.xml')
    parser.add_argument('--urdf', type=Path, default=ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_s_ccs_696_v4_local.urdf')
    parser.add_argument('--deterministic-test', action='store_true',
                        help='relax only wall-clock solve budgets for numerical comparison, NOT real-time use')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    rate = float(config['controller']['rate_hz'])
    period_ns = round(1e9 / rate)
    if period_ns <= 0 or abs(period_ns * rate - 1e9) > 1e-3:
        parser.error('replay requires an integer-nanosecond control period')
    limits = config['pico_teleop']
    receiver = ReferenceTjvrReceiver('offline-replay', limits['max_position_jump_m'],
                                   limits['max_orientation_jump_rad'])
    manifest = dict(schema_version=1, kind='spark_trace_manifest',
        deterministic_test=args.deterministic_test, period_ns=period_ns,
        receive_origin_ns=1_000_000_000, tail_ns=250_000_000,
        scheduling='arrivals_le_tick_then_latest',
        files={name: {'path': str(getattr(args, name).resolve()), 'sha256': sha256(getattr(args, name))}
               for name in ('input', 'config', 'model', 'urdf', 'worker')})
    manifest['model_semantic_sha256'], manifest['model_meshes_sha256'] = model_fingerprints(args.model)
    command = [str(args.worker.resolve()), str(args.config.resolve()), str(args.model.resolve()), str(args.urdf.resolve())]
    if args.deterministic_test:
        command.append('--deterministic-test')
    summary = dict(kind='spark_trace_complete', ticks=0, applied_ticks=0,
                   guidance_accepted_ticks=0, control_failures=0, budget_exhausted_ticks=0)
    digest = hashlib.sha256()
    # Exclusive create: neither an old trace nor the input can be overwritten.
    with args.output.open('x') as output, args.input.open('rb') as recorded:
        output.write(json.dumps(manifest, sort_keys=True) + '\n')
        with subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              text=True, bufsize=1) as process, selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            try:
                for tick in iter_reference_ticks(iter_tjvr_records(recorded), period_ns=period_ns, receiver=receiver):
                    process.stdin.write(encode_tick(tick))
                    process.stdin.flush()
                    if not selector.select(timeout=20):
                        raise TimeoutError(f'SPARK worker stalled at tick {tick.tick_id}')
                    line = process.stdout.readline()
                    row = json.loads(line)
                    if row.get('kind') != 'spark_bilateral_result' or row.get('tick_id') != tick.tick_id:
                        raise ValueError(f'unassociated SPARK result at tick {tick.tick_id}')
                    row['consumed_sequence'] = tick.sample.observation.frame.sequence if tick.sample else None
                    canonical = json.dumps(row, sort_keys=True, allow_nan=False) + '\n'
                    output.write(canonical)
                    digest.update(canonical.encode())
                    summary['ticks'] += 1
                    summary['applied_ticks'] += int(row['input_live'])
                    summary['guidance_accepted_ticks'] += int(row['guidance_accepted'])
                    summary['control_failures'] += int(row['control_executed'] and
                        not (row['left']['accepted'] and row['right']['accepted']))
                    summary['budget_exhausted_ticks'] += int(row['left']['budget_exhausted'] or row['right']['budget_exhausted'])
                process.stdin.close()
                if process.wait(timeout=10) != 0:
                    raise RuntimeError('SPARK worker failed')
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                if not process.stdin.closed:
                    process.stdin.close()
                process.stdout.close()
        summary['receiver'] = receiver.stats()
        summary['results_sha256'] = digest.hexdigest()
        output.write(json.dumps(summary, sort_keys=True) + '\n')
    print(json.dumps({'output': str(args.output), **summary}, indent=2))


if __name__ == '__main__':
    main()
