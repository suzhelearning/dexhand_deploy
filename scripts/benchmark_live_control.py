#!/usr/bin/env python3
"""Offline complete control-core A/B; no sockets, viewer, hands or recording.

Normal native wall-clock QP budgets remain active, so differing results are
reported and must not automatically be attributed to the guard implementation.
"""
import argparse
from contextlib import ExitStack
from itertools import islice
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/tianji_teleop'))
from benchmark_native_results import summary
from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND, MAPPED_PALM_BACKEND
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.hand_tracking.spark_replay import iter_reference_ticks
from tianji_teleop.hand_tracking.tjvr_trace import iter_tjvr_records
from tianji_teleop.producers.spark.live_simulation import SparkLiveSimulation
from tianji_teleop.producers.spark.backend_assets import bilateral_assets


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',type=Path,required=True)
    parser.add_argument('--cycles',type=int,default=1000)
    parser.add_argument('--comparison', choices=('guard', 'simulation', 'coordinator'), default='guard')
    parser.add_argument('--deterministic-test', action='store_true',
        help='offline value parity only; replaces workers with relaxed-budget instances')
    parser.add_argument('--backend',choices=(SPARK_BACKEND,MAPPED_PALM_BACKEND),default=MAPPED_PALM_BACKEND)
    args=parser.parse_args()
    if not 2<=args.cycles<=100000:
        parser.error('cycles must be in 2..100000')
    with args.trace.open('rb') as stream:
        records=list(iter_tjvr_records(stream))
    receiver=ReferenceTjvrReceiver('bench-source',.15,.6,
        **({'target_source':'mapped_corrected_palm'} if args.backend==MAPPED_PALM_BACKEND else {}))
    ticks=list(islice(iter_reference_ticks(records,receiver=receiver),args.cycles))
    now=[ticks[0].now_ns]
    measurements={key:{} for key in ('python','cpp')}
    native_count={key:0 for key in measurements}
    differing=0
    feedback_differing=0
    first_difference=None
    with ExitStack() as stack:
        cores={}
        for key in measurements:
            core=SparkLiveSimulation(ROOT,run_id='bench',router_zid='offline',instance_id='bench',
                clock=lambda:now[0],backend=args.backend,native_result_format='binary',
                execution_guard=key if args.comparison == 'guard' else 'cpp',
                simulation_backend=key if args.comparison == 'simulation' else 'python',
                coordinator_math=key if args.comparison == 'coordinator' else 'python')
            stack.callback(core.close)
            if args.deterministic_test:
                # Offline harness only. No live CLI/config may select relaxed
                # solver budgets. The core has not consumed input or started.
                selected = bilateral_assets(ROOT, args.backend)
                replacement = selected['client'](
                    **{k:selected[k] for k in ('worker','config','model','urdf')},
                    result_format='binary', startup_handshake=True, deterministic_test=True)
                stack.callback(replacement.close)
                core.producer.backend.close()
                core.producer.backend = replacement
            cores[key]=core
        for i,tick in enumerate(ticks):
            now[0]=tick.now_ns
            results={}
            for key in (('python','cpp') if i%2 else ('cpp','python')):
                core=cores[key]
                start=time.perf_counter()
                result=core.step(tick.sample)
                elapsed=time.perf_counter()-start
                for name,value in dict(total=elapsed,**core.last_timing).items():
                    measurements[key].setdefault(name,[]).append(value)
                if core.failure or core.coordinator.state.state=='fault':
                    raise RuntimeError(core.failure or core.coordinator.state.reason)
                if core.coordinator.state.state=='idle':
                    core.request('start')
                results[key]=result.native_result
                native_count[key]+=result.native_result is not None
            differing+=results['python']!=results['cpp']
            if results['python'] != results['cpp'] and first_difference is None:
                first_difference = dict(index=i, python=results['python'], cpp=results['cpp'])
            feedback_differing += (
                cores['python'].sim.arm_state != cores['cpp'].sim.arm_state or
                cores['python'].sim.data.qpos.tolist() != cores['cpp'].sim.data.qpos.tolist() or
                cores['python'].sim.data.xpos.tolist() != cores['cpp'].sim.data.xpos.tolist())
        if not all(native_count.values()):
            raise RuntimeError('trace did not start native control; no active-cycle benchmark')
    print(json.dumps(dict(scope='offline_control_core_only',backend=args.backend,cycles=len(ticks),
        comparison=args.comparison, native_cycles=native_count,
        differing_native_results=differing, differing_simulation_feedback=feedback_differing,
        deterministic_test=args.deterministic_test, first_difference=first_difference,
        real_time_qualified=False,
        timing={k:{name:summary(values) for name,values in stages.items()}
                for k,stages in measurements.items()}),indent=2))
    return 1 if args.deterministic_test and (differing or feedback_differing) else 0


if __name__=='__main__':
    raise SystemExit(main())
