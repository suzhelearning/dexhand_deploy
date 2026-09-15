#!/usr/bin/env python3
"""Compare post-driver parsers on a recorded rawviz prefix; no device/network I/O."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/tianji_teleop'))
sys.path.insert(0,str(ROOT))
import h5py
import numpy as np
from scripts.benchmark_native_publication import summary
from tianji_teleop.hand_tracking.reference_manus import HandInputAssembler,RawvizHandInputProcessor
from tianji_teleop.hand_tracking.native_manus_parser import NativeManusProcessor


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording',type=Path,required=True)
    parser.add_argument('--lines',type=int,default=2000)
    parser.add_argument('--allow-incomplete-prefix',action='store_true')
    args=parser.parse_args()
    if not 1<=args.lines<=10000: parser.error('lines must be in 1..10000')
    digest=hashlib.sha256()
    with args.recording.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    lines=[]
    with h5py.File(args.recording) as file:
        complete=bool(file.attrs.get('complete',False))
        if not complete and not args.allow_incomplete_prefix:
            parser.error('recording incomplete; explicitly allow prefix-only analysis')
        audit=file['meta/dual_audit'];run=None;last_time=None
        for start in range(0,len(audit['kind']),1000):
            for offset,kind in enumerate(audit['kind'].asstr()[start:start+1000]):
                if kind!='manus_rawviz_line':continue
                i=start+offset;row=json.loads(audit['payload_json'].asstr()[i]);stamp=int(audit['time_ns'][i])
                if (type(row['line_sequence']) is not int or row['line_sequence']!=len(lines)+1 or
                        not isinstance(row['run_id'],str) or not row['run_id'].strip() or row['terminator']!='LF' or
                        row['input_stage']!='rawviz_stdout_before_parser' or
                        (run is not None and row['run_id']!=run) or
                        (last_time is not None and stamp<last_time)):
                    raise ValueError('inconsistent rawviz prefix identity/order')
                run=row['run_id'];last_time=stamp;lines.append(row['text'])
                if len(lines)>=args.lines:break
            if len(lines)>=args.lines:break
    if not lines:raise ValueError('recording has no rawviz lines')
    timings={'python':[],'cpp':[]};count=0
    for repeat in range(4):
        outputs={'python':[],'cpp':[]}
        reference=RawvizHandInputProcessor(HandInputAssembler(),outputs['python'].append)
        native=NativeManusProcessor(outputs['cpp'].append)
        try:
            for index,line in enumerate(lines):
                implementations=[('python',reference),('cpp',native)]
                if (repeat+index)%2:implementations.reverse()
                for name,implementation in implementations:
                    started=time.perf_counter_ns();implementation.process_line(line)
                    elapsed=time.perf_counter_ns()-started
                    if repeat:timings[name].append(elapsed)
                if len(outputs['python'])!=len(outputs['cpp']):
                    raise ValueError(f'callback count mismatch at line {index+1}')
            count=len(outputs['python'])
            if not count:raise ValueError('prefix produced no bilateral callbacks')
            for a,b in zip(outputs['python'],outputs['cpp']):
                np.testing.assert_array_equal(a.values,b.values)
                if a.sequences!=b.sequences or a.source_timestamps_ns!=b.source_timestamps_ns:
                    raise ValueError('callback metadata mismatch')
        finally:native.close()
    print(json.dumps(dict(scope='rawviz_prefix_parser_and_callback_only',sha256=digest.hexdigest(),
        recording_complete=complete,lines=len(lines),callbacks=count,repeats=3,mismatches=0,
        timings={k:summary(v) for k,v in timings.items()},end_to_end_measured=False,
        hardware_acceptance=False),indent=2))


if __name__=='__main__':main()
