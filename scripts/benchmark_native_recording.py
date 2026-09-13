#!/usr/bin/env python3
"""Offline mixed-stream recording stress test; no devices or control authority.

Uses one recorded raw TJVR packet and representative audit payloads as templates.
Generated sequences/timing are synthetic. Never treats output as device acceptance.
"""
import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))
import h5py
from tianji_teleop.protocol.messages import (
    ARM_JOINT_NAMES, ALL_ARM_JOINT_NAMES, HAND_JOINT_NAMES,
    ArmJointCommand, ArmJointState, HandJointCommand, HandJointState, SessionState)
from tianji_teleop.recording.async_dual import AsyncDualRecorder
from tianji_teleop.recording.live_capture import LiveCycleSnapshot
from tianji_teleop.hand_tracking.reference_tjvr import parse_reference_tjvr_packet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--template', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-s', type=float, default=120)
    parser.add_argument('--speed', type=float, default=1)
    args = parser.parse_args()
    if not 1 <= args.duration_s <= 3600 or not 0 < args.speed <= 10:
        parser.error('duration must be 1..3600; speed must be >0 and <=10')
    with h5py.File(args.template) as f:
        packet = bytes(f['raw/tjvr_upper_limb/raw_packet'][0])
        points = f['raw/manus_callbacks/points'][0].tolist()
        templates = {}
        group = f['meta/dual_audit']
        for offset in range(0, len(group['kind']), 1024):
            kinds = group['kind'].asstr()[offset:offset+1024]
            payloads = group['payload_json'].asstr()[offset:offset+1024]
            for kind, payload in zip(kinds, payloads):
                row = json.loads(payload)
                if kind == 'native_cycle' and row.get('native') is not None:
                    templates[kind] = row
                else:
                    templates.setdefault(kind, row)
            if (all(k in templates for k in ('native_cycle', 'component_status', 'hand_output'))
                    and templates['native_cycle'].get('native') is not None):
                break
    native = templates['native_cycle']
    if native.get('native') is None:
        parser.error('template must include active native IK cycles')
    telemetry = templates['component_status']
    source = SimpleNamespace(to_dict=lambda: native['source_status'])
    recorder = AsyncDualRecorder(args.output, router_zid='benchmark',
        metadata={'synthetic': True, 'scope': 'recording_throughput_only'})
    start = time.monotonic()
    counts = dict(control=0, manus=0, tjvr=0, hand=0)
    rates = dict(control=200, manus=240, tjvr=90, hand=50)
    failure = None
    try:
        while time.monotonic() - start < args.duration_s:
            elapsed = time.monotonic() - start
            for stream, rate in rates.items():
                expected = int(elapsed * rate * args.speed)
                while counts[stream] < expected:
                    counts[stream] += 1
                    seq = counts[stream]
                    now = time.monotonic_ns()
                    if stream == 'manus':
                        recorder.append('append_manus_callback', points, callback_sequence=seq,
                            received_timestamp_ns=now, receiver_instance_id='bench-manus', single_hand_side='right')
                        for kind in ('manus_rawviz_line', 'manus_callback_metadata', 'manus_superseded_input'):
                            recorder.append('append_dual_audit', kind, templates.get(kind, {}), received_timestamp_ns=now)
                    elif stream == 'tjvr':
                        observation = parse_reference_tjvr_packet(packet, receiver_instance_id='bench-pico',
                            receiver_frame_sequence=seq, received_timestamp_ns=now)
                        recorder.append('append_raw_reference_tjvr', observation)
                        recorder.append('append_dual_audit', 'tjvr_stream_decision',
                            templates.get('tjvr_stream_decision', {}), received_timestamp_ns=now)
                    elif stream == 'hand':
                        for side in ('left', 'right'):
                            command = HandJointCommand(1, seq, now, 'retarget', side,
                                list(HAND_JOINT_NAMES[side]), [0.] * 20, 'hand', 'benchmark')
                            recorder.append('append_hand_command', command, received_time_ns=now)
                        recorder.append('append_dual_audit', 'hand_output', templates['hand_output'], received_timestamp_ns=now)
                    else:
                        commands = {side: ArmJointCommand(1, seq, now, 'ik', side, 'teleop', seq, seq,
                            list(ARM_JOINT_NAMES[side]), [0.] * 7, 'arm', 'benchmark') for side in ('left', 'right')}
                        hands = {side: HandJointState(1, seq, now, 'mujoco', side,
                            list(HAND_JOINT_NAMES[side]), [0.] * 20, None, 'state', 'benchmark') for side in ('left', 'right')}
                        arm = ArmJointState(1, seq, now, 'mujoco', list(ALL_ARM_JOINT_NAMES),
                            [0.] * 14, None, 'state', 'benchmark')
                        state = SessionState(1, seq, now, 'teleop', 'benchmark', 'coordinator', seq, 'co', 'benchmark')
                        result = SimpleNamespace(commands=commands, native_result=native.get('native'),
                            native_attempt=native.get('native_attempt'), receipt_accepted=True)
                        hand_commands = {side: HandJointCommand.from_dict(row)
                            for side, row in native.get('accepted_hand_commands', {}).items()}
                        snapshot = LiveCycleSnapshot(result, source, None, None, arm, state,
                            telemetry, hands, hand_commands, 1, native.get('coordinator_receipt'),
                            native.get('bilateral_command'), now)
                        recorder.append_live_cycle_snapshot(snapshot, run_id='benchmark')
            if recorder.failure:
                raise RuntimeError(recorder.failure)
            time.sleep(.001)
    except Exception as exc:
        failure = str(exc)
    finally:
        try:
            recorder.close(complete=failure is None)
        except Exception as exc:
            failure = failure or str(exc)
    with h5py.File(args.output) as f:
        saved = dict(control=len(f['joint/state/arm/time_ns']),
            manus=len(f['raw/manus_callbacks/time_ns']), tjvr=len(f['raw/tjvr_upper_limb/time_ns']),
            hand=len(f['joint/command/hand/left/time_ns']))
        complete = bool(f.attrs['complete'])
    load_met = all(counts[stream] >= .95 * rate * args.speed * args.duration_s
                   for stream, rate in rates.items())
    passed = failure is None and complete and counts == saved and load_met
    print(json.dumps(dict(passed=passed, synthetic=True, error=failure, complete=complete,
        generated=counts, saved=saved, load_met=load_met, speed=args.speed, duration_s=time.monotonic()-start,
        statistics=recorder.statistics, output=str(args.output)), indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
