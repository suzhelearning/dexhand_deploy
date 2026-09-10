#!/usr/bin/env python3
"""Deterministic offline TJVR→SPARK→coordinator→MuJoCo smoke; no devices.

This verifies the arm integration, not a synchronized VR+Manus hardware take.
The live profile and realtime qualification are separate acceptance gates.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import subprocess

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tjvr', type=Path, required=True)
    parser.add_argument('--record', type=Path, help='new session HDF5 1.2 path; refuses overwrite')
    hands = parser.add_mutually_exclusive_group(required=True)
    hands.add_argument('--disable-hands', action='store_true', help='arm-only smoke')
    hands.add_argument('--manus-recording', type=Path,
                       help='independent recorded Manus21 pickle; NOT synchronized with TJVR')
    args = parser.parse_args()
    from tianji_teleop.hand_tracking.spark_replay import iter_reference_ticks
    from tianji_teleop.hand_tracking.tjvr_trace import iter_tjvr_records
    from tianji_teleop.producers.spark.simulation import OfflineSparkSimulation
    from tianji_teleop.producers.hand_retarget import OfficialHandClient, HandRetargetProducer
    from tianji_teleop.protocol.messages import strict_loads
    from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
    from tianji_teleop.recording.session_h5 import SessionH5Writer
    callbacks = []
    manuscript = None
    python = ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python'
    if args.manus_recording is not None:
        # Decode in pinned NumPy; no untrusted pickle loading in the runtime.
        exported = subprocess.run([str(python), str(ROOT / 'scripts/export_manus_callbacks.py'),
            '--input', str(args.manus_recording)], capture_output=True, timeout=30, check=True)
        rows = [strict_loads(line) for line in exported.stdout.splitlines()]
        if (len(rows) < 3 or rows[0].get('kind') != 'manus_callback_header' or
                rows[-1].get('kind') != 'manus_callback_complete' or
                rows[-1].get('frames') != len(rows) - 2):
            raise ValueError('incomplete Manus callback export')
        manuscript, callbacks = rows[0], rows[1:-1]
    writer = None
    def record_cycle(result, commands, state, session_state):
        if writer is not None:
            for command in commands.values():
                writer.append_arm_command(command, received_time_ns=result['timestamp_ns'])
            writer.append_arm_state(state, received_time_ns=result['timestamp_ns'])
            writer.append_session_state(session_state, received_time_ns=result['timestamp_ns'])
    simulation = OfflineSparkSimulation(ROOT, hand_sides=() if args.disable_hands else ('left', 'right'),
                                        cycle_sink=record_cycle)
    hand_client = None
    recording_complete = False
    try:
        if args.record is not None:
            writer = SessionH5Writer(args.record, source_type='vr_manus_sim', schema_version='1.2',
                robot_model='spark/marvin_m6_wuji2.xml', router_zid='offline', metadata=dict(
                    input_mode='vr_manus', simulation_only=True, deterministic_test=True,
                    synchronized_hardware_recording=False, control_period_ns=5_000_000,
                    input_sha256=hashlib.sha256(args.tjvr.read_bytes()).hexdigest(),
                    manus_input_sha256=manuscript['input_sha256'] if manuscript else None))
        receiver = ReferenceTjvrReceiver('offline-replay', .15, .6,
            raw_frame_sink=writer.append_raw_reference_tjvr if writer else None)
        hand_producer = None
        if manuscript is not None:
            hand_client = OfficialHandClient(python=python, script=ROOT / 'scripts/wuji_hand_worker.py')
            hand_producer = HandRetargetProducer(hand_client, publisher_instance_id='official-hand',
                router_zid='offline', coordinator_instance_id='coord', receiver_instance_id='manus-replay',
                freshness_ns=200_000_000)
        callback_index = hand_commands = 0
        maximum_hand_error = 0.
        with args.tjvr.open('rb') as stream:
            for tick in iter_reference_ticks(iter_tjvr_records(stream), receiver=receiver):
                simulation.step(tick)
                if hand_producer is None:
                    continue
                hand_producer.update_session(simulation.coordinator.state)
                while (callback_index < len(callbacks) and
                       1_000_000_000 + callbacks[callback_index]['relative_receive_ns'] <= tick.now_ns):
                    row = callbacks[callback_index]
                    received = 1_000_000_000 + row['relative_receive_ns']
                    if writer is not None:
                        writer.append_manus_callback(row['points'], callback_sequence=row['callback_sequence'],
                            received_timestamp_ns=received, receiver_instance_id='manus-replay')
                    if not hand_producer.update_input(row['points'], sequence=row['callback_sequence'],
                            timestamp_ns=received, receiver_instance_id='manus-replay', now_ns=tick.now_ns):
                        raise RuntimeError(hand_producer.reason or 'Manus callback rejected')
                    commands = hand_producer.commands(tick.now_ns)
                    for command in commands.values():
                        if not simulation.sim.on_hand_command(command):
                            raise RuntimeError('official hand command rejected by MuJoCo')
                    applied = simulation.sim.tick(now_ns=tick.now_ns)
                    for side, command in commands.items():
                        if side not in applied['hand']:
                            raise RuntimeError('missing hand execution')
                        actual = simulation.sim.hand_state(side).position_rad
                        error = max(abs(a - b) for a, b in zip(actual, command.position_rad))
                        maximum_hand_error = max(maximum_hand_error, error)
                        if error != 0:
                            raise RuntimeError('simulation hand state differs from official output')
                        if writer is not None:
                            writer.append_hand_command(command, received_time_ns=tick.now_ns)
                            writer.append_hand_state(simulation.sim.hand_state(side), received_time_ns=tick.now_ns)
                    hand_commands += len(commands)
                    callback_index += 1
        if callback_index != len(callbacks):
            raise RuntimeError('TJVR schedule ended before Manus recording; full smoke incomplete')
        report = simulation.report()
        if report['control_ticks'] == 0:
            raise RuntimeError('no authorized control ticks; smoke did not exercise execution')
        report.update(kind='vr_manus_offline_sim_smoke', hands_enabled=hand_producer is not None,
                      synchronized_hardware_recording=False,
                      schedule='independent receive timelines aligned at zero; fixed 5ms control clock',
                      manus_callbacks=callback_index, hand_commands=hand_commands,
                      maximum_hand_sim_error_rad=maximum_hand_error,
                      manus_input_sha256=manuscript['input_sha256'] if manuscript else None,
                      input_sha256=hashlib.sha256(args.tjvr.read_bytes()).hexdigest())
        if writer is not None:
            writer.close()
        recording_complete = True
        print(json.dumps(report, allow_nan=False))
    finally:
        try:
            if writer is not None and not recording_complete:
                writer.abort()
        finally:
            try:
                if hand_client is not None:
                    hand_client.close()
            finally:
                simulation.close()


if __name__ == '__main__':
    main()
