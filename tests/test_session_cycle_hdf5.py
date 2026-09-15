"""Complete native control-cycle recording parity, including failed IK attempts."""
import json
from dataclasses import replace
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import unittest

import h5py
import numpy as np

from tests.test_native_raw_input import compile_driver
from tests.test_native_session_publication import manifest, cycle
from tests.test_reference_tjvr_receiver import packet
from tianji_teleop.hand_tracking.native_binary_results import _SPECS, _STRUCTS, decode_result
from tianji_teleop.producers.spark.native_live_runner import _typed_cycle, _frame_sample
from tianji_teleop.recording.session_h5 import SessionH5Writer
from tianji_teleop.protocol.messages import HandJointCommand, HandJointState, HAND_JOINT_NAMES

ROOT = Path(__file__).resolve().parents[1]


class SessionCycleHdf5Test(unittest.TestCase):
    def test_complete_cycle_matches_python(self):
        self.assertTrue((ROOT / 'native/control/session_cycle_hdf5.hpp').is_file(),
                        'native full-cycle recording encoder missing')
        disk_binary = ROOT / 'build/hdf5_recorder/tianji_hdf5_recorder'
        if not disk_binary.is_file(): self.skipTest('build-hdf5-recorder required')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root / 'cycle'
            compile_driver('session_cycle_hdf5_fixture.cpp', binary)
            for prefix, hands in (('spark', False), ('mapped_palm', False), ('spark', True), ('mapped_palm', True)):
                config = manifest() | dict(worker_prefix=prefix)
                if hands:
                    config['hand_authorities'] = {
                        'producer': dict(logical='official_wuji_hand2', instance='hand', router='router'),
                        **{s: dict(logical='wuji_'+s, instance='sim', router='router') for s in ('left', 'right')}}
                if prefix == 'mapped_palm': config['algorithm'] = 'pico_ee_mapped_corrected_palm_velocity_qp'
                values = []
                for key, fmt in _SPECS[prefix]:
                    if fmt.endswith('d'):
                        values.extend([0.25] * int(fmt[:-1] or '1'))
                    else:
                        values.append(1 if fmt == 'B' else 9)
                wire = struct.pack('<4sBBH', b'TJBR', 1, 2 if prefix == 'mapped_palm' else 1,
                                   _STRUCTS[prefix].size) + _STRUCTS[prefix].pack(*values)
                decoded = decode_result(wire, prefix)
                expected, actual = root / (prefix+str(hands)+'-expected.h5'), root / (prefix+str(hands)+'-actual.h5')
                kwargs = dict(source_type='vr_manus_sim', robot_model='spark', router_zid='router', schema_version='1.2')
                writer = SessionH5Writer(expected, **kwargs)
                inputs = []
                for i, phase in enumerate(('idle', 'teleop', 'returning', 'fault')):
                    row = cycle(phase, None if i == 0 else decoded)
                    row['timestamp_ns'] += i
                    if i == 1: row['timestamp_ns'] = 9999
                    row['ik_adopted'] = i == 1
                    if i:
                        row['request'] = dict(packet=packet(2), source_sequence=37, received_ns=9000,
                                              generation=2, discontinuity=True)
                    if i == 2:
                        row['request']['packet'] = b''
                        row['request']['received_ns'] = 0
                    sample = _frame_sample(row, receiver_instance_id='source')
                    snapshot = _typed_cycle(row, manifest=config, prefix=prefix, sample=sample)
                    if hands:
                        row['hand_results'] = []
                        if i in (1, 2):
                            row['hand_results'].append(dict(run_id=config['run_id'],
                                producer_authority=config['hand_authorities']['producer'],
                                observed_timestamp_ns=9500, accepted=i==1, reason='accepted' if i==1 else 'hand result expired',
                                result=dict(output_sequence=i,input_sequence=8,input_timestamp_ns=9000,epoch=3,
                                    scheduler_timestamp_ns=9400,valid_flags=3,phase=i,status=2 if i==1 else 3,
                                    positions_rad=[0.1]*40)))
                        row['hand_feedback'] = [[0.1]*20, [0.2]*20]
                        # A single-sided update and bilateral Home, with no fabricated idle/fault commands.
                        row['hand_commands'] = {'left': [0.3]*20} if i == 1 else (
                            {'left': [0.]*20, 'right': [0.]*20} if i == 2 else {})
                        states = {s: HandJointState(1, row['ticks'], row['timestamp_ns'], 'wuji_'+s,
                            s, list(HAND_JOINT_NAMES[s]), row['hand_feedback'][j], None, 'sim', 'router')
                            for j, s in enumerate(('left', 'right'))}
                        commands = {s: HandJointCommand(1, row['ticks'], row['timestamp_ns'], 'official_wuji_hand2',
                            s, list(HAND_JOINT_NAMES[s]), q, 'hand', 'router') for s, q in row['hand_commands'].items()}
                        snapshot = replace(snapshot, hand_states=states, hand_commands=commands)
                        for command in commands.values():
                            writer.append_hand_command(command, received_time_ns=row['timestamp_ns'])
                    writer.append_live_cycle_snapshot(snapshot, run_id=config['run_id'])
                    if i: row['request']['packet'] = list(row['request']['packet'])
                    inputs.append(dict(cycle=row, manifest=config, origin_ns=10000, **(dict(wire=list(wire)) if i else {})))
                writer.close()
                SessionH5Writer(actual, **kwargs).abort()
                parent, child = socket.socketpair()
                with parent, child:
                    disk = subprocess.Popen([str(disk_binary), str(actual)], stdin=child, stdout=child, stderr=subprocess.PIPE)
                    try:
                        child.close()
                        result = subprocess.run([str(binary), str(parent.fileno())], pass_fds=(parent.fileno(),),
                            input='\n'.join(map(json.dumps, inputs)), text=True, capture_output=True, timeout=10)
                        self.assertEqual(result.returncode, 0, result.stderr)
                        parent.close()
                        _, errors = disk.communicate(timeout=5)
                        self.assertEqual(disk.returncode, 0, errors)
                    finally:
                        if disk.poll() is None: disk.kill()
                        disk.communicate()
                with h5py.File(expected) as left, h5py.File(actual) as right:
                    self.assertEqual(dict(left.attrs), dict(right.attrs))
                    def compare(name, item):
                        self.assertEqual(dict(item.attrs), dict(right[name].attrs), name)
                        if isinstance(item, h5py.Dataset):
                            self.assertEqual(item.dtype, right[name].dtype, name)
                            if name.endswith('payload_json'):
                                wanted=list(map(json.loads, item[:]))
                                if hands:
                                    for payload, original in zip(wanted, inputs):
                                        payload['native_hand_results']=original['cycle']['hand_results']
                                self.assertEqual(wanted, list(map(json.loads, right[name][:])))
                            else:
                                np.testing.assert_array_equal(item[()], right[name][()], err_msg=name)
                    left.visititems(compare)
