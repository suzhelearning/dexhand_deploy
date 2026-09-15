import json
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest
import h5py
import numpy as np
from tests.test_native_raw_input import compile_driver
from tianji_teleop.recording.session_h5 import SessionH5Writer

ROOT=Path(__file__).resolve().parents[1]


class ManusHdf5Test(unittest.TestCase):
    def test_raw_callback_parity_and_invalid_aborts(self):
        self.assertTrue((ROOT/'native/control/manus_hdf5.hpp').is_file(), 'missing native Manus recording')
        disk_binary=ROOT/'build/hdf5_recorder/tianji_hdf5_recorder'
        if not disk_binary.is_file(): self.skipTest('build-hdf5-recorder required')
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);binary=root/'encoder'
            compile_driver('manus_hdf5_fixture.cpp',binary)
            kwargs=dict(source_type='vr_manus_sim',robot_model='spark',router_zid='router',schema_version='1.2')
            rows=[dict(sequence=1,timestamp=100,text='SDK 启动\r'),
                  dict(sequence=2,timestamp=99,text='',points=[i/128 for i in range(126)])]
            expected=root/'expected.h5'
            with SessionH5Writer(expected,**kwargs) as writer:
                for r in rows:
                    writer.append_dual_audit('manus_rawviz_line',dict(line_sequence=r['sequence'],text=r['text'],
                        terminator='LF',input_stage='rawviz_stdout_before_parser',run_id='test-run'),
                        received_timestamp_ns=r['timestamp'])
                    if 'points' in r:
                        meta=dict(source_sequences=dict(right=4,left=5),source_timestamps_ns=dict(right=100,left=200))
                        writer.append_manus_callback(r['points'],callback_sequence=1,received_timestamp_ns=r['timestamp'],
                            receiver_instance_id='manus-source',**meta)
                        writer.append_dual_audit('manus_callback_metadata',dict(callback_sequence=1,
                            receiver_instance_id='manus-source',run_id='test-run',**meta),received_timestamp_ns=r['timestamp'])
            for index,bad in enumerate((None,dict(rows[0],sequence=0),dict(rows[0],text='x\0y'),
                                        dict(rows[0],text='x'*65537),dict(rows[0],timestamp=0))):
                actual=root/f'actual-{index}.h5';SessionH5Writer(actual,**kwargs).abort()
                parent,child=socket.socketpair()
                with parent,child:
                    disk=subprocess.Popen([str(disk_binary),str(actual)],stdin=child,stdout=child,stderr=subprocess.PIPE)
                    try:
                        child.close()
                        p=subprocess.run([str(binary),str(parent.fileno())],pass_fds=(parent.fileno(),),
                            input='\n'.join(map(json.dumps,rows if bad is None else [rows[0],bad])),
                            text=True,capture_output=True,timeout=10)
                        parent.close();disk.communicate(timeout=5)
                        self.assertEqual(p.returncode==0,bad is None,p.stderr)
                        if bad is None: self.assertEqual(disk.returncode,0)
                    finally:
                        if disk.poll() is None: disk.kill()
                        disk.communicate()
                with h5py.File(actual) as got:
                    self.assertEqual(bool(got.attrs['complete']),bad is None)
                    if bad is not None:
                        self.assertEqual(len(got['meta/dual_audit/time_ns']),1)
                        continue
                    with h5py.File(expected) as ref:
                        def compare(name,item):
                            other=got[name];self.assertEqual(dict(item.attrs),dict(other.attrs))
                            if not isinstance(item,h5py.Dataset): return
                            self.assertEqual(item.dtype,other.dtype,name)
                            if name.endswith(('payload_json','source_sequences_json','source_timestamps_ns_json')):
                                self.assertEqual(list(map(json.loads,item[:])),list(map(json.loads,other[:])),name)
                            else: np.testing.assert_array_equal(item[()],other[()],err_msg=name)
                        ref.visititems(compare)
