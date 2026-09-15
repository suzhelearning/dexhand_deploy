import json
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest

import h5py
import numpy as np

from tests.test_native_raw_input import compile_driver
from tests.test_native_session_publication import manifest
from tests.test_reference_tjvr_receiver import packet
from tianji_teleop.recording.session_h5 import SessionH5Writer

ROOT = Path(__file__).resolve().parents[1]


class SessionRecordingSinkTest(unittest.TestCase):
    def test_events_raw_close_and_failure_poisoning(self):
        self.assertTrue((ROOT / 'native/control/session_recording_sink.hpp').is_file(),
                        'native recording sink is missing')
        disk_binary = ROOT / 'build/hdf5_recorder/tianji_hdf5_recorder'
        if not disk_binary.is_file(): self.skipTest('build-hdf5-recorder required')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root / 'sink'
            compile_driver('session_recording_sink_fixture.cpp', binary)
            for mode in ('complete', 'incomplete', 'abandon', 'cancel', 'invalid', 'manus', 'manus_batch', 'missing_manus'):
                with self.subTest(mode=mode):
                    path = root / (mode+'.h5')
                    SessionH5Writer(path, source_type='vr_manus_sim', robot_model='spark',
                                    router_zid='router', schema_version='1.2').abort()
                    parent, child = socket.socketpair()
                    with parent, child:
                        disk = subprocess.Popen([str(disk_binary), str(path)], stdin=child,
                                                stdout=child, stderr=subprocess.PIPE)
                        try:
                            child.close()
                            config=manifest() | dict(worker_prefix='spark')
                            if mode!='missing_manus':
                                config['manus_source_authority']=dict(logical='manus',instance='manus-source',router='router')
                            result = subprocess.run([str(binary), str(parent.fileno()), mode],
                                input=json.dumps(dict(manifest=config, bytes=list(packet(1)))),
                                pass_fds=(parent.fileno(),), text=True, capture_output=True, timeout=10)
                            self.assertEqual(result.returncode, 0, result.stderr)
                            parent.close()
                            disk.communicate(timeout=5)
                        finally:
                            if disk.poll() is None: disk.kill()
                            disk.communicate()
                    with h5py.File(path) as file:
                        self.assertEqual(bool(file.attrs['complete']), mode in ('complete','manus','manus_batch'))
                        np.testing.assert_array_equal(file['raw/tjvr_upper_limb/raw_packet'][0], list(packet(1)))
                        self.assertEqual(file['raw/tjvr_upper_limb/time_ns'][0], -1)
                        audit=file['meta/dual_audit']
                        rows=list(map(json.loads,audit['payload_json'][:]))
                        self.assertEqual(rows[0]['stage'],'opening')
                        self.assertEqual(rows[1],dict(kind='operator_result',action='start',accepted=True,
                                                     reason='accepted',run_id='test-run'))
                        self.assertEqual(audit['received_timestamp_ns'][1],101)
                        self.assertEqual(rows[2]['receiver_frame_sequence'],1)
                        if mode in ('complete','incomplete','manus','manus_batch'):
                            self.assertEqual(rows[-1]['stage'],'native_recording_close')
                            self.assertEqual(rows[-1]['complete'],mode in ('complete','manus','manus_batch'))
                            if mode=='manus_batch':
                                self.assertEqual([r['line_sequence'] for r in rows[3:-1]],list(range(1,131)))
                                self.assertTrue(all(r['text']=='SDK ready' for r in rows[3:-1]))
                            if mode=='manus':
                                self.assertEqual(rows[3]['text'],'SDK ready')
                                self.assertEqual(rows[3]['line_sequence'],1)
                        else:
                            self.assertEqual(len(rows),3)
