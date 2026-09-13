"""Exercise the actual VR launcher Ctrl-C cleanup with a native recorder.

Only authority/device setup is stubbed. The launcher, signals, dispatcher,
C++ child and HDF5 close are real; no SDK, router or robot is opened.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import h5py

ROOT = Path(__file__).resolve().parents[1]


class VrManusShutdownTest(unittest.TestCase):
    def test_graceful_stop_rejects_reused_pid_identity(self):
        process = subprocess.Popen(
            ['bash', '-c', 'trap "exit 0" TERM INT; echo ready; while true; do sleep 0.05; done'],
            start_new_session=True,
            stdout=subprocess.PIPE,
            text=True,
        )
        try:
            self.assertEqual(process.stdout.readline().strip(), 'ready')
            result = subprocess.run(
                ['bash', '-c',
                 'source "$1"; graceful_stop_session_owner "$2" 1 wrong-start-ticks',
                 '_', str(ROOT / 'scripts/graceful_process_stop.sh'), str(process.pid)],
                capture_output=True,
                text=True,
                timeout=5,
            )
            self.assertEqual(result.returncode, 1)
            self.assertIsNone(process.poll())
        finally:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            process.stdout.close()

    def test_unresponsive_owner_returns_to_group_cleanup_after_bounded_wait(self):
        process = subprocess.Popen(['bash', '-c', 'trap "" TERM INT; echo ready; exec sleep 100'],
            start_new_session=True, stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(process.stdout.readline().strip(), 'ready')
            started = time.monotonic()
            result = subprocess.run(['bash', '-c',
                'source "$1"; graceful_stop_session_owner "$2" 1', '_',
                str(ROOT / 'scripts/graceful_process_stop.sh'), str(process.pid)], timeout=5)
            self.assertEqual(result.returncode, 1)
            self.assertLess(time.monotonic() - started, 5)
            self.assertIsNone(process.poll())
        finally:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            process.stdout.close()

    def test_ctrl_c_drains_native_recorder_before_group_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('run_vr_manus_session.sh', 'graceful_process_stop.sh'):
                (root / name).write_text((ROOT / 'scripts' / name).read_text())
            (root / 'common.sh').write_text('''
activate_bundle_runtime() { :; }
acquire_teleop_guard() { export TELEOP_RUNTIME_DIR="$TEST_DIRECTORY"; }
read_teleop_node_list() { :; }
assert_profile_domains_free() { :; }
read_router_zid() { echo test; }
new_instance_id() { echo test; }
register_teleop_process_group() { :; }
teleop_cleanup_and_release() { kill -TERM -- "-$dual_pid" 2>/dev/null || true; wait "$dual_pid" 2>/dev/null || true; }
''')
            (root / 'vr_manus_live.py').write_text('''
import os, signal, sys, time
from pathlib import Path
from threading import Event
if '--check' in sys.argv: raise SystemExit(0)
from tianji_teleop.recording.async_dual import AsyncDualRecorder
stop = Event()
signal.signal(signal.SIGINT, lambda *_: stop.set())
signal.signal(signal.SIGTERM, lambda *_: stop.set())
root = Path(os.environ['TEST_DIRECTORY'])
recorder = AsyncDualRecorder(root / 'capture.h5', router_zid='test', metadata={})
for sequence in range(200):
    recorder.append('append_dual_audit', 'lifecycle', {'i': sequence}, received_timestamp_ns=sequence+1)
(root / 'ready').write_text(str(os.getpid()))
stop.wait(15)
time.sleep(.2)
recorder.close()
(root / 'closed').write_text(str(recorder._writer._process.returncode))
''')
            env = dict(os.environ, TEST_DIRECTORY=str(root),
                       PATH=str(Path(sys.executable).parent) + ':' + os.environ['PATH'],
                       PYTHONPATH=str(ROOT / 'src/tianji_teleop'))
            process = subprocess.Popen(['bash', str(root / 'run_vr_manus_session.sh')],
                env=env, start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            owner = None
            try:
                deadline = time.monotonic() + 10
                while not (root / 'ready').exists() and time.monotonic() < deadline:
                    self.assertIsNone(process.poll())
                    time.sleep(.02)
                self.assertTrue((root / 'ready').exists())
                owner = int((root / 'ready').read_text())
                process.send_signal(signal.SIGINT)
                stdout, stderr = process.communicate(timeout=10)
                self.assertEqual(process.returncode, 130, (stdout, stderr))
                self.assertTrue((root / 'closed').exists(), (stdout, stderr))
                self.assertEqual((root / 'closed').read_text(), '0')
                with h5py.File(root / 'capture.h5') as f:
                    self.assertTrue(f.attrs['complete'])
                    self.assertEqual(len(f['meta/dual_audit/kind']), 200)
            finally:
                for group in (owner, process.pid):
                    if group is not None:
                        try:
                            os.killpg(group, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                process.communicate(timeout=5)
