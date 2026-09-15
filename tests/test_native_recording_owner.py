import json
from pathlib import Path
import subprocess
import socket
import time
import tempfile
import unittest

import h5py

from tests.test_native_raw_input import compile_driver
from tests.test_native_session_publication import manifest
from tests.test_reference_tjvr_receiver import packet

ROOT = Path(__file__).resolve().parents[1]


class NativeRecordingOwnerTest(unittest.TestCase):
    def test_viewer_startup_failure_keeps_recording_incomplete(self):
        self._gateway_recording('spark_upper_qpoases_headroom_feedforward_velocity_qp',
                                'spark', viewer_failure=True)

    def test_actual_gateway_idle_shutdown_records_without_python_capture(self):
        self._gateway_recording('spark_upper_qpoases_headroom_feedforward_velocity_qp', 'spark')

    def test_actual_mapped_gateway_recording(self):
        self._gateway_recording('pico_ee_mapped_corrected_palm_velocity_qp', 'mapped_palm')

    def _gateway_recording(self, backend, prefix, viewer_failure=False):
        import importlib.util
        from tianji_teleop.recording.native_owner import NativeRecordingOwner
        from tianji_teleop.producers.spark.native_gateway import (
            NativeGatewayProcess, build_native_gateway_manifest, write_native_gateway_manifest)
        spec = importlib.util.spec_from_file_location('recording_cli', ROOT / 'scripts/vr_manus_live.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _, resolved = module.resolve(['--disable-hands', '--scheduler-backend', 'cpp', '--ik-backend', backend])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'gateway.h5'
            owner = NativeRecordingOwner(path, root=ROOT, router_zid='offline', robot_model='spark', metadata={})
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
                reservation.bind(('127.0.0.1', 0))
                port = reservation.getsockname()[1]
            config = build_native_gateway_manifest(ROOT, resolved, run_id='test', instance_id='offline',
                                                    router_zid='offline', tjvr_bind='127.0.0.1', tjvr_port=port)
            config.update(recording_fd=owner.fileno(), recording_origin_ns=owner.origin_ns)
            config['viewer_backend'] = 'cpp' if viewer_failure else 'python'
            config_path = write_native_gateway_manifest(Path(directory), config)
            gateway = NativeGatewayProcess(ROOT / 'build/control-native/tianji_native_session_gateway',
                config_path, prefix=prefix, recording_fd=owner.fileno(),
                viewer_backend=config['viewer_backend'])
            try:
                if viewer_failure:
                    from unittest.mock import patch
                    import os
                    with patch.dict(os.environ, {'DISPLAY':'', 'WAYLAND_DISPLAY':''}):
                        with self.assertRaises(RuntimeError):
                            gateway.start()
                    self.assertIn('GLFW initialization failed', gateway.stderr)
                    owner.release_descriptor()
                    gateway.close(graceful=False)
                    owner.close(complete=False)
                    with h5py.File(path) as file:
                        self.assertFalse(file.attrs['complete'])
                    return
                gateway.start()
                owner.release_descriptor()
                gateway.wait_for_cycle(5)
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    sender.sendto(packet(1), ('127.0.0.1', port))
                deadline = time.monotonic() + 5
                raw_seen = False
                while time.monotonic() < deadline:
                    frame = gateway.get_frame(.1)
                    if frame and frame.kind == 'raw':
                        raw_seen = True
                        break
                self.assertTrue(raw_seen, 'synthetic raw frame was not captured')
                gateway.send_action('shutdown')
                self.assertTrue(gateway.wait_for_complete(10))
                self.assertEqual(gateway.process.wait(timeout=5), 0, gateway.stderr)
                owner.close(complete=True)
            finally:
                gateway.close(graceful=False)
                owner.close(complete=False)
            with h5py.File(path) as file:
                self.assertTrue(file.attrs['complete'])
                self.assertGreater(len(file['joint/state/arm/time_ns']), 0)
                kinds=list(file['meta/dual_audit/kind'].asstr()[:])
                self.assertIn('operator_result', kinds)
                self.assertIn('native_cycle', kinds)
                self.assertEqual(len(file['raw/tjvr_upper_limb/time_ns']), 1)
                self.assertEqual(bytes(file['raw/tjvr_upper_limb/raw_packet'][0]), packet(1))

    def test_exclusive_file_fd_transfer_and_completion(self):
        self.assertTrue((ROOT / 'src/tianji_teleop/tianji_teleop/recording/native_owner.py').is_file())
        from tianji_teleop.recording.native_owner import NativeRecordingOwner
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root / 'sink'
            compile_driver('session_recording_sink_fixture.cpp', binary)
            for mode, approved in (('complete', True), ('complete', False), ('abandon', False)):
                path = root / f'{mode}-{approved}.h5'
                owner = NativeRecordingOwner(path, root=ROOT, router_zid='router', robot_model='spark', metadata={})
                try:
                    with self.assertRaises(FileExistsError):
                        NativeRecordingOwner(path, root=ROOT, router_zid='router', robot_model='spark', metadata={})
                    process = subprocess.Popen([str(binary), str(owner.fileno()), mode],
                        pass_fds=(owner.fileno(),), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    owner.release_descriptor()
                    _, errors = process.communicate(json.dumps(dict(manifest=manifest() | dict(worker_prefix='spark'),
                                                                   bytes=list(packet(1)))).encode(), timeout=10)
                    self.assertEqual(process.returncode, 0, errors)
                    owner.close(complete=approved)
                finally:
                    owner.close(complete=False)
                with h5py.File(path) as file:
                    self.assertEqual(bool(file.attrs['complete']), approved)
