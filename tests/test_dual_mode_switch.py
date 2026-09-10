"""Owned-router integration: no hardware or shared user router."""
import inspect
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import zenoh
from scripts import pico_sim_smoke
from tianji_teleop.coordination.live_domain_guard import _control_token

ROOT = Path(__file__).resolve().parents[1]


class DualModeSwitchTest(unittest.TestCase):
    def test_smoke_accepts_explicit_test_owned_router(self):
        parameters = inspect.signature(pico_sim_smoke.main).parameters
        self.assertIn('test_router_endpoint', parameters)
        self.assertIn('test_runtime_directory', parameters)

    @unittest.skipUnless(os.environ.get('DUAL_SWITCH_TEST') == '1', 'owned-router full-process integration')
    def test_vr_pico_vr_switch_on_same_router_and_runtime(self):
        with tempfile.TemporaryDirectory(prefix='dual-switch-') as directory:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                endpoint = f'tcp/127.0.0.1:{probe.getsockname()[1]}'
            with open(Path(directory) / 'router.log', 'wb') as log:
                router = subprocess.Popen([str(ROOT / 'vendor/zenoh-router/zenohd'),
                    '-l', endpoint, '--no-multicast-scouting'], stdout=log, stderr=log)
            try:
                time.sleep(1)
                config = zenoh.Config()
                config.insert_json5('mode', '"client"')
                config.insert_json5('connect/endpoints', json.dumps([endpoint]))
                config.insert_json5('scouting/multicast/enabled', 'false')
                session = zenoh.open(config)
                try:
                    sentinel = session.liveliness().declare_token('tj/live/source/switch-test/owned')
                    try:
                        with self.assertRaises(AssertionError):
                            self.assert_no_controls(session)
                    finally:
                        sentinel.undeclare()
                    runtime = str(Path(directory) / 'runtime')
                    self.run_vr(endpoint, runtime)
                    self.assertIsNone(router.poll(), 'VR cleanup killed caller-owned router')
                    self.assert_no_controls(session)
                    with patch.object(sys, 'argv', ['pico_sim_smoke', '--official-hands-profile', '--gesture-start-test',
                            '--ik-backend', 'pico_ee_dexhand_qp', '--joint-limit-source', 'urdf']):
                        self.assertEqual(pico_sim_smoke.main(test_router_endpoint=endpoint,
                            test_runtime_directory=runtime), 0)
                    self.assertIsNone(router.poll(), 'PICO cleanup killed caller-owned router')
                    self.assert_no_controls(session)
                    self.run_vr(endpoint, runtime)
                    self.assertIsNone(router.poll())
                    self.assert_no_controls(session)
                finally:
                    session.close()
            finally:
                router.terminate()
                try:
                    router.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    router.kill()
                    router.wait()

    def run_vr(self, endpoint, runtime):
        # Startup/shutdown authority test only: no synthetic VR authorization
        # and no Manus source. Full joint motion is covered independently.
        result = subprocess.run(['bash', 'scripts/run_session.sh', '--profile',
            'vr_manus_sim', '--disable-hands', '--headless', '--duration-s', '2',
            '--tjvr-port', '0'], cwd=ROOT, env=dict(os.environ,
            TIANJI_ROUTER_ENDPOINT=endpoint, TIANJI_TELEOP_RUNTIME_DIR=runtime),
            capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def assert_no_controls(self, session):
        keys = []
        for reply in session.liveliness().get('tj/live/**', timeout=1):
            self.assertTrue(reply.ok, 'authority inventory failed')
            key = str(reply.result.key_expr)
            if _control_token(key):
                keys.append(key)
        self.assertEqual(keys, [], 'stale control authority after managed shutdown')
