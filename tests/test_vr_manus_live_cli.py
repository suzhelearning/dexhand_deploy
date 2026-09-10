import json
import os
from pathlib import Path
import subprocess
import sys
import socket
import time
from threading import Event, Thread
import tempfile
import unittest
import importlib.util
from unittest.mock import patch
from contextlib import redirect_stderr
import io

ROOT = Path(__file__).resolve().parents[1]


class VrManusLivePreflightTest(unittest.TestCase):
    def test_sdk_directory_is_resolved_and_hashed_without_global_environment_changes(self):
        spec = importlib.util.spec_from_file_location('vr_sdk_contract', ROOT / 'scripts/vr_manus_live.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            library = Path(directory) / 'libManusSDK_Integrated.so'
            library.touch()
            original = dict(os.environ)
            with patch.object(Path, 'is_file', return_value=True), \
                    patch.object(Path, 'read_bytes', return_value=b'fixture asset'):
                args, resolved = module.resolve(['--manus-rawviz', sys.executable,
                    '--manus-user', 'fixture', '--manus-library-dir', directory])
            self.assertEqual(str(args.manus_library_dir), directory)
            self.assertIn('external/manus/sdk/libManusSDK_Integrated.so', resolved['asset_sha256'])
            self.assertEqual(resolved['manus_sdk_loading'], 'explicit_child_path')
            self.assertEqual(dict(os.environ), original)

    def test_single_left_config_records_actual_parser_sides(self):
        spec = importlib.util.spec_from_file_location('vr_left_contract', ROOT / 'scripts/vr_manus_live.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        from tianji_teleop.config_loader import load_yaml
        config = load_yaml(ROOT / 'src/tianji_teleop/config/sessions/vr_manus_sim.yaml')
        config['active_hand_sides'] = ['left']
        # This is pure config resolution, with no native environment/device requirement.
        with patch('tianji_teleop.config_loader.load_yaml', return_value=config), \
                patch.object(Path, 'is_file', return_value=True), \
                patch.object(Path, 'read_bytes', return_value=b'fixture asset'):
            _, resolved = module.resolve(['--manus-rawviz', sys.executable, '--manus-user', 'fixture'])
        self.assertEqual(resolved['manus_input_contract']['sides'], ['left'])

    def test_live_worker_binds_single_left_explicitly_and_keeps_dual_default(self):
        from tianji_teleop.producers.spark.live_runner import _make_hand_client
        for sides, expected in ((['left'], 'left'), (['right'], 'right'), (['left', 'right'], 'right')):
            with self.subTest(sides=sides):
                kwargs = _make_hand_client(ROOT, sides, client_factory=lambda **options: options)
                self.assertEqual(kwargs['single_hand_side'], expected)
                self.assertTrue(kwargs['startup_handshake'])

    def run_cli(self, *args):
        script = ROOT / 'scripts/vr_manus_live.py'
        self.assertTrue(script.is_file(), 'missing managed live entry implementation')
        return subprocess.run([sys.executable, str(script), '--check', *args],
                              capture_output=True, text=True, timeout=10)

    def test_arms_only_preflight_has_no_router_or_device_side_effect(self):
        result = self.run_cli('--disable-hands')
        self.assertEqual(result.returncode, 0, result.stderr)
        config = json.loads(result.stdout)
        self.assertEqual(config['config']['receivers'], ['tjvr'])
        self.assertEqual(config['tjvr_bind'], ['127.0.0.1', 15000])
        self.assertFalse(config['real_time_qualified'])
        self.assertEqual(config['tjvr_stream_contract'], dict(version=1,
            initial_state='reset', max_position_jump_m=.15, max_orientation_jump_rad=.6))
        self.assertEqual(config['manus_input_contract'], dict(
            version=1, sides=[], right_glove=None, left_glove=None,
            callback_order='right_then_left', callback_trigger='each_accepted_pose'))
        assets = config['asset_sha256']
        for relative in ('src/tianji_teleop/config/coordinator/arm_v131.yaml',
                         'src/tianji_teleop/config/robot/arm.yaml',
                         'src/tianji_teleop/assets/tianji_wuji2/meshes/Link_Base.STL',
                         'tools/spark_native/pixi.lock'):
            self.assertIn(relative, assets)

    def test_enabled_hands_need_explicit_rawviz_and_operator_calibration(self):
        result = self.run_cli()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('manus', result.stderr.lower())

    def test_blank_glove_bindings_are_rejected_before_asset_preflight(self):
        for flag in ('--left-glove', '--right-glove'):
            for value in ('', ' ', '\t'):
                for mode in ((), ('--disable-hands',)):
                    with self.subTest(flag=flag, value=value, mode=mode):
                        result = self.run_cli(*mode, flag, value)
                        self.assertEqual(result.returncode, 2, result.stderr)
                        self.assertIn('glove binding must be nonblank', result.stderr)

    def test_preflight_rejects_unmigrated_smoothing_and_invalid_port(self):
        for options in (('--joint-trajectory', 'ruckig'), ('--tjvr-port', '-1')):
            result = self.run_cli('--disable-hands', *options)
            self.assertNotEqual(result.returncode, 0)

    def test_record_preflight_is_readonly_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'new.h5'
            result = self.run_cli('--disable-hands', '--record', str(path))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse(path.exists())
            self.assertEqual(json.loads(result.stdout)['record_path'], str(path))
            path.touch()
            result = self.run_cli('--disable-hands', '--record', str(path))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('overwrite', result.stderr)

    def test_new_overlay_has_its_own_opt_in_flag(self):
        result = self.run_cli('--disable-hands', '--viewer', '--spark-overlay')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(json.loads(result.stdout)['spark_overlay'])

    def test_unimplemented_operator_binding_in_yaml_is_not_silently_ignored(self):
        spec = importlib.util.spec_from_file_location('dual_preflight_test', ROOT / 'scripts/vr_manus_live.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        from tianji_teleop.config_loader import load_yaml
        value = load_yaml(ROOT / 'src/tianji_teleop/config/sessions/vr_manus_sim.yaml')
        value['operator_input'] = 'controller'
        errors = io.StringIO()
        with patch('tianji_teleop.config_loader.load_yaml', return_value=value), redirect_stderr(errors):
            with self.assertRaises(SystemExit):
                module.resolve(['--disable-hands'])
        self.assertIn('operator', errors.getvalue())

    @unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional full process live session')
    def test_live_headless_without_input_never_authorizes(self):
        import zenoh
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            endpoint = 'tcp/127.0.0.1:' + str(probe.getsockname()[1])
        router = subprocess.Popen([str(ROOT / 'vendor/zenoh-router/zenohd'), '-l', endpoint,
                                   '--no-multicast-scouting'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            deadline = time.monotonic() + 3
            port = int(endpoint.rsplit(':', 1)[1])
            while True:
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=.1):
                        break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.02)
            config = zenoh.Config.from_json5(json.dumps(dict(mode='client',
                connect=dict(endpoints=[endpoint]), scouting=dict(multicast=dict(enabled=False)))))
            session = zenoh.open(config)
            try:
                zid = str(next(iter(session.info.routers_zid())))
                result = subprocess.run([sys.executable, str(ROOT / 'scripts/vr_manus_live.py'),
                    '--disable-hands', '--headless', '--tjvr-port', '0', '--duration-s', '.15'],
                    env=dict(os.environ, TIANJI_DUAL_MANAGED='1', TIANJI_RUN_ID='test-run',
                             TIANJI_DUAL_INSTANCE_ID='test-owner', TIANJI_ROUTER_ZID=zid,
                             TIANJI_ROUTER_ENDPOINT=endpoint), capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr)
                rows = [json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
                report = rows[-1]
                self.assertEqual(report['kind'], 'dual_live_complete')
                self.assertEqual(report['native_ticks'], 0)
                self.assertEqual(report['state'], 'idle')
                # Real UDP receive thread, real native budgets and explicit
                # keyboard request through a pipe. No robot/SDK hardware.
                from tests.test_reference_tjvr_receiver import packet
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                    probe.bind(('127.0.0.1', 0))
                    input_port = probe.getsockname()[1]
                latest = {}
                def receive(sample):
                    value = json.loads(bytes(sample.payload))
                    if value.get('component_role') == 'producer_arm':
                        latest['producer'] = value
                subscriber = session.declare_subscriber('tianji/producer/status', receive)
                state_subscriber = session.declare_subscriber('tianji/session/state',
                    lambda sample: latest.update(state=json.loads(bytes(sample.payload))))
                capture_directory = tempfile.TemporaryDirectory()
                self.addCleanup(capture_directory.cleanup)
                capture_path = Path(capture_directory.name) / 'live.h5'
                child = subprocess.Popen([sys.executable, str(ROOT / 'scripts/vr_manus_live.py'),
                    '--disable-hands', '--headless', '--tjvr-port', str(input_port), '--duration-s', '8.0',
                    '--record', str(capture_path)],
                    env=dict(os.environ, TIANJI_DUAL_MANAGED='1', TIANJI_RUN_ID='test-active',
                             TIANJI_DUAL_INSTANCE_ID='test-active-owner', TIANJI_ROUTER_ZID=zid,
                             TIANJI_ROUTER_ENDPOINT=endpoint), stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                stop_feed = Event()
                def feed():
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                        sequence = 0
                        while not stop_feed.is_set():
                            sequence += 1
                            sender.sendto(packet(sequence), ('127.0.0.1', input_port))
                            stop_feed.wait(.01)
                thread = Thread(target=feed)
                thread.start()
                try:
                    deadline = time.monotonic() + 8
                    while not latest.get('producer', {}).get('ready') and child.poll() is None and time.monotonic() < deadline:
                        time.sleep(.005)
                    self.assertTrue(latest.get('producer', {}).get('ready'))
                    child.stdin.write('s')
                    child.stdin.flush()
                    def wait_until(predicate):
                        deadline = time.monotonic() + 5
                        while not predicate() and child.poll() is None and time.monotonic() < deadline:
                            time.sleep(.005)
                        self.assertTrue(predicate(), repr(latest))
                    wait_until(lambda: latest.get('state', {}).get('state') == 'teleop')
                    wait_until(lambda: latest.get('producer', {}).get('diagnostics', {}).get('native_ticks', 0) > 0)
                    child.stdin.write('h')
                    child.stdin.flush()
                    wait_until(lambda: latest.get('state', {}).get('state') == 'idle')
                    child.stdin.write('r')
                    child.stdin.flush()
                    wait_until(lambda: 'fresh input and start required' in latest.get('state', {}).get('reason', ''))
                    wait_until(lambda: latest.get('producer', {}).get('ready', False))
                    self.assertEqual(latest['state']['state'], 'idle')
                    child.stdin.write('s')
                    child.stdin.flush()
                    output, errors = child.communicate(timeout=10)
                    self.assertEqual(child.returncode, 0, output + errors)
                    report = [json.loads(line) for line in output.splitlines() if line.startswith('{')][-1]
                    self.assertGreater(report['native_ticks'], 0)
                    self.assertEqual(report['state'], 'teleop')
                    self.assertTrue(any(row.get('action') == 'rearm' and row.get('accepted')
                                        for row in [json.loads(line) for line in output.splitlines() if line.startswith('{')]))
                    from tianji_teleop.recording.session_h5 import SessionH5Reader
                    self.assertTrue(capture_path.is_file())
                    with SessionH5Reader(capture_path) as reader:
                        self.assertGreater(len(reader.read_raw_reference_tjvr()), 0)
                        self.assertGreater(len(reader.read_arm_command('left')), 0)
                        audit = reader.read_dual_audit()
                        self.assertEqual({r['payload']['execution_epoch'] for r in audit
                                          if r['kind'] == 'native_cycle'}, {1, 2})
                        self.assertTrue(any(r['kind'] == 'operator_result' and
                                            r['payload']['action'] == 'rearm' for r in audit))
                        rearm = next(r['payload'] for r in audit if r['kind'] == 'operator_result'
                                     and r['payload']['action'] == 'rearm')
                        self.assertEqual(len(rearm['reset_ack']['position_rad']), 14)
                        self.assertTrue(any(r['kind'] == 'native_cycle' and r['payload'].get('native_attempt')
                                            and r['payload']['native_attempt']['sample'] is not None for r in audit))
                    from tianji_teleop.recording.tjvr_check import check_tjvr_recording
                    reconstruction = check_tjvr_recording(capture_path)
                    self.assertTrue(reconstruction['passed'], reconstruction)
                    self.assertGreater(reconstruction['matched_decisions'], 10)
                    self.assertEqual(reconstruction['native_input_check'], 'checked')
                    self.assertGreater(reconstruction['matched_native_inputs'], 0)
                    from tianji_teleop.recording.spark_reset_check import check_spark_reset_recording
                    reset_reconstruction = check_spark_reset_recording(capture_path)
                    self.assertTrue(reset_reconstruction['passed'], reset_reconstruction)
                    self.assertEqual(reset_reconstruction['reset_audit']['validated_reset_acks'], 1)
                finally:
                    stop_feed.set()
                    thread.join(timeout=2)
                    subscriber.undeclare()
                    state_subscriber.undeclare()
                    if child.poll() is None:
                        child.terminate()
                        child.wait(timeout=5)
                    for stream in (child.stdin, child.stdout, child.stderr):
                        stream.close()
                with tempfile.TemporaryDirectory() as directory:
                    managed = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                        '--profile', 'vr_manus_sim', '--disable-hands', '--headless',
                        '--tjvr-port', '0', '--duration-s', '.15', '--record', str(Path(directory) / 'managed.h5')],
                        env=dict(os.environ, TIANJI_ROUTER_ENDPOINT=endpoint,
                                 TIANJI_TELEOP_RUNTIME_DIR=directory),
                        capture_output=True, text=True, timeout=20)
                    self.assertEqual(managed.returncode, 0, managed.stdout + managed.stderr)
                    self.assertIn('dual_live_complete', managed.stdout)
                    self.assertFalse((Path(directory) / 'guards/vr_manus_sim').exists())
                    with SessionH5Reader(Path(directory) / 'managed.h5') as reader:
                        self.assertEqual(reader.read_raw_reference_tjvr(), [])
            finally:
                session.close()
        finally:
            router.terminate()
            router.wait(timeout=5)
