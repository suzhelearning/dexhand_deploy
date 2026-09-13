import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class DualSessionLauncherTest(unittest.TestCase):
    def test_xr_sdk_pythonpath_is_scoped_to_xr_observation_process(self):
        script = (ROOT / 'scripts/run_session.sh').read_text()
        base_start = script.index('base_env=(')
        base_end = script.index('\nif [[ "${required_capability}"', base_start)
        self.assertNotIn('TIANJI_XR_SDK_PYTHONPATH=', script[base_start:base_end])

        observation_start = script.index('elif [[ "${xr_manus_simulation}" == true ]]; then')
        observation_end = script.index('\nlaunch_arm_executor()', observation_start)
        observation_block = script[observation_start:observation_end]
        self.assertIn(
            '"TIANJI_XR_SDK_PYTHONPATH=${xr_sdk_pythonpath}"',
            observation_block,
        )

    def test_xr_sdk_library_path_is_scoped_to_xr_observation_process(self):
        script = (ROOT / 'scripts/run_session.sh').read_text()
        base_start = script.index('base_env=(')
        base_end = script.index('\nif [[ "${required_capability}"', base_start)
        self.assertNotIn('TIANJI_XR_SDK_LIBRARY_DIR=', script[base_start:base_end])
        observation_start = script.index('elif [[ "${xr_manus_simulation}" == true ]]; then')
        observation_end = script.index('\nlaunch_arm_executor()', observation_start)
        observation_block = script[observation_start:observation_end]
        self.assertIn(
            '"TIANJI_XR_SDK_LIBRARY_DIR=${xr_sdk_library_dir}"',
            observation_block,
        )
        self.assertIn('"LD_LIBRARY_PATH=${xr_sdk_ld_library_path}"', observation_block)
        self.assertIn('TIANJI_XR_SDK_LIBRARY_DIR=', script[script.index('check_xr_sdk.py'):])

    def test_xr_qp_override_keeps_controller_only_conditioner_by_default(self):
        script = (ROOT / 'scripts/run_session.sh').read_text()
        start = script.index('if [[ "${ik_backend_override}" == pico_ee_dexhand_qp ]]; then')
        end = script.index('\nfi', start)
        block = script[start:end]
        self.assertIn('if [[ "${profile}" != "vr_manus_xr_sim" ]]; then', block)
        self.assertIn('target_processor_override="${target_processor_override:-passthrough}"', block)

    def test_xr_sdk_api_preflight_runs_before_live_guard(self):
        script = (ROOT / 'scripts/run_session.sh').read_text()
        preflight = script.index('${SCRIPT_DIR}/check_xr_sdk.py')
        guard = script.index('acquire_teleop_guard')
        self.assertLess(preflight, guard)
        preflight_block_start = script.rfind(
            'if [[ "${xr_manus_simulation}" == true ]]; then', 0, preflight
        )
        self.assertIn('xr_manus_simulation', script[preflight_block_start:preflight])

    def test_xr_sdk_preflight_uses_selected_arm_input_contract(self):
        script = (ROOT / 'scripts/run_session.sh').read_text()
        preflight = script.index('${SCRIPT_DIR}/check_xr_sdk.py')
        self.assertIn(
            '--arm-input "${arm_input_override}"',
            script[preflight:preflight + 180],
        )

    def test_xr_manus_profile_resolves_controller_input_without_device_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'vr_manus_xr_sim', '--arm-input', 'xr_controller',
                '--operator-input', 'controller', '--resolve-only'],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            value = json.loads(result.stdout)
            self.assertTrue(value['runtime_available'])
            self.assertEqual(value['config']['input_mode'], 'vr_manus')
            self.assertEqual(value['config']['arm_input'], 'xr_controller')
            self.assertEqual(value['config']['operator_input'], 'controller')
            self.assertFalse(runtime.exists())

    def test_xr_manus_live_requires_external_assets_before_router_or_processes(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'vr_manus_xr_sim', '--headless'],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertIn('rawviz', result.stderr)
            self.assertFalse(runtime.exists())

    def test_xr_manus_disable_hands_does_not_require_manus_assets(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'vr_manus_xr_sim', '--disable-hands', '--headless'],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime),
                         TIANJI_ROUTER_ENDPOINT='tcp/127.0.0.1:1'),
                capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertNotIn('rawviz', result.stderr)
            self.assertFalse(runtime.exists())

    def test_xr_sdk_pythonpath_is_a_runtime_only_option(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'vr_manus_xr_sim', '--xr-sdk-pythonpath', '/portable/xr-sdk',
                '--resolve-only'], env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2)
            self.assertIn('resolve-only', result.stderr)
            self.assertFalse(runtime.exists())

    def test_xr_overlay_is_rejected_for_the_pico2_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(
                ['bash', str(ROOT / 'scripts/run_session.sh'),
                 '--profile', 'pico2_hands_sim', '--disable-hands', '--xr-overlay'],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10,
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn('vr_manus_xr_sim', result.stderr)
            self.assertFalse(runtime.exists())

    def test_unimplemented_vr_controller_binding_is_not_reported_runtime_available(self):
        result = subprocess.run([sys.executable, str(ROOT / 'scripts/resolve_dual_session.py'),
            '--profile', 'vr_manus_sim', '--operator-input', 'controller'],
            capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        value = json.loads(result.stdout)
        self.assertEqual(value['config']['operator_input'], 'controller')
        self.assertFalse(value['runtime_available'])
        self.assertIn('keyboard', value['reason'])

    def test_pico_gesture_start_is_explicit_and_resolves_without_resources(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'pico2_hands_sim', '--operator-input', 'gesture', '--resolve-only'],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            value = json.loads(result.stdout)
            self.assertEqual(value['config']['operator_input'], 'gesture')
            self.assertTrue(value['runtime_available'])
            self.assertFalse(runtime.exists())

    def test_pico_resolve_reports_new_runtime_without_starting_it(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'pico2_hands_sim', '--resolve-only'],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            resolved = json.loads(result.stdout)
            self.assertTrue(resolved['runtime_available'])
            self.assertEqual(resolved['config']['ik_backend'], 'pico_ee_dexhand_qp')
            self.assertEqual(resolved['config']['hand_retarget_backend'], 'official_wuji_hand2')
            self.assertFalse(runtime.exists())

    def test_embedded_pico_profile_resolves_without_starting_processes(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(
                [
                    'bash', str(ROOT / 'scripts/run_session.sh'),
                    '--profile', 'pico_vr_manus_sim', '--disable-hands', '--resolve-only',
                ],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            value = json.loads(result.stdout)
            self.assertTrue(value['runtime_available'])
            self.assertEqual(value['config']['input_mode'], 'vr_manus')
            self.assertEqual(value['config']['arm_input'], 'tjvr_corrected_palm')
            self.assertEqual(value['config']['receivers'], ['tjvr'])
            self.assertEqual(value['config']['active_hand_sides'], [])
            self.assertFalse(runtime.exists())

            pico = subprocess.run(
                [
                    'bash', str(ROOT / 'scripts/run_session.sh'),
                    '--profile', 'pico2_hands_sim', '--resolve-only',
                ],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10,
            )
            self.assertEqual(pico.returncode, 0, pico.stderr)
            pico_value = json.loads(pico.stdout)
            self.assertEqual(pico_value['config']['receivers'], ['pico2'])
            self.assertNotIn('embedded', pico_value.get('reason', '').lower())

    def test_pico_runtime_rejects_real_before_locks(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'pico2_hands_sim', '--confirm-real'],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2)
            self.assertIn('simulation', result.stderr)
            self.assertFalse(runtime.exists())

    def test_resolve_only_does_not_create_runtime_or_start_receivers(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            environment = dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime))
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'vr_manus_sim', '--disable-hands', '--resolve-only'],
                env=environment, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            resolved = json.loads(result.stdout)
            self.assertEqual(resolved['config']['receivers'], ['tjvr'])
            self.assertTrue(resolved['runtime_available'])
            self.assertFalse(runtime.exists())

    def test_missing_manus_calibration_is_rejected_before_router_or_processes(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'vr_manus_sim'], env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2)
            self.assertIn('live', result.stderr)
            self.assertFalse(runtime.exists())

    def test_live_vr_route_validates_runtime_options_before_creating_guard(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'vr_manus_sim', '--disable-hands', '--spark-overlay', '--tjvr-port', '-1'],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2)
            self.assertIn('0..65535', result.stderr)
            self.assertFalse(runtime.exists())

    def test_record_path_reaches_preflight_before_runtime_guard(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'existing.h5'
            path.touch()
            runtime = Path(directory) / 'runtime'
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'),
                '--profile', 'vr_manus_sim', '--disable-hands', '--record', str(path)],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=str(runtime)),
                capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2)
            self.assertIn('overwrite', result.stderr)
            self.assertFalse(runtime.exists())
