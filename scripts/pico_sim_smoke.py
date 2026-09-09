"""Bounded, isolated synthetic PICO full-process smoke test (no hardware).

Run: pixi run python scripts/pico_sim_smoke.py
Logs and the session recording are retained in the printed temporary directory.
"""
import collections
import fcntl
import json
import math
import os
from pathlib import Path
import pty
import socket
import subprocess
import sys
import tempfile
import termios
import threading
import time

import yaml
import zenoh
import numpy as np
import h5py

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))
from tests.test_pico_mujoco_pipeline import packet


def main():
    arms_only = '--disable-hands' in sys.argv[1:]
    tracking_loss_test = '--tracking-loss-test' in sys.argv
    if tracking_loss_test and not arms_only:
        raise ValueError('tracking-loss-test requires --disable-hands')
    loss_mask = 0
    overrides = []
    if '--pico-overlay' in sys.argv:
        overrides.append('--pico-overlay')
    if '--ik-target-overlay' in sys.argv:
        overrides.append('--ik-target-overlay')
    for flag in ('--ik-backend', '--joint-trajectory', '--command-step-clipping', '--arm-target-processor', '--joint-limit-source', '--arm-pose-mapper'):
        if flag in sys.argv:
            overrides += [flag, sys.argv[sys.argv.index(flag) + 1]]
    output = Path(tempfile.mkdtemp(prefix='pico-sim-smoke-'))
    print(f'Artifacts: {output}', flush=True)
    processes = []
    stop = threading.Event()
    moving = threading.Event()
    latest = {}
    counts = collections.Counter()
    samples = collections.defaultdict(lambda: collections.deque(maxlen=500))
    lock = threading.Lock()
    session = None
    master = slave = None
    result = {'passed': False}
    listener = socket.socket()
    listener.bind(('127.0.0.1', 0))
    listener.listen(1)
    listener.settimeout(0.5)

    def feed():
        while not stop.is_set():
            try:
                connection, _ = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with connection:
                connection.settimeout(1)
                started = time.monotonic()
                while not stop.is_set():
                    value = (1 - math.cos((time.monotonic() - started) * 1.5)) / 2 if moving.is_set() else 0
                    try:
                        frame = bytearray(packet(0.015 * value, 0.01 * value))
                        frame[15] &= ~loss_mask  # PICO v1 validity flags; TCP stays live
                        connection.sendall(frame)
                    except OSError:
                        break
                    stop.wait(0.02)

    def receive(sample):
        try:
            value = json.loads(sample.payload.to_bytes())
        except (ValueError, UnicodeError):
            return
        key = str(sample.key_expr)
        with lock:
            latest[key] = value
            counts[key] += 1
            if '/state/' in key or '/command/' in key or '/proposal/' in key:
                samples[key].append(value)

    def wait_for(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError(f'process exited {process.returncode}: {process.args}')
            with lock:
                if predicate():
                    return
            time.sleep(0.1)
        raise TimeoutError(label)

    try:
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        endpoint = f'tcp/127.0.0.1:{port}'
        with (output / 'router.log').open('wb') as log:
            processes.append(subprocess.Popen([str(ROOT / 'vendor/zenoh-router/zenohd'), '-l', endpoint,
                                               '--no-multicast-scouting'], stdout=log, stderr=log))
        time.sleep(1)
        config = zenoh.Config()
        config.insert_json5('mode', '"client"')
        config.insert_json5('connect/endpoints', json.dumps([endpoint]))
        config.insert_json5('scouting/multicast/enabled', 'false')
        session = zenoh.open(config)
        subscriber = session.declare_subscriber('tianji/**', receive)
        observation = yaml.safe_load((ROOT / 'src/tianji_teleop/config/sources/hand_tracking_observation.yaml').read_text())
        observation['pico'].update(host='127.0.0.1', port=listener.getsockname()[1], auto_adb_forward=False)
        config_path = output / 'observation.yaml'
        config_path.write_text(yaml.safe_dump(observation))
        threading.Thread(target=feed, daemon=True).start()
        master, slave = pty.openpty()

        def terminal():
            os.setsid()
            fcntl.ioctl(0, termios.TIOCSCTTY, 0)

        env = dict(os.environ, TIANJI_ROUTER_ENDPOINT=endpoint,
                   TIANJI_TELEOP_RUNTIME_DIR=str(output / 'runtime'))
        launcher = subprocess.Popen(['bash', 'scripts/run_session.sh', '--profile', 'hand_tracking_sim',
                                     '--headless', '--observation-config', str(config_path),
                                     '--record', str(output / 'session.h5'),
                                     *(['--disable-hands'] if arms_only else []), *overrides], cwd=ROOT, env=env,
                                    stdin=slave, stdout=slave, stderr=slave, preexec_fn=terminal)
        processes.append(launcher)
        os.close(slave)
        slave = None

        def drain():
            with (output / 'launcher.log').open('wb') as log:
                while True:
                    try:
                        data = os.read(master, 65536)
                    except OSError:
                        return
                    if not data:
                        return
                    log.write(data)
                    log.flush()

        threading.Thread(target=drain, daemon=True).start()
        wait_for(lambda: latest.get('tianji/source/status', {}).get('ready') and
                 counts['tianji/observation/hand/right'] > 10, 90, 'source readiness')
        # Startup readiness describes coordinator discovery, not receipt of the
        # first observation in the newly started target process.
        time.sleep(1)
        if '--height-calibration-test' in sys.argv:
            os.write(master, b'c')
            wait_for(lambda: latest.get('tianji/source/status', {}).get('diagnostics', {}).get(
                'height_calibration', {}).get('state') == 'collecting', 3, 'calibration collecting')
            # Deliberately malformed input on THIS smoke's isolated router only.
            session.put('tianji/observation/arm_input/right', json.dumps({'bad': 'calibration regression'}))
            wait_for(lambda: latest.get('tianji/source/status', {}).get('diagnostics', {}).get(
                'height_calibration', {}).get('state') == 'failed', 3, 'calibration rejected malformed input')
            assert latest['tianji/source/status']['healthy'] is False
            os.write(master, b'c')
            wait_for(lambda: latest.get('tianji/source/status', {}).get('diagnostics', {}).get(
                'height_calibration', {}).get('state') == 'calibrated', 8, 'height calibration')
            assert latest['tianji/source/status']['healthy'] is True
            assert latest['tianji/source/status']['ready'] is True
            assert latest['tianji/source/status']['error'] is None
            result['height_calibration_retry_recovered'] = True
            assert latest['tianji/session/state']['state'] != 'teleop'
            result['height_calibration'] = latest['tianji/source/status']['diagnostics']['height_calibration']
        os.write(master, b's')
        wait_for(lambda: latest.get('tianji/session/state', {}).get('state') == 'teleop', 30, 'teleop authorization')
        print('Teleop authorized; exercising synthetic movement', flush=True)
        if '--ik-target-overlay' in overrides:
            wait_for(lambda: all(latest.get('tianji/executor/status', {}).get('diagnostics', {}).get(
                'ik_target_overlay', {}).get(side, {}).get('state') == 'live'
                for side in ('left', 'right')), 10, 'IK target overlay receiver')
            result['ik_target_overlay'] = latest['tianji/executor/status']['diagnostics']['ik_target_overlay']
        if '--joint-limit-source' in overrides and overrides[overrides.index('--joint-limit-source') + 1] == 'urdf':
            snapshots = list((output / 'runtime').glob('*-arm-limits.yaml'))
            assert len(snapshots) == 1, 'missing shared URDF arm config'
            limits = yaml.safe_load(snapshots[0].read_text())
            assert limits['upper_limits_rad'][5] == 1.0472
            # Confirm actual child processes inherited the SAME snapshot, not
            # merely that the launcher created a file nobody reads.
            consumers = {}
            for proc in Path('/proc').iterdir():
                if not proc.name.isdigit():
                    continue
                try:
                    environment = dict(entry.split(b'=', 1) for entry in
                        (proc / 'environ').read_bytes().split(b'\0') if b'=' in entry)
                    cmdline = (proc / 'cmdline').read_bytes().decode().replace('\0', ' ')
                except (OSError, UnicodeError):
                    continue
                if environment.get(b'TIANJI_ARM_CONFIG') != str(snapshots[0]).encode():
                    continue
                for component in ('arm_ik_producer', 'arm_command_coordinator', 'mujoco_executor'):
                    if component in cmdline:
                        consumers[component] = str(snapshots[0])
            assert len(consumers) == 3, f'limit source not propagated: {consumers}'
            result['joint_limit_consumers'] = consumers
        if '--pico-overlay' in overrides:
            wait_for(lambda: latest.get('tianji/executor/status', {}).get('diagnostics', {}).get(
                'pico_overlay', {}).get('state') == 'live', 10, 'raw PICO overlay receiver')
            result['pico_overlay'] = latest['tianji/executor/status']['diagnostics']['pico_overlay']
        with lock:
            result['processing_status'] = {
                'producer': latest.get('tianji/producer/status'),
                'coordinator': latest.get('tianji/coordinator/status'),
                'source': latest.get('tianji/source/status'),
            }
            if '--arm-pose-mapper' in overrides:
                expected_mapper = overrides[overrides.index('--arm-pose-mapper') + 1]
                assert latest['tianji/source/status']['diagnostics']['arm_pose_mapper'] == expected_mapper
            if '--ik-backend' in overrides:
                expected_backend = overrides[overrides.index('--ik-backend') + 1]
                assert latest['tianji/producer/status']['diagnostics']['backend'] == expected_backend
                if expected_backend == 'pico_ee_dexhand_qp':
                    assert latest['tianji/producer/status']['diagnostics']['algorithm'] == 'pico_ee_v131_velocity_qp'
                    diagnostics = latest['tianji/producer/status']['diagnostics']
                    assert diagnostics['cartesian_otg_enabled'] is True
                    assert diagnostics['adaptive_cartesian_gain_enabled'] is True
                    assert diagnostics['cartesian_linear_limit_m_s'] == 3.0
                    assert diagnostics['cartesian_angular_limit_rad_s'] == 12.0
                    assert diagnostics['joint_velocity_limit_rad_s'] == 4.0
                    assert diagnostics['model_state_only'] is True
                    assert diagnostics['kinematics'] == 'mujoco_world'
            if '--arm-target-processor' in overrides:
                assert latest['tianji/source/status']['diagnostics']['arm_target_processor'] == overrides[overrides.index('--arm-target-processor') + 1]
            if '--joint-trajectory' in overrides:
                assert latest['tianji/producer/status']['diagnostics']['joint_trajectory_processor'] == overrides[overrides.index('--joint-trajectory') + 1]
            if '--command-step-clipping' in overrides:
                assert latest['tianji/coordinator/status']['diagnostics']['command_step_clipping_enabled'] == (overrides[overrides.index('--command-step-clipping') + 1] == 'true')
        with lock:
            samples.clear()
        moving.set()
        time.sleep(6)
        with lock:
            result['motion_samples'] = {key: list(values) for key, values in samples.items()}
            motion = {}
            expected = [('tianji/state/arm', 14)]
            if not arms_only:
                expected += [('tianji/state/hand/left', 20), ('tianji/state/hand/right', 20)]
            else:
                assert not any(counts[key] for key in counts if '/target/hand/' in key or '/command/hand/' in key), 'unexpected hand control'
                assert latest['tianji/source/status']['diagnostics']['active_hand_sides'] == []
            for key, width in expected:
                values = np.array([value['position_rad'] for value in samples[key]])
                if values.ndim != 2 or values.shape[0] < 10 or values.shape[1] != width:
                    raise AssertionError(f'missing state samples: {key}')
                if not np.isfinite(values).all():
                    raise AssertionError(f'nonfinite state: {key}')
                ranges = np.ptp(values, axis=0)
                groups = [ranges[:7], ranges[7:]] if width == 14 else [ranges]
                motion[key] = [float(np.max(group)) for group in groups]
                if any(value < 0.001 for value in motion[key]):
                    raise AssertionError(f'no joint motion: {key}: {motion[key]}')
            result['joint_motion_rad'] = motion
            if '--command-step-clipping' in overrides and overrides[overrides.index('--command-step-clipping') + 1] == 'false':
                checked = 0
                for side in ('left', 'right'):
                    proposals = {v['sequence']: v for v in samples[f'tianji/proposal/arm/{side}']}
                    for command in samples[f'tianji/command/arm/{side}']:
                        proposal = proposals.get(command.get('proposal_sequence'))
                        if command.get('mode') == 'teleop' and proposal is not None:
                            assert command['position_rad'] == proposal['position_rad'], 'coordinator changed direct proposal'
                            checked += 1
                assert checked > 10, 'insufficient matched direct commands'
                result['unchanged_direct_commands'] = checked
        if tracking_loss_test:
            for mask, sides in ((2, ('left',)), (6, ('left', 'right'))):
                loss_mask = mask
                wait_for(lambda: set(latest.get('tianji/source/status', {}).get('diagnostics', {}).get(
                    'tracking_hold_sides', [])) == set(sides), 5, 'tracking loss source state')
                wait_for(lambda: all(latest.get(f'tianji/proposal/arm/{side}', {}).get('diagnostics', {}).get(
                    'tracking_hold') is True for side in sides), 5, 'native tracking hold')
                time.sleep(.2)
                boundary = time.monotonic_ns()
                time.sleep(1.0)
                with lock:
                    assert latest['tianji/session/state']['state'] == 'teleop', 'tracking loss exited teleop'
                    for side in sides:
                        commands = [v['position_rad'] for v in samples[f'tianji/command/arm/{side}']
                                    if v['timestamp_ns'] >= boundary]
                        assert len(commands) > 20, 'missing hold heartbeat commands'
                        assert np.max(np.ptp(np.array(commands), axis=0)) == 0, 'arm drifted during tracking loss'
                    if len(sides) == 1:
                        commands = [v['position_rad'] for v in samples['tianji/command/arm/right']
                                    if v['timestamp_ns'] >= boundary]
                        assert np.max(np.ptp(np.array(commands), axis=0)) > 1e-5, 'valid arm stopped with lost peer'
            frozen = {side: latest[f'tianji/command/arm/{side}']['position_rad'] for side in ('left', 'right')}
            loss_mask = 0
            wait_for(lambda: latest['tianji/source/status']['diagnostics'].get('tracking_hold_sides') == [],
                     5, 'tracking recovery source state')
            wait_for(lambda: all(np.max(np.abs(np.array(latest[f'tianji/command/arm/{side}']['position_rad'])-
                frozen[side])) > 1e-4 for side in ('left', 'right')), 5, 'automatic original mapping recovery')
            assert latest['tianji/session/state']['state'] == 'teleop'
            result['tracking_loss_hold_recovery'] = True
        stop.set()
        wait_for(lambda: latest.get('tianji/session/state', {}).get('state') != 'teleop', 10, 'disconnect response')
        time.sleep(1)
        with lock:
            before = {key: value for key, value in counts.items() if '/target/' in key}
        time.sleep(1)
        with lock:
            after = {key: value for key, value in counts.items() if '/target/' in key}
        if not before or before != after:
            raise AssertionError('targets missing or still published after disconnect')
        wait_for(lambda: latest.get('tianji/session/state', {}).get('state') == 'idle', 30, 'return to idle')
        result['passed'] = True
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        stop.set()
        listener.close()
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=40)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        if session is not None:
            session.close()
        for fd in (master, slave):
            if fd is not None:
                os.close(fd)
        result.update(counts=dict(counts), latest=latest)
        if result['passed']:
            try:
                with h5py.File(output / 'session.h5', 'r') as recording:
                    raw = recording['raw/pico_hand_tracking']
                    frames = len(raw['time_ns'])
                    assert frames > 10, 'missing raw frames'
                    for side in ('left', 'right'):
                        assert raw[f'hands/{side}/joint_poses'].shape == (frames, 26, 7)
                    assert len(raw['raw_packet']) == frames
                    result['raw_pico_frames'] = frames
            except Exception as error:
                result.update(passed=False, error=f'HDF5 verification: {error}')
        (output / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({key: result[key] for key in ('passed', 'error') if key in result}), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
