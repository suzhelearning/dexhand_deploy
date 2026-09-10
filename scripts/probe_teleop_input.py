#!/usr/bin/env python3
"""Receive-only device probe. Owns input temporarily; stop teleop first.

No router, solver, actuator, ADB forwarding, source fallback or auto-start.
Exit 0: required input updates observed; 1: missing/stale/error; 2: bad options.
"""
import argparse
import json
import math
import os
from pathlib import Path
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))


def _load_xr_probe_config(path, arm_input=None):
    """Load only the XR binding portion needed by the receive-only probe."""
    import yaml
    from tianji_teleop.hand_tracking.xr_input import XrBindingConfig

    with path.open('r', encoding='utf-8') as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, dict) or not isinstance(raw.get('xr'), dict):
        raise ValueError('XR probe config must contain an xr mapping')
    xr = raw['xr']
    selected_arm_input = arm_input or xr.get('arm_input', 'xr_tracker')
    binding = XrBindingConfig(
        arm_input=selected_arm_input,
        tracker_serials=xr.get('tracker_serials'),
        controller_sides=xr.get('controller_sides'),
        elbow_tracker_serials=xr.get('elbow_tracker_serials'),
    )
    host = xr.get('host', '127.0.0.1')
    port = xr.get('port', 60061)
    poll_hz = xr.get('poll_hz', 90.0)
    if not isinstance(host, str) or not host.strip():
        raise ValueError('xr.host must be a non-empty string')
    if isinstance(port, bool) or not isinstance(port, int) or not 0 < port <= 65535:
        raise ValueError('xr.port must be an integer in 1..65535')
    if not isinstance(poll_hz, (int, float)) or not math.isfinite(float(poll_hz)) or poll_hz <= 0:
        raise ValueError('xr.poll_hz must be finite and positive')
    required_serials = tuple(dict.fromkeys(
        tuple(binding.tracker_serials.values()) + tuple(binding.elbow_tracker_serials.values())
    ))
    return binding, host.strip(), port, float(poll_hz), required_serials


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('pico', 'tjvr', 'manus', 'xr'), required=True)
    parser.add_argument('--host')
    parser.add_argument('--port', type=int)
    parser.add_argument('--duration-s', type=float, default=10.)
    parser.add_argument('--minimum-frames', type=int, default=10)
    parser.add_argument('--manus-rawviz', type=Path)
    parser.add_argument('--manus-user')
    parser.add_argument('--manus-library-dir', type=Path)
    parser.add_argument('--xr-config', type=Path)
    parser.add_argument('--xr-sdk-pythonpath', type=Path)
    parser.add_argument('--arm-input', choices=('xr_controller', 'xr_tracker'))
    parser.add_argument('--minimum-trackers', type=int)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration_s) or not 0 < args.duration_s <= 60:
        parser.error('duration must be finite and in (0, 60] seconds')
    if (args.minimum_frames < 1 or
            (args.host is not None and not args.host.strip()) or
            (args.port is not None and not 0 < args.port <= 65535)):
        parser.error('positive frame count, explicit host and port 1..65535 required')
    if args.mode != 'manus' and any((args.manus_rawviz, args.manus_user, args.manus_library_dir)):
        parser.error('Manus options require --mode manus')
    if args.mode != 'xr' and any((args.xr_config, args.xr_sdk_pythonpath, args.arm_input, args.minimum_trackers)):
        parser.error('XR options require --mode xr')
    if args.minimum_trackers is not None and args.minimum_trackers < 0:
        parser.error('--minimum-trackers must be non-negative')
    if args.mode == 'xr':
        if args.xr_sdk_pythonpath is not None and not args.xr_sdk_pythonpath.is_dir():
            parser.error('--xr-sdk-pythonpath must be an existing directory')
        if args.xr_config is not None and not args.xr_config.is_file():
            parser.error('--xr-config must be an existing file')
    from tianji_teleop.hand_tracking.device_probe import InputProbe
    probe = InputProbe()
    error = None
    start = time.monotonic()
    deadline = start + args.duration_s
    try:
        if args.mode == 'manus':
            if not args.manus_rawviz or not args.manus_user or not args.manus_user.isalnum():
                parser.error('Manus requires executable --manus-rawviz and alphanumeric --manus-user')
            rawviz = args.manus_rawviz.resolve(strict=True)
            if not os.access(rawviz, os.X_OK):
                raise ValueError('Manus rawviz is not executable')
            for side in ('Left', 'Right'):
                if not (rawviz.parent / 'calibration' / (args.manus_user + side + 'MetaglovePro.mcal')).is_file():
                    raise ValueError('Manus user calibration missing for ' + side)
            from tianji_teleop.hand_tracking.manus_environment import manus_environment
            from tianji_teleop.hand_tracking.reference_manus_process import ReferenceManusProcess
            env, _ = manus_environment(rawviz, library_dir=args.manus_library_dir)
            source = ReferenceManusProcess(command=[str(rawviz), '--user', args.manus_user],
                receiver_instance_id='device-probe-manus', env=env)
            try:
                while time.monotonic() < deadline:
                    if source.failure:
                        raise RuntimeError(source.failure)
                    row = source.try_read()
                    if row is None:
                        time.sleep(.001)
                        continue
                    for side, sequence in row.source_sequences.items():
                        probe.observe(side, sequence, row.received_timestamp_ns)
            finally:
                source.close()
        elif args.mode == 'xr':
            from tianji_teleop.hand_tracking.device_probe import XrInputProbe
            from tianji_teleop.hand_tracking.xr_input import (
                XRoboToolkitClient,
                XrRoboToolkitSource,
                validate_xr_sdk_module,
            )
            config_path = args.xr_config or (
                ROOT / 'src/tianji_teleop/config/sources/xr_manus_observation.yaml'
            )
            config_path = config_path.resolve(strict=True)
            binding, config_host, config_port, poll_hz, required_serials = _load_xr_probe_config(
                config_path, args.arm_input
            )
            if args.minimum_trackers is None:
                minimum_trackers = len(required_serials)
            else:
                minimum_trackers = args.minimum_trackers
            if minimum_trackers > len(required_serials):
                raise ValueError('--minimum-trackers cannot exceed configured tracker bindings')
            probe = XrInputProbe(
                required_serials,
                minimum_trackers=minimum_trackers,
            )
            if args.xr_sdk_pythonpath is not None:
                os.environ['TIANJI_XR_SDK_PYTHONPATH'] = str(
                    args.xr_sdk_pythonpath.resolve(strict=True)
                )
            host = args.host or config_host
            port = args.port or config_port
            client = XRoboToolkitClient(host, port)
            missing = validate_xr_sdk_module(client._load_sdk())
            if missing:
                raise RuntimeError('XRoboToolkit SDK missing required API: ' + ', '.join(missing))
            source = XrRoboToolkitSource(
                client=client,
                binding=binding,
                receiver_instance_id='device-probe-xr',
            )
            try:
                if not source.initialize():
                    raise RuntimeError('XRoboToolkit SDK could not connect to PC-Service')
                period = 1.0 / poll_hz
                while time.monotonic() < deadline:
                    probe.observe(source.read_frame(), binding)
                    time.sleep(min(period, .02))
            finally:
                source.close()
        else:
            from tianji_teleop.hand_tracking.runtime import PicoPacketStream
            from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
            decoder = (PicoPacketStream(receiver_instance_id='device-probe-pico', connection_generation=0)
                if args.mode == 'pico' else ReferenceTjvrReceiver('device-probe-tjvr', .15, .6))
            host = args.host or '127.0.0.1'
            port = args.port or (10002 if args.mode == 'pico' else 15000)
            stream = (socket.create_connection((host, port), timeout=min(2., args.duration_s))
                if args.mode == 'pico' else socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            with stream:
                if args.mode == 'tjvr':
                    stream.bind((host, port))  # exclusive, no SO_REUSEADDR
                stream.settimeout(min(.05, args.duration_s))
                while time.monotonic() < deadline:
                    try:
                        data = stream.recv(65536)
                    except socket.timeout:
                        continue
                    now = time.monotonic_ns()
                    if args.mode == 'pico':
                        if not data:
                            raise RuntimeError('PICO TCP disconnected')
                        for frame in decoder.feed(data, received_timestamp_ns=now):
                            for side, hand in frame.hands.items():
                                if frame.head_valid and hand.valid and hand.wrist_valid and all(j.valid for j in hand.joints):
                                    probe.observe(side, frame.source_timestamp_ms, now)
                    else:
                        decoder.ingest(data, now)
                        received = decoder.try_read_latest()
                        if received is not None:
                            frame = received.observation.frame
                            if frame.protocol_version == 4 and frame.upper_limb_skeleton_valid and frame.upper_limb_rotations_valid:
                                for side in ('left', 'right'):
                                    probe.observe(side, frame.receiver_frame_sequence, now)
    except (OSError, ValueError, RuntimeError, TypeError, ImportError) as exc:
        error = str(exc)
    result = probe.report(time.monotonic_ns(), minimum_frames=args.minimum_frames)
    result.update(mode=args.mode, duration_s=time.monotonic() - start, error=error)
    if error:
        result['passed'] = False
    print(json.dumps(result, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
