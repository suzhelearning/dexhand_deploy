#!/usr/bin/env python3
"""Read-only resolution for new session contracts; does not launch devices."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True,
                        choices=('pico2_hands_sim', 'vr_manus_sim', 'vr_manus_xr_sim'))
    parser.add_argument('--disable-hands', action='store_true')
    names = {'ik-backend': 'ik_backend', 'arm-pose-mapper': 'arm_pose_mapper',
             'arm-target-processor': 'arm_target_processor', 'joint-trajectory': 'joint_trajectory',
             'joint-limit-source': 'joint_limit_source', 'operator-input': 'operator_input',
             'arm-input': 'arm_input'}
    for flag, key in names.items():
        parser.add_argument('--' + flag, dest=key)
    parser.add_argument('--command-step-clipping', choices=('true', 'false'))
    args = parser.parse_args()
    from tianji_teleop.config_loader import component_path, load_yaml, resolve_dual_session_config
    value = load_yaml(component_path(f'sessions/{args.profile}.yaml'))
    for key in names.values():
        if getattr(args, key) is not None:
            value[key] = getattr(args, key)
    if args.command_step_clipping is not None:
        value['command_step_clipping'] = args.command_step_clipping == 'true'
    result = resolve_dual_session_config(value, disable_hands=args.disable_hands)
    available = args.profile in ('vr_manus_sim', 'pico2_hands_sim', 'vr_manus_xr_sim')
    reason = 'live binding implemented; runtime asset/calibration preflight still required'
    if args.profile == 'vr_manus_sim' and result['operator_input'] != 'keyboard':
        available = False
        reason = 'VR live operator binding is keyboard-only; controller binding is not implemented'
    if args.profile == 'pico2_hands_sim':
        from tianji_teleop.hand_tracking.session_config import validate_pico_runtime
        try:
            validate_pico_runtime(result)
        except ValueError as exc:
            available, reason = False, str(exc)
    if args.profile == 'vr_manus_xr_sim':
        reason = 'XRoboToolkit/Manus executable and calibration preflight still required'
    print(json.dumps(dict(profile=args.profile, config=result, runtime_available=available,
        reason=reason), allow_nan=False))


if __name__ == '__main__':
    main()
