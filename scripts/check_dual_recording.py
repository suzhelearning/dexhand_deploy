#!/usr/bin/env python3
"""Offline HDF5 raw/observation consistency check (no control replay).

Prints JSON. Exit 0: consistent; 1: differences; 2: invalid input/options.
PICO: raw26 -> canonical21/head-relative wrist. Manus: rawviz -> callbacks.
TJVR: decoded raw -> stream decisions and recorded native input associations.
XR: raw acquisition columns -> complete XrFrame payload and connection epochs.
Does not infer scheduling or execute recorded IK/control actions.
Optional hand-command reconstruction starts only isolated offline solvers.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--mode', choices=('pico', 'manus', 'tjvr', 'xr'), default='pico')
    parser.add_argument('--retarget-hand-commands', action='store_true',
                        help='verify assets, replay offline retarget at fixed 1e-5 rad tolerance')
    parser.add_argument('--check-native-resets', action='store_true',
                        help='TJVR only: passively check reset acknowledgements and execution epochs')
    parser.add_argument('--atol', type=float, default=1e-9,
                        help='PICO/Manus numerical tolerance; TJVR decisions and discrete metadata remain exact')
    args = parser.parse_args()
    if args.check_native_resets and args.mode != 'tjvr':
        parser.error('--check-native-resets requires --mode tjvr')
    if args.mode == 'tjvr' and args.retarget_hand_commands:
        parser.error('TJVR contains no hand fingers; use --mode manus for hand reconstruction')
    if args.mode == 'xr' and args.retarget_hand_commands:
        parser.error('XR recording check stops at raw acquisition; hand reconstruction uses --mode manus or pico')
    from tianji_teleop.recording.dual_check import check_pico_recording
    from tianji_teleop.recording.manus_check import check_manus_recording
    try:
        if args.mode == 'tjvr':
            if args.check_native_resets:
                from tianji_teleop.recording.spark_reset_check import check_spark_reset_recording
                result = check_spark_reset_recording(args.input)
            else:
                from tianji_teleop.recording.tjvr_check import check_tjvr_recording
                result = check_tjvr_recording(args.input)
        elif args.mode == 'xr':
            from tianji_teleop.recording.xr_check import check_xr_recording
            result = check_xr_recording(args.input, atol=args.atol)
        elif args.retarget_hand_commands:
            from tianji_teleop.recording.hand_command_check import check_manus_hand_commands
            from tianji_teleop.recording.pico_hand_command_check import check_pico_hand_commands
            check = check_pico_hand_commands if args.mode == 'pico' else check_manus_hand_commands
            result = check(args.input, root=ROOT, input_atol=args.atol)
        else:
            check = check_pico_recording if args.mode == 'pico' else check_manus_recording
            result = check(args.input, atol=args.atol)
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps(dict(passed=False, error=str(exc), operator_events_executed=0)))
        return 2
    print(json.dumps(result, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
