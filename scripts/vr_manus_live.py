#!/usr/bin/env python3
"""VR+Manus live session preflight. Runtime is entered by the managed launcher."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))


def _portable_asset_key(path, *, root, external_keys):
    path = Path(path).resolve()
    try:
        return path.relative_to(root.resolve()).as_posix()
    except ValueError:
        try:
            return external_keys[path]
        except KeyError as exc:
            raise ValueError(f'no portable asset key for external path: {path.name}') from exc


def resolve(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('vr_manus_sim',), default='vr_manus_sim')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--disable-hands', action='store_true')
    display = parser.add_mutually_exclusive_group()
    display.add_argument('--viewer', action='store_true')
    display.add_argument('--headless', action='store_true')
    parser.add_argument('--tjvr-bind', default='127.0.0.1')
    parser.add_argument('--tjvr-port', type=int, default=15000)
    parser.add_argument('--duration-s', type=float)
    parser.add_argument('--record', type=Path, help='new HDF5 path in an existing directory; refuses overwrite')
    parser.add_argument('--spark-overlay', action='store_true',
                        help='passive TJVR corrected skeleton, packet targets and actual SPARK IK targets')
    parser.add_argument('--manus-rawviz', type=Path)
    parser.add_argument('--manus-user')
    parser.add_argument('--manus-library-dir', type=Path,
                        help='Manus SDK lib directory; defaults to rawviz sibling ManusSDK/lib if present')
    parser.add_argument('--right-glove')
    parser.add_argument('--left-glove')
    overrides = {'ik-backend': 'ik_backend', 'arm-pose-mapper': 'arm_pose_mapper',
                 'arm-target-processor': 'arm_target_processor', 'joint-trajectory': 'joint_trajectory',
                 'joint-limit-source': 'joint_limit_source'}
    for flag, field in overrides.items():
        parser.add_argument('--' + flag, dest=field)
    parser.add_argument('--command-step-clipping', choices=('true', 'false'))
    args = parser.parse_args(argv)
    for binding in (args.right_glove, args.left_glove):
        if binding is not None and not binding.strip():
            parser.error('glove binding must be nonblank; omit the option for automatic side detection')
    if not args.tjvr_bind.strip() or not 0 <= args.tjvr_port <= 65535:
        parser.error('explicit TJVR bind host and port in 0..65535 required')
    if args.duration_s is not None and (not math.isfinite(args.duration_s) or args.duration_s <= 0):
        parser.error('duration must be finite and positive')
    if args.record is not None:
        if args.record.exists() or args.record.is_symlink():
            parser.error('refusing to overwrite existing recording')
        args.record = args.record.absolute()
        if not args.record.parent.is_dir():
            parser.error('recording parent directory must exist')
    from tianji_teleop.config_loader import component_path, load_yaml, resolve_dual_session_config
    value = load_yaml(component_path('sessions/vr_manus_sim.yaml'))
    for field in overrides.values():
        if getattr(args, field) is not None:
            value[field] = getattr(args, field)
    if args.command_step_clipping is not None:
        value['command_step_clipping'] = args.command_step_clipping == 'true'
    config = resolve_dual_session_config(value, disable_hands=args.disable_hands)
    if config['operator_input'] != 'keyboard':
        parser.error('live operator binding is keyboard-only; native TJVR bit8 is not a complete controller binding')
    paths = [ROOT / 'build/spark-native/spark_native_worker',
             ROOT / 'tools/spark_native/pixi.lock',
             ROOT / 'src/tianji_teleop/config/sessions/vr_manus_sim.yaml',
             ROOT / 'src/tianji_teleop/config/coordinator/arm_v131.yaml',
             ROOT / 'src/tianji_teleop/config/robot/arm.yaml',
             ROOT / 'src/tianji_teleop/config/producers/spark_reference.yaml',
             ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_wuji2.xml',
             ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_s_ccs_696_v4_local.urdf']
    from tianji_teleop.recording.model_assets import flat_mujoco_asset_files
    try:
        paths += flat_mujoco_asset_files(
            ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_wuji2.xml',
            asset_root=ROOT / 'src/tianji_teleop/assets')
    except ValueError as exc:
        parser.error(str(exc))
    if args.disable_hands:
        if any((args.manus_rawviz, args.manus_user, args.right_glove, args.left_glove, args.manus_library_dir)):
            parser.error('disabled hands cannot include Manus device options')
    else:
        if not args.manus_rawviz or not args.manus_user:
            parser.error('live Manus requires --manus-rawviz and explicit --manus-user calibration')
        if not args.manus_user.isalnum():
            parser.error('Manus calibration user must be alphanumeric')
        args.manus_rawviz = args.manus_rawviz.resolve(strict=True)
        if not os.access(args.manus_rawviz, os.X_OK):
            parser.error('Manus rawviz must be executable')
        from tianji_teleop.hand_tracking.manus_environment import manus_environment
        try:
            _, sdk_library = manus_environment(args.manus_rawviz, library_dir=args.manus_library_dir)
        except ValueError as exc:
            parser.error(str(exc))
        args.manus_library_dir = sdk_library.parent if sdk_library is not None else None
        if sdk_library is not None:
            paths.append(sdk_library)
        paths += [args.manus_rawviz,
                  ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
                  ROOT / 'scripts/wuji_hand_worker.py']
        official = ROOT / 'third_party/wuji_hand_retargeting'
        paths += [ROOT / 'tools/wuji_hand_native/pixi.lock', official / 'source_manifest.json']
        manifest = json.loads((official / 'source_manifest.json').read_text())
        for entry in manifest['files']:
            asset = (official / entry['destination']).resolve()
            if not asset.is_relative_to(official.resolve()):
                parser.error('official hand asset escapes its pinned directory')
            paths.append(asset)
        for side in ('Left', 'Right'):
            paths.append(args.manus_rawviz.parent / 'calibration' /
                         (args.manus_user + side + 'MetaglovePro.mcal'))
        if args.right_glove and args.right_glove == args.left_glove:
            parser.error('left and right glove binding must differ')
    external_keys = {}
    if not args.disable_hands:
        external_keys[args.manus_rawviz.resolve()] = 'external/manus/rawviz.out'
        if args.manus_library_dir is not None:
            external_keys[(args.manus_library_dir / 'libManusSDK_Integrated.so').resolve()] = \
                'external/manus/sdk/libManusSDK_Integrated.so'
        for side in ('Left', 'Right'):
            calibration = (args.manus_rawviz.parent / 'calibration' /
                           (args.manus_user + side + 'MetaglovePro.mcal')).resolve()
            external_keys[calibration] = f'external/manus/calibration/{calibration.name}'
    hashes = {}
    asset_paths = {}
    for path in paths:
        if not path.is_file():
            parser.error(f'missing runtime asset: {path}')
        try:
            key = _portable_asset_key(path, root=ROOT, external_keys=external_keys)
        except ValueError as exc:
            parser.error(str(exc))
        resolved_path = path.resolve()
        if key in asset_paths:
            if asset_paths[key] != resolved_path:
                parser.error(f'portable asset key collision: {key}')
            continue
        asset_paths[key] = resolved_path
        hashes[key] = hashlib.sha256(path.read_bytes()).hexdigest()
    return args, dict(profile=args.profile, config=config, tjvr_bind=[args.tjvr_bind, args.tjvr_port],
                      manus_sdk_library_dir='external/manus/sdk' if args.manus_library_dir else None,
                      manus_sdk_loading='explicit_child_path' if args.manus_library_dir else 'inherited_unverified',
                      tjvr_stream_contract=dict(version=1, initial_state='reset',
                          max_position_jump_m=.15, max_orientation_jump_rad=.6),
                      manus_input_contract=dict(version=1,
                          sides=[side for side in ('right', 'left') if side in config['active_hand_sides']],
                          right_glove=args.right_glove, left_glove=args.left_glove,
                          callback_order='right_then_left', callback_trigger='each_accepted_pose'),
                      asset_sha256=hashes, record_path=str(args.record) if args.record else None,
                      spark_overlay=args.spark_overlay,
                      real_time_qualified=False)


def main():
    args, resolved = resolve()
    if args.check:
        print(json.dumps(resolved, allow_nan=False))
        return 0
    from tianji_teleop.producers.spark.live_runner import run_live
    return run_live(ROOT, args, resolved)


if __name__ == '__main__':
    raise SystemExit(main())
