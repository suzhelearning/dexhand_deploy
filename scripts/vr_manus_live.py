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
    parser.add_argument('--coordinator-math', choices=('python', 'cpp'), default='python',
                        help='coordinator numeric checks and joint commands, not authority/state machine')
    parser.add_argument('--simulation-backend', choices=('python', 'cpp'), default='python',
                        help='joint application and MuJoCo forward implementation')
    parser.add_argument('--execution-guard', choices=('python', 'cpp'), default='python',
                        help='receipt supervision implementation; cpp requires build-native-control')
    parser.add_argument('--native-result-format', choices=('json', 'binary'), default='json',
                        help='native IK result transport; binary is opt-in, algorithms unchanged')
    parser.add_argument('--spark-resync-policy', choices=('reference', 'resume'), default='reference',
                        help='SPARK C++ same-epoch recovery; reference preserves original takeover')
    parser.add_argument('--scheduler-backend', choices=('python', 'cpp'), default='python',
                        help='fixed-rate application scheduler; cpp is an opt-in native gateway')
    parser.add_argument('--publication-backend', choices=('python', 'cpp'), default='python',
                        help='arm topic encoder/Zenoh publisher; cpp requires native scheduler')
    parser.add_argument('--recording-adapter', choices=('python', 'cpp'), default='python',
                        help='per-frame recording owner; cpp requires native scheduler and --record')
    parser.add_argument('--viewer-backend', choices=('python', 'cpp'), default='python',
                        help='window/overlay owner; cpp requires native scheduler and --viewer')
    parser.add_argument('--hand-worker-backend', choices=('python', 'cpp'), default=None,
                        help='Hand2 post-driver worker; default python, cpp is opt-in')
    parser.add_argument('--hand-scheduler-backend', choices=('python', 'cpp'), default=None,
                        help='Hand2 fixed-rate scheduler; default python, cpp is opt-in')
    parser.add_argument('--mapped-palm-common-x-reference', action='store_true',
                        help='requires X/Z calibration: share the smaller robot reference TCP X')
    parser.add_argument('--mapped-palm-xz-calibration', action='store_true',
                        help='full C++ mapped route: c aligns X/Z to J2=-90 degrees, other joints zero')
    parser.add_argument('--mapped-palm-height-calibration', action='store_true',
                        help='mapped backend only: c samples horizontal arms; Z only, then s starts')
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
    parser.add_argument('--manus-parser-backend',choices=('python','cpp'),default='python',
                        help='post-driver rawviz parser; default Python reference')
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
    hand_worker_backend = args.hand_worker_backend or os.environ.get(
        'TIANJI_HAND_WORKER_BACKEND', 'python')
    if hand_worker_backend not in ('python', 'cpp'):
        parser.error('hand worker backend must be python or cpp')
    if args.disable_hands and hand_worker_backend != 'python':
        parser.error('--hand-worker-backend requires enabled hands; remove --disable-hands')
    hand_scheduler_backend = args.hand_scheduler_backend or os.environ.get(
        'TIANJI_HAND_SCHEDULER_BACKEND', 'python')
    if hand_scheduler_backend not in ('python', 'cpp'):
        parser.error('hand scheduler backend must be python or cpp')
    if args.disable_hands and hand_scheduler_backend != 'python':
        parser.error('--hand-scheduler-backend requires enabled hands; remove --disable-hands')
    if hand_scheduler_backend == 'cpp' and hand_worker_backend != 'python':
        parser.error('--hand-scheduler-backend cpp cannot be combined with --hand-worker-backend cpp')
    args.hand_worker_backend = hand_worker_backend
    args.hand_scheduler_backend = hand_scheduler_backend
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
    if args.spark_resync_policy != 'reference' and (
            args.scheduler_backend != 'cpp' or
            config['ik_backend'] != 'spark_upper_qpoases_headroom_feedforward_velocity_qp'):
        parser.error('--spark-resync-policy resume requires C++ scheduler and SPARK IK')
    if args.manus_parser_backend == 'cpp' and args.disable_hands:
        parser.error('native Manus parser requires hands')
    if args.publication_backend == 'cpp' and args.scheduler_backend != 'cpp':
        parser.error('--publication-backend cpp requires --scheduler-backend cpp')
    if args.viewer_backend == 'cpp':
        if args.scheduler_backend != 'cpp': parser.error('--viewer-backend cpp requires --scheduler-backend cpp')
        if not args.viewer: parser.error('--viewer-backend cpp requires --viewer')
    if args.recording_adapter == 'cpp':
        if args.scheduler_backend != 'cpp':
            parser.error('--recording-adapter cpp requires --scheduler-backend cpp')
        if args.record is None:
            parser.error('--recording-adapter cpp requires --record')
        recorder_binary = ROOT / 'build/hdf5_recorder/tianji_hdf5_recorder'
        if not recorder_binary.is_file() or not os.access(recorder_binary, os.X_OK):
            parser.error('native recording requires pixi run build-hdf5-recorder')
    if args.scheduler_backend == 'cpp':
        if config['hands_enabled']:
            if (hand_scheduler_backend != 'cpp' or args.publication_backend != 'cpp' or
                    args.viewer_backend != 'cpp' or not args.viewer or
                    args.recording_adapter != 'cpp' or args.record is None or
                    config['hand_input'] != 'manus' or sorted(config['active_hand_sides']) != ['left','right']):
                parser.error('native joint route requires bilateral Manus, --hand-scheduler-backend cpp, '
                             '--publication-backend cpp, --viewer --viewer-backend cpp, '
                             '--recording-adapter cpp and --record')
        if any((args.execution_guard != 'python', args.simulation_backend != 'python',
                args.coordinator_math != 'python', args.native_result_format != 'json')):
            parser.error('--scheduler-backend cpp owns IK result transport, coordinator checks and MuJoCo; omit --execution-guard/--simulation-backend/--coordinator-math/--native-result-format')
        gateway = ROOT / 'build/control-native/tianji_native_session_gateway'
        if not gateway.is_file() or not os.access(gateway, os.X_OK):
            parser.error(f'missing executable native scheduler: {gateway}; run pixi run build-native-session-gateway')
    if args.mapped_palm_common_x_reference and not args.mapped_palm_xz_calibration:
        parser.error('--mapped-palm-common-x-reference requires --mapped-palm-xz-calibration')
    if args.mapped_palm_xz_calibration:
        if args.mapped_palm_height_calibration:
            parser.error('choose only one mapped-palm calibration mode')
        if (args.scheduler_backend != 'cpp' or args.publication_backend != 'cpp'
                or args.viewer_backend != 'cpp' or args.recording_adapter != 'cpp'
                or not args.viewer or not args.record):
            parser.error('X/Z calibration requires full C++ scheduler/publication/viewer/recording and --viewer --record')
    if (args.mapped_palm_height_calibration or args.mapped_palm_xz_calibration) and config['ik_backend'] != 'pico_ee_mapped_corrected_palm_velocity_qp':
        parser.error('--mapped-palm-height-calibration requires mapped-palm IK')
    if config['operator_input'] != 'keyboard':
        parser.error('live operator binding is keyboard-only; native TJVR bit8 is not a complete controller binding')
    from tianji_teleop.producers.spark.backend_assets import bilateral_assets
    selected = bilateral_assets(ROOT, config['ik_backend'])
    hand_worker = None
    hand_scheduler = None
    if args.manus_parser_backend == 'cpp':
        from tianji_teleop.hand_tracking.native_manus_parser import NativeManusProcessor
        try:
            check=NativeManusProcessor(lambda _:None,right_glove=args.right_glove,left_glove=args.left_glove)
            check.close()
        except (ValueError,RuntimeError,OSError,AttributeError) as exc:
            parser.error(str(exc))
    paths = [selected['worker'], selected['lock'],
             ROOT / 'src/tianji_teleop/config/sessions/vr_manus_sim.yaml',
             ROOT / 'src/tianji_teleop/config/coordinator/arm_v131.yaml',
             ROOT / 'src/tianji_teleop/config/robot/arm.yaml',
             selected['config'], selected['model'], selected['urdf']]
    if args.scheduler_backend == 'cpp':
        paths.append(ROOT / 'build/control-native/tianji_native_session_gateway')
    if args.manus_parser_backend == 'cpp':
        paths.extend([ROOT/'build/hand-native/libtianji_hand_manus.so',
                      ROOT/'native/hand/manus_input.cpp',
                      ROOT/'src/tianji_teleop/tianji_teleop/hand_tracking/native_manus_parser.py'])
    if 'manifest' in selected:
        paths.append(selected['manifest'])
    if args.coordinator_math == 'cpp':
        from tianji_teleop.coordination.native_command_math import extension_path, load_native
        try:
            load_native()
        except (RuntimeError, ImportError, OSError) as exc:
            parser.error(str(exc))
        paths.append(extension_path())
    if args.simulation_backend == 'cpp':
        from tianji_teleop.executors.mujoco.native_kernel import extension_path, load_native
        try:
            load_native()
        except (RuntimeError, ImportError, OSError) as exc:
            parser.error(str(exc))
        paths.append(extension_path())
    if args.execution_guard == 'cpp':
        from tianji_teleop.producers.spark.native_execution import extension_path, load_native
        try:
            load_native()
        except (RuntimeError, ImportError, OSError) as exc:
            parser.error(str(exc))
        paths.append(extension_path())
    from tianji_teleop.recording.model_assets import flat_mujoco_asset_files
    try:
        paths += flat_mujoco_asset_files(
            selected['model'],
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
        if hand_worker_backend == 'cpp':
            from tianji_teleop.producers.native_hand_worker import (
                launcher_path, worker_path, recording_metadata as worker_metadata)
            try:
                hand_worker = worker_metadata(dict(os.environ,
                    TIANJI_HAND_WORKER_BACKEND=hand_worker_backend))
            except (RuntimeError, ValueError) as exc:
                parser.error(str(exc))
            paths += [worker_path(), launcher_path(),
                      ROOT / 'build/hand-native/libtianji_hand_optimizer.so',
                      ROOT / 'native/hand/worker.cpp',
                      ROOT / 'native/hand/geometry.cpp',
                      ROOT / 'native/hand/lowpass.cpp',
                      ROOT / 'native/hand/optimizer.cpp']
        if hand_scheduler_backend == 'cpp':
            from tianji_teleop.producers.native_hand_scheduler import (
                recording_metadata as scheduler_metadata)
            try:
                hand_scheduler = scheduler_metadata(dict(os.environ,
                    TIANJI_HAND_SCHEDULER_BACKEND=hand_scheduler_backend))
            except (RuntimeError, ValueError) as exc:
                parser.error(str(exc))
            paths += [ROOT / 'build/hand-native/tianji_hand_native_scheduler',
                      ROOT / 'build/hand-native/libtianji_hand_optimizer.so',
                      ROOT / 'scripts/wuji_hand_native_scheduler_launcher.py',
                      ROOT / 'native/hand/scheduler.hpp',
                      ROOT / 'native/hand/scheduler_wire.hpp',
                      ROOT / 'native/hand/scheduler_main.cpp',
                      ROOT / 'native/hand/pipeline.hpp']
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
    stream_contract = dict(version=1, initial_state='reset', max_position_jump_m=.15,
                           max_orientation_jump_rad=.6)
    if 'manifest' in selected:
        from tianji_teleop.config_loader import load_yaml
        native_config = load_yaml(selected['config'])
        stream_contract.update(target_source='mapped_corrected_palm',
            max_position_jump_m=native_config['pico_teleop']['max_position_jump_m'],
            max_orientation_jump_rad=native_config['pico_teleop']['max_orientation_jump_rad'])
    hand_filter = None
    hand_geometry = None
    hand_optimizer = None
    if not args.disable_hands:
        from tianji_teleop.producers.native_hand_filter import recording_metadata
        hand_filter = recording_metadata(os.environ)
        from tianji_teleop.producers.native_hand_geometry import recording_metadata as geometry_metadata
        hand_geometry = geometry_metadata(os.environ)
        from tianji_teleop.producers.native_hand_optimizer import recording_metadata as optimizer_metadata
        hand_optimizer = optimizer_metadata(os.environ)
        if hand_worker_backend == 'cpp' and hand_worker is None:
            from tianji_teleop.producers.native_hand_worker import recording_metadata as worker_metadata
            hand_worker = worker_metadata(dict(os.environ,
                TIANJI_HAND_WORKER_BACKEND=hand_worker_backend))
    return args, dict(profile=args.profile, config=config, tjvr_bind=[args.tjvr_bind, args.tjvr_port],
                      **({'hand_filter': hand_filter} if hand_filter is not None else {}),
                      **({'hand_geometry': hand_geometry} if hand_geometry is not None else {}),
                      **({'hand_optimizer': hand_optimizer} if hand_optimizer is not None else {}),
                      **({'hand_worker': hand_worker} if hand_worker is not None else {}),
                      hand_worker_backend=hand_worker_backend,
                      **({'hand_scheduler': hand_scheduler} if hand_scheduler is not None else {}),
                      hand_scheduler_backend=hand_scheduler_backend,
                      manus_parser_backend=args.manus_parser_backend,
                      **({'native_manus_ingress': 'cpp_owned_stdout', 'native_hand_domain': 'bilateral'}
                         if args.scheduler_backend == 'cpp' and config['hands_enabled'] else {}),
                      native_result_format=args.native_result_format,
                      scheduler_backend=args.scheduler_backend,
                      spark_resync_policy=args.spark_resync_policy,
                      publication_backend=args.publication_backend,
                      recording_adapter=args.recording_adapter,
                      viewer_backend=args.viewer_backend,
                      execution_guard=args.execution_guard,
                      simulation_backend=args.simulation_backend,
                      coordinator_math=args.coordinator_math,
                      **({'mapped_palm_height_calibration': True} if args.mapped_palm_height_calibration or args.mapped_palm_xz_calibration else {}),
                      **({'mapped_palm_xz_calibration': True} if args.mapped_palm_xz_calibration else {}),
                      **({'mapped_palm_common_x_reference': True} if args.mapped_palm_common_x_reference else {}),
                      manus_sdk_library_dir='external/manus/sdk' if args.manus_library_dir else None,
                      manus_sdk_loading='explicit_child_path' if args.manus_library_dir else 'inherited_unverified',
                      tjvr_stream_contract=stream_contract,
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
