"""Resolve new dual-input session contracts, without starting any components.

This is deliberately separate from legacy session YAML parsing. A resolved
contract is not proof of runtime availability, calibration or authorization.
"""
from copy import deepcopy
import math
from collections.abc import Mapping

from .input_modes import resolve_input_mode, validate_ik_input, SPARK_BACKEND

_INPUT = {'input_mode', 'hand_input', 'arm_input', 'operator_input'}
_COMMON = _INPUT | {'required_capability', 'active_sides', 'active_hand_sides', 'ik_backend',
    'arm_pose_mapper', 'retarget_owner', 'hand_retarget_backend', 'arm_target_processor',
    'joint_trajectory', 'command_step_clipping', 'joint_limit_source', 'rate_hz'}

def validate_pico_runtime(config):
    """Validate the implemented live composition, not just the future contract."""
    if config.get('input_mode') != 'pico2_hands' or config.get('required_capability') != 'simulation':
        raise ValueError('PICO live requires pico2_hands simulation')
    if config.get('operator_input') not in ('keyboard', 'gesture'):
        raise ValueError('PICO live operator binding must be keyboard or explicit gesture start')
    if (sorted(config.get('active_sides', [])) != ['left', 'right'] or
            config.get('active_hand_sides') not in ([], ['left', 'right'])):
        raise ValueError('PICO live requires both arms and either both hands or disabled hands')


def resolve_session(value, *, disable_hands=False):
    if not isinstance(value, Mapping) or type(disable_hands) is not bool:
        raise ValueError('session mapping and boolean disable_hands required')
    if not _COMMON <= set(value) or set(value) - _COMMON - {'reference_execution_mode'}:
        raise ValueError('unknown or missing dual-input session fields')
    config = deepcopy(dict(value))
    mode = resolve_input_mode({key: config[key] for key in _INPUT})
    validate_ik_input(mode, config['ik_backend'])
    if config['required_capability'] != 'simulation':
        raise ValueError('new dual-input backends are not qualified for real execution')
    if (type(config['rate_hz']) not in (float, int) or not math.isfinite(config['rate_hz']) or
            config['rate_hz'] != 200):
        raise ValueError('reference integration requires an explicit 200Hz control rate')
    for key in ('active_sides', 'active_hand_sides'):
        sides = config[key]
        if (not isinstance(sides, list) or not sides or
                any(side not in ('left', 'right') for side in sides) or len(set(sides)) != len(sides)):
            raise ValueError(f'{key} must contain distinct active left/right sides')
    if config['hand_retarget_backend'] != 'official_wuji_hand2':
        raise ValueError('new session requires the explicit official Hand2 backend')
    if type(config['command_step_clipping']) is not bool:
        raise ValueError('command_step_clipping must be boolean')
    if config['joint_limit_source'] not in ('yaml', 'urdf'):
        raise ValueError('unknown joint limit source')
    if config['arm_target_processor'] not in ('passthrough', 'conditioned'):
        raise ValueError('unknown arm target processor')
    if config['joint_trajectory'] not in ('passthrough', 'ruckig'):
        raise ValueError('unknown joint trajectory processor')
    if config['ik_backend'] == SPARK_BACKEND:
        if (config['arm_pose_mapper'] != 'none' or config['retarget_owner'] != 'spark' or
                sorted(config['active_sides']) != ['left', 'right'] or config['joint_limit_source'] != 'urdf'):
            raise ValueError('SPARK owns bilateral skeleton retargeting; no pose remapping or shared YAML limits')
        if config.get('reference_execution_mode') != 'reference_direct':
            raise ValueError('processed_guarded is not yet enabled; reference_direct must be explicit')
        if (config['arm_target_processor'] != 'passthrough' or config['joint_trajectory'] != 'passthrough' or
                config['command_step_clipping']):
            raise ValueError('reference_direct prohibits additional shaping, trajectory smoothing and clipping')
    elif mode.input_mode == 'pico2_hands':
        if config['arm_pose_mapper'] not in ('relative_home', 'head_direct', 'head_palm_direct'):
            raise ValueError('PICO2 requires an explicit supported head/wrist mapper')
        if config['retarget_owner'] != 'mapper' or 'reference_execution_mode' in config:
            raise ValueError('PICO2 pose mapping must not claim SPARK reference-state ownership')
        if config['joint_limit_source'] == 'urdf' and config['ik_backend'] != 'pico_ee_dexhand_qp':
            raise ValueError('PICO2 URDF limit source requires pico_ee_dexhand_qp')
    elif mode.input_mode == 'vr_manus' and mode.arm_input in {'xr_controller', 'xr_tracker'}:
        if config['arm_pose_mapper'] != 'xr_incremental':
            raise ValueError('XR arm input requires the xr_incremental pose mapper')
        if config['retarget_owner'] != 'mapper' or 'reference_execution_mode' in config:
            raise ValueError('XR pose mapping must use the generic mapper/IK path')
        if config['joint_limit_source'] == 'urdf' and config['ik_backend'] != 'pico_ee_dexhand_qp':
            raise ValueError('XR URDF limit source requires pico_ee_dexhand_qp')
    else:
        raise ValueError('unsupported dual-input pose pipeline')
    if disable_hands:
        config['active_hand_sides'] = []
    config['receivers'] = [receiver for receiver in mode.receivers
                           if not (disable_hands and receiver == 'manus')]
    config['requires_upper_limb_skeleton'] = mode.requires_upper_limb_skeleton
    config['hands_enabled'] = not disable_hands
    return config
