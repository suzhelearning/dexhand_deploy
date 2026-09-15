"""Fixed result ABI v1; reconstruct the existing diagnostic/recording contract.

Only the per-tick response changes. No actuator authority, retries or fallback.
The C++ writer lives at src/ik/native_binary_results.hpp. Keep explicit sizes
and little-endian fields; never send compiler-dependent native struct layouts.
"""
import math
import struct
from .input_modes import SPARK_BACKEND, MAPPED_PALM_BACKEND

_COMMON = [('deterministic_test', 'B'), ('tick_id', 'Q'), ('timestamp_ns', 'Q'),
    ('applied_epoch', 'Q'), ('applied_sequence', 'Q'), ('epoch_reset', 'B'),
    ('input_live', 'B'), ('button_action', 'i'), ('control_executed', 'B')]
_ARM = [('q', '7d'), ('qdot', '7d'), ('qddot', '7d'), ('accepted', 'B'),
    ('qp_status', 'i'), ('hold_reason', 'i'), ('headroom_scale', 'd'),
    ('task_scale_position', 'd'), ('task_scale_orientation', 'd'),
    ('target_position', '3d'), ('target_quaternion_xyzw', '4d')]
_GUIDANCE = [('stage1_q', '7d'), ('ik_q', '7d'), ('ik_accepted', 'B'),
    ('stage1_iterations', 'i'), ('stage2_iterations', 'i'), ('budget_exhausted', 'B'),
    ('feedforward_q', '7d'), ('feedforward_qdot', '7d'), ('feedforward_qddot', '7d'),
    ('feedforward_state', 'i'), ('feedforward_target_accepted', 'B'), ('headroom_state', 'i'),
    ('settled_hold', 'B'), ('settled_hold_reason', 'i'), ('stationary_hold', 'B')]
_EXTRA = {
    'spark': [('joint_takeover_cycle', 'B'), ('guidance_accepted', 'B'),
              ('guidance_updates', 'Q'), ('headroom_updates', 'Q')],
    'mapped_palm': [('_height_present', 'B'), ('target_height_offsets_m', '2d')]}
_IDENTITY = {'spark': (1, SPARK_BACKEND), 'mapped_palm': (2, MAPPED_PALM_BACKEND)}
_SPECS = {}
_STRUCTS = {}
for _prefix in _IDENTITY:
    _spec = list(_COMMON) + _EXTRA[_prefix]
    for _side in ('left', 'right'):
        _spec += [(_side + '.' + key, fmt) for key, fmt in
                  (_ARM + (_GUIDANCE if _prefix == 'spark' else []))]
    _spec += [('_native_step_ns', 'Q'), ('_native_encode_ns', 'Q')]
    _SPECS[_prefix] = _spec
    _STRUCTS[_prefix] = struct.Struct('<' + ''.join(fmt for _, fmt in _spec))


def frame_size(prefix):
    return 8 + _STRUCTS[prefix].size


def payload_size(header, prefix):
    if len(header) != 8:
        raise ValueError('incomplete native binary header')
    magic, version, backend, size = struct.unpack('<4sBBH', header)
    if (magic != b'TJBR' or version != 1 or backend != _IDENTITY[prefix][0]
            or size != _STRUCTS[prefix].size):
        raise ValueError('incompatible native binary result header')
    return size


def decode_result(data, prefix):
    size = payload_size(data[:8], prefix)
    if len(data) != 8 + size:
        raise ValueError('incomplete or unsolicited native binary result')
    values = _STRUCTS[prefix].unpack_from(data, 8)
    row = dict(schema_version=1, kind=prefix + '_bilateral_result',
        algorithm=_IDENTITY[prefix][1], state_source='model_reference',
        simulation_only=True, left={}, right={})
    index = 0
    for key, fmt in _SPECS[prefix]:
        count = int(fmt[:-1]) if len(fmt) > 1 else 1
        if fmt.endswith('d'):
            fields = values[index:index+count]
            if not all(math.isfinite(v) for v in fields):
                raise ValueError('non-finite native binary result')
            value = list(fields) if len(fmt) > 1 else fields[0]
        else:
            value = values[index]
            if fmt == 'B':
                if value not in (0, 1):
                    raise ValueError('invalid native binary boolean')
                value = bool(value)
        index += count
        if '.' in key:
            side, key = key.split('.', 1)
            row[side][key] = value
        else:
            row[key] = value
    row.pop('_native_step_ns')
    row.pop('_native_encode_ns')
    if prefix == 'mapped_palm' and not row.pop('_height_present'):
        row.pop('target_height_offsets_m')
    return row


def native_timing(data):
    solve, encode = struct.unpack_from('<QQ', data, len(data)-16)
    return dict(native_step=solve*1e-9, native_result_encode=encode*1e-9)
