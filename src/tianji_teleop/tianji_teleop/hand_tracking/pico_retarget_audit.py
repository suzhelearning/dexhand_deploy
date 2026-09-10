"""Passive PICO retarget consumption boundary, never an operation/command."""

TOPIC = 'tianji/observation/hand_retarget/pico_consumed'


def validate_consumed(value):
    keys = {'schema_version', 'kind', 'router_zid', 'publisher_instance_id',
            'receiver_instance_id', 'connection_generation', 'receiver_frame_sequence',
            'frame_association_id', 'worker_sequence', 'processing_sequence',
            'received_timestamp_ns', 'processed_timestamp_ns', 'valid_sides'}
    if not isinstance(value, dict) or set(value) != keys:
        raise ValueError('invalid PICO consumption audit fields')
    if (type(value['schema_version']) is not int or value['schema_version'] != 1 or
            value['kind'] != 'pico_hand_retarget_consumed'):
        raise ValueError('invalid PICO consumption audit version')
    for key in ('router_zid', 'publisher_instance_id', 'receiver_instance_id'):
        if not isinstance(value[key], str) or not value[key].strip() or '/' in value[key]:
            raise ValueError('explicit consumption identities required')
    for key in ('connection_generation', 'receiver_frame_sequence', 'worker_sequence',
                'processing_sequence', 'received_timestamp_ns', 'processed_timestamp_ns'):
        if type(value[key]) is not int or not 0 <= value[key] < 2**63:
            raise ValueError('invalid consumption sequence/time')
    if (value['worker_sequence'] != value['receiver_frame_sequence'] + 1 or
            value['processing_sequence'] < 1 or value['received_timestamp_ns'] < 1 or
            value['processed_timestamp_ns'] < value['received_timestamp_ns']):
        raise ValueError('inconsistent consumption sequence/time')
    association = f"{value['receiver_instance_id']}:{value['connection_generation']}:{value['receiver_frame_sequence']}"
    if value['frame_association_id'] != association:
        raise ValueError('consumption association mismatch')
    sides = value['valid_sides']
    if not isinstance(sides, list) or sides not in ([], ['left'], ['right'], ['left', 'right']):
        raise ValueError('invalid consumption side validity')
    return value
