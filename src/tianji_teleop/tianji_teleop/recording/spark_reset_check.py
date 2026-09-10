"""Passive reset/epoch journal checks for the current VR simulation contract."""
import math
import sys

from .session_h5 import SessionH5Reader
from .tjvr_check import check_tjvr_recording


def check_reset_audits(audits, *, run_id):
    """The managed VR core starts at execution epoch 1. Never execute events.

    Ack shape and zero derivatives are checked, not physical Home/stationarity
    or authorization. The internal fresh-input cutoff is not in old audits.
    """
    if not isinstance(run_id, str) or not run_id.strip():
        raise ValueError('explicit recorded run_id required')
    report = dict(passed=True, first_difference=None, validated_reset_acks=0,
        rejected_rearms=0, checked_native_cycles=0, initial_execution_epoch=1,
        operator_events_executed=0, physical_state_verified=False,
        limitations=['current managed VR initial epoch 1 contract only',
            'does not compare ack positions to physical feedback or prove Home/stationarity',
            'does not reconstruct authorization or internal fresh-input cutoff',
            'does not execute reset, replay native dynamics, or authorize any device'])

    def difference(index, field):
        report['passed'] = False
        if report['first_difference'] is None:
            report['first_difference'] = dict(audit_index=index, field=field)

    epoch, previous_stamp, reset_stamp = 1, 0, 0
    for index, row in enumerate(audits):
        payload = row['payload']
        rearm = row['kind'] == 'operator_result' and payload.get('action') == 'rearm'
        if not rearm and row['kind'] != 'native_cycle':
            continue
        stamp = row['received_timestamp_ns']
        if type(stamp) is not int or not 0 < stamp < 2**63:
            difference(index, 'audit_clock')
            continue
        if stamp < previous_stamp:
            difference(index, 'audit_clock_order')
        previous_stamp = stamp
        if payload.get('run_id') != run_id:
            difference(index, 'run_id')
            continue
        if rearm:
            accepted = payload.get('accepted')
            if type(accepted) is not bool:
                difference(index, 'rearm_accepted_type')
                continue
            if not accepted:
                report['rejected_rearms'] += 1
                continue
            next_epoch = payload.get('execution_epoch')
            if type(next_epoch) is not int or next_epoch != epoch + 1:
                difference(index, 'reset_execution_epoch')
                continue
            ack = payload.get('reset_ack')
            fields = {'schema_version', 'kind', 'execution_epoch', 'position_rad',
                      'velocity_rad_s', 'acceleration_rad_s2'}
            if not isinstance(ack, dict) or set(ack) != fields:
                difference(index, 'reset_ack_fields')
                continue
            if (type(ack['schema_version']) is not int or ack['schema_version'] != 1 or
                    ack['kind'] != 'spark_reset_ack' or type(ack['execution_epoch']) is not int or
                    ack['execution_epoch'] != next_epoch):
                difference(index, 'reset_ack_identity')
                continue
            valid = True
            for field in ('position_rad', 'velocity_rad_s', 'acceleration_rad_s2'):
                values = ack[field]
                if (not isinstance(values, list) or len(values) != 14 or
                        any(type(value) not in (int, float) or abs(value) > sys.float_info.max or
                            not math.isfinite(value) for value in values)):
                    difference(index, 'reset_ack.' + field)
                    valid = False
                elif field != 'position_rad' and any(value != 0 for value in values):
                    difference(index, 'reset_ack.' + field)
                    valid = False
            if valid:
                epoch, reset_stamp = next_epoch, stamp
                report['validated_reset_acks'] += 1
            continue
        report['checked_native_cycles'] += 1
        if type(payload.get('execution_epoch')) is not int or payload['execution_epoch'] != epoch:
            difference(index, 'native_epoch_without_matching_reset')
        attempt = payload.get('native_attempt')
        if attempt is not None:
            if not isinstance(attempt, dict) or type(attempt.get('timestamp_ns')) is not int:
                difference(index, 'native_attempt_clock')
            elif not reset_stamp < attempt['timestamp_ns'] <= stamp:
                difference(index, 'native_attempt_before_reset_or_after_audit')
    return report


def check_spark_reset_recording(path):
    """Stricter opt-in audit stage, preserving ordinary TJVR check behavior."""
    result = check_tjvr_recording(path)
    if not result['passed']:
        result['reset_audit'] = dict(passed=False, status='not_checked_input_difference')
        return result
    if result['native_input_check'] not in ('checked', 'no_native_attempts'):
        raise ValueError('native cycle consumption boundaries required for reset audit check')
    with SessionH5Reader(path) as reader:
        audits = reader.read_dual_audit()
        run_id = reader.read_hand_tracking_metadata().get('run_id')
    reset_report = check_reset_audits(audits, run_id=run_id)
    result['reset_audit'] = reset_report
    if not reset_report['passed']:
        result['passed'] = False
        result['first_difference'] = dict(stage='reset_audit', **reset_report['first_difference'])
    return result
