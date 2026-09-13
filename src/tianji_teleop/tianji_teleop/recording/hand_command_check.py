"""Offline recorded Manus callbacks -> official hand command numbers only.

The explicit offline solver owns no router, coordinator or executor. This does
not reproduce authorization decisions or prove that a recorded command was
executed. Every callback, including idle callbacks, advances the original bridge,
except inputs explicitly audited as expired before live retargeting.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from ..producers.hand_retarget import OfficialHandClient
from .manus_check import check_manus_recording
from .session_h5 import SessionH5Reader


def _asset_key(root, path):
    """Use portable asset identifiers in new recordings."""
    return path.relative_to(root).as_posix()


def _asset_matches(recorded, root, key, digest):
    """Accept portable keys and absolute keys from old recordings."""
    if not isinstance(recorded, dict):
        return False
    return recorded.get(key) == digest or recorded.get(str(root / key)) == digest


def hand_replay_asset_hashes(root):
    """Hash only known local hand assets, never paths supplied by a recording."""
    root = Path(root).resolve(strict=True)
    official = root / 'third_party/wuji_hand_retargeting'
    manifest = official / 'source_manifest.json'
    paths = [manifest, root / 'scripts/wuji_hand_worker.py',
             root / 'tools/wuji_hand_native/pixi.lock',
             root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python']
    for row in json.loads(manifest.read_text())['files']:
        path = (official / row['destination']).resolve(strict=True)
        if not path.is_relative_to(official.resolve()):
            raise ValueError('hand asset escapes pinned source closure')
        paths.append(path)
    result = {}
    for path in paths:
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        result[_asset_key(root, path)] = digest.hexdigest()
    return result


def pico_hand_replay_asset_hashes(root):
    result = hand_replay_asset_hashes(root)
    package = Path(root).resolve(strict=True) / 'src/tianji_teleop/tianji_teleop'
    for relative in ('hand_tracking/official_pico.py', 'hand_tracking/pico.py',
                     'hand_tracking/models.py', 'producers/pico_official_hand.py',
                     'producers/hand_retarget.py'):
        path = package / relative
        result[_asset_key(root, path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def compare_recorded_hand_command(report, hand, command, *, side, sequence):
    """Shared numerical comparison; never creates or publishes a command."""
    def difference(field, **details):
        report['passed'] = False
        if report['first_difference'] is None:
            report['first_difference'] = dict(stage='hand_command', side=side,
                callback_sequence=sequence, field=field, **details)
    if not hand['valid']:
        difference('invalid_retarget_side')
    if hand['joint_names'] != command['names']:
        difference('joint_names')
    expected, actual = np.asarray(hand['position_rad']), np.asarray(command['position_rad'])
    if expected.shape != (20,) or actual.shape != (20,):
        raise ValueError('invalid hand command shape')
    error = np.abs(expected - actual)
    if not np.isfinite(error).all():
        raise ValueError('nonfinite hand command difference')
    report['max_error_rad'] = max(report['max_error_rad'], float(error.max()))
    bad = np.flatnonzero(error > 1e-5)
    if len(bad):
        joint = int(bad[0])
        difference('position_rad', joint_name=command['names'][joint], error_rad=float(error[joint]))
    report['matched_commands'] += 1


def check_manus_hand_commands(path, *, root, backend_factory=None, input_atol=1e-9):
    """Validate provenance, then replay callback order/time with no authority.

    An injected factory is for tests and is explicitly labelled in the report.
    CLI callers always use the existing isolated official worker.
    """
    # XR/Manus targets and executor commands intentionally use independent
    # sequence domains.  Route those recordings to the association-aware
    # checker without changing the legacy rawviz callback checker below.
    with SessionH5Reader(path) as reader:
        if reader.attrs.get('source_type') == 'vr_manus_xr_sim':
            from .xr_manus_hand_command_check import check_xr_manus_hand_commands
            return check_xr_manus_hand_commands(
                path, root=root, backend_factory=backend_factory, input_atol=input_atol
            )
    reconstruction = check_manus_recording(path, atol=input_atol)
    report = dict(passed=reconstruction['passed'], scope='manus_callback_to_recorded_hand_command',
        solver='injected_test_backend' if backend_factory is not None else 'official_wuji_hand2',
        input_reconstruction=reconstruction, replayed_callbacks=0, matched_commands=0,
        max_error_rad=0., tolerance_rad=1e-5, first_difference=reconstruction['first_difference'],
        operator_events_executed=0,
        limitations=['command numerical consistency only; no authorization/execution replay',
                     'asset hashes are snapshots, not runtime dependency attestation'])
    if not reconstruction['passed']:
        return report
    with SessionH5Reader(path) as reader:
        configuration = reader.read_hand_tracking_metadata()['resolved_configuration']
        recorded_assets = configuration.get('asset_sha256', {})
        root = Path(root).resolve(strict=True)
        for asset, digest in hand_replay_asset_hashes(root).items():
            if not _asset_matches(recorded_assets, root, asset, digest):
                raise ValueError(f'hand replay asset provenance mismatch: {asset}')
        callbacks = reader.read_manus_callbacks()
        expired = {row['payload']['callback_sequence'] for row in reader.read_dual_audit()
                   if row['kind'] in ('manus_expired_input', 'manus_superseded_input')}
        commands = {side: reader.read_hand_command(side) for side in ('left', 'right')}
    index = {}
    for side, rows in commands.items():
        for command in rows:
            key = (side, command['sequence'])
            if key in index:
                raise ValueError(f'duplicate recorded hand command: {key}')
            index[key] = command
    if not index:
        raise ValueError('no recorded hand commands to compare')
    callback_sequences = {row['callback_sequence'] for row in callbacks}
    if any(sequence not in callback_sequences for _, sequence in index):
        raise ValueError('recorded hand command has no associated callback')
    if not expired <= callback_sequences or any(sequence in expired for _, sequence in index):
        raise ValueError('expired Manus callback audit conflicts with raw inputs or commands')
    root = Path(root).resolve(strict=True)
    sides = configuration['manus_input_contract']['sides']
    factory = backend_factory if backend_factory is not None else OfficialHandClient
    options = {}
    if 'manus_filter_continuity_ns' in configuration:
        options['filter_continuity_ns'] = configuration['manus_filter_continuity_ns']
    backend = factory(python=root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
        script=root / 'scripts/wuji_hand_worker.py', single_hand_side='left' if sides == ['left'] else 'right',
        startup_handshake=True, **options)

    try:
        for callback in callbacks:
            sequence = callback['callback_sequence']
            if sequence in expired:
                continue  # Live discarded this input before advancing retarget state.
            result = backend.retarget(callback['points'], sequence=sequence,
                                      timestamp_ns=callback['received_timestamp_ns'])
            report['replayed_callbacks'] += 1
            for side in ('left', 'right'):
                command = index.get((side, sequence))
                if command is None:
                    continue  # No invented command during idle/return/shutdown.
                compare_recorded_hand_command(report, result[side], command, side=side, sequence=sequence)
    finally:
        backend.close()
    return report
