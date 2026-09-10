"""Replay only documented PICO hand-worker inputs; no live authorization."""
from contextlib import ExitStack
from pathlib import Path

from ..hand_tracking.pico import parse_pico_packet
from ..hand_tracking.pico_retarget_audit import validate_consumed
from ..producers.hand_retarget import OfficialHandClient
from ..producers.pico_official_hand import PicoOfficialHandBackend
from .dual_check import check_pico_recording
from .hand_command_check import _asset_matches, pico_hand_replay_asset_hashes, compare_recorded_hand_command
from .session_h5 import SessionH5Reader


def check_pico_hand_commands(path, *, root, client_factory=None, input_atol=1e-9):
    reconstruction = check_pico_recording(path, atol=input_atol)
    report = dict(passed=reconstruction['passed'], scope='pico_consumed_input_to_recorded_hand_command',
        solver='injected_test_backend' if client_factory is not None else 'official_wuji_hand2',
        input_reconstruction=reconstruction, raw_frames=reconstruction['raw_frames'],
        replayed_consumed_frames=0, matched_commands=0, max_error_rad=0., tolerance_rad=1e-5,
        first_difference=reconstruction['first_difference'], operator_events_executed=0,
        limitations=['command numerical consistency only; no authorization/execution replay',
                     'asset hashes are snapshots, not runtime dependency attestation'])
    if not reconstruction['passed']:
        return report
    if reconstruction['raw_receiver_clock_frames'] != reconstruction['raw_frames']:
        raise ValueError('PICO hand replay requires original raw receiver clock')
    with SessionH5Reader(path) as reader:
        recorded_assets = reader.read_hand_tracking_metadata().get('hand_retarget_asset_sha256', {})
        root = Path(root).resolve(strict=True)
        for asset, digest in pico_hand_replay_asset_hashes(root).items():
            if not _asset_matches(recorded_assets, root, asset, digest):
                raise ValueError(f'PICO hand replay asset provenance mismatch: {asset}')
        raw = {row['association_id']: row for row in reader.read_raw_pico()}
        consumed = [validate_consumed(row['payload']) for row in reader.read_dual_audit()
                    if row['kind'] == 'hand_retarget_input']
        commands = {side: reader.read_hand_command(side) for side in ('left', 'right')}
        router = reader.attrs['router_zid']
    if not consumed:
        raise ValueError('PICO hand replay requires actual consumption audit; cannot infer worker start')
    identity_fields = ('publisher_instance_id', 'receiver_instance_id', 'connection_generation')
    identity = tuple(consumed[0][field] for field in identity_fields)
    previous = -1
    frames = []
    for ordinal, row in enumerate(consumed, 1):
        if row['processing_sequence'] != ordinal:
            raise ValueError('missing, repeated or reordered PICO consumption sequence')
        if tuple(row[field] for field in identity_fields) != identity or row['router_zid'] != router:
            raise ValueError('PICO hand replay requires one explicitly bound worker epoch')
        if row['receiver_frame_sequence'] <= previous:
            raise ValueError('PICO consumed receive ordinal must increase')
        previous = row['receiver_frame_sequence']
        packet = raw.get(row['frame_association_id'])
        if packet is None or packet['received_timestamp_ns'] != row['received_timestamp_ns']:
            raise ValueError('PICO consumed input lacks matching raw frame/time')
        frames.append(parse_pico_packet(packet['raw_packet'],
            receiver_instance_id=row['receiver_instance_id'], connection_generation=row['connection_generation'],
            receiver_frame_sequence=row['receiver_frame_sequence'], received_timestamp_ns=row['received_timestamp_ns']))
    sequences = {row['worker_sequence'] for row in consumed}
    index = {}
    for side, rows in commands.items():
        for command in rows:
            key = side, command['sequence']
            if key in index or key[1] not in sequences:
                raise ValueError('duplicate or unassociated PICO hand command')
            if command['publisher_instance_id'] != identity[0]:
                raise ValueError('PICO hand command publisher differs from consumption audit')
            index[key] = command
    if not index:
        raise ValueError('no recorded PICO hand commands to compare')
    root = Path(root).resolve(strict=True)
    factory = client_factory if client_factory is not None else OfficialHandClient
    with ExitStack() as stack:
        clients = {}
        for side in ('left', 'right'):
            client = factory(python=root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
                script=root / 'scripts/wuji_hand_worker.py', single_hand_side=side, startup_handshake=True)
            stack.callback(client.close)
            clients[side] = client
        backend = PicoOfficialHandBackend(clients, receiver_instance_id=identity[1], connection_generation=identity[2])
        for frame, audit in zip(frames, consumed):
            result = backend.retarget(frame)
            report['replayed_consumed_frames'] += 1
            valid = [side for side in ('left', 'right') if result[side]['valid']]
            if valid != audit['valid_sides']:
                report['passed'] = False
                if report['first_difference'] is None:
                    report['first_difference'] = dict(stage='pico_hand_consumption',
                        receiver_frame_sequence=frame.receiver_frame_sequence, field='valid_sides')
            for side in ('left', 'right'):
                command = index.get((side, audit['worker_sequence']))
                if command is not None:
                    compare_recorded_hand_command(report, result[side], command, side=side,
                                                  sequence=audit['worker_sequence'])
    return report
