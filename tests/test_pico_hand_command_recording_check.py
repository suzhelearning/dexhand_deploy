from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tests.test_pico_hand_service import wire_frame
from tests.test_hand_command_recording_check import Backend
from tianji_teleop.hand_tracking.runtime import pico_frame_observations
from tianji_teleop.protocol.messages import HandJointCommand, HAND_JOINT_NAMES
from tianji_teleop.recording.session_h5 import SessionH5Writer

ROOT = Path(__file__).resolve().parents[1]


class SideBackend(Backend):
    def __init__(self, side):
        super().__init__()
        self.side = side

    def retarget(self, points, *, sequence, timestamp_ns):
        result = super().retarget(points, sequence=sequence, timestamp_ns=timestamp_ns)
        result.update(callback_sequence=sequence, timestamp_ns=timestamp_ns)
        result['left' if self.side == 'right' else 'right']['valid'] = False
        return result


class PicoHandCommandRecordingCheckTest(unittest.TestCase):
    def recording(self, path, *, omit_first_consumption=False):
        with SessionH5Writer(path, source_type='pico2_hands_sim', robot_model='test',
                router_zid='router', schema_version='1.2',
                metadata={'hand_retarget_asset_sha256': {'test-asset': 'digest'}}) as writer:
            for sequence in (5, 6, 7):
                frame = replace(wire_frame(), receiver_frame_sequence=sequence,
                                received_timestamp_ns=1000 + sequence)
                writer.append_raw_pico(frame)
                for hand, arm in pico_frame_observations(frame).values():
                    writer.append_hand_observation(hand)
                    writer.append_arm_input_observation(arm)
                if sequence == 5:
                    continue  # Received before the hand worker subscribed.
                if not (omit_first_consumption and sequence == 6):
                    writer.append_dual_audit('hand_retarget_input', dict(schema_version=1,
                        kind='pico_hand_retarget_consumed', router_zid='router', publisher_instance_id='producer',
                        receiver_instance_id='pico', connection_generation=3,
                        receiver_frame_sequence=sequence, frame_association_id=frame.association_id,
                        worker_sequence=sequence + 1, processing_sequence=sequence - 5,
                        received_timestamp_ns=1000 + sequence, processed_timestamp_ns=1100 + sequence,
                        valid_sides=['left', 'right']), received_timestamp_ns=1200 + sequence)
                for side in ('left', 'right'):
                    writer.append_hand_command(HandJointCommand(1, sequence + 1, 1000 + sequence,
                        'hand', side, list(HAND_JOINT_NAMES[side]), [.01 * (sequence + 1)] * 20,
                        'producer', 'router'), received_time_ns=1300 + sequence)

    def test_only_actually_consumed_frames_advance_independent_side_workers(self):
        from tianji_teleop.recording.pico_hand_command_check import check_pico_hand_commands
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path)
            clients = []
            def factory(**kwargs):
                client = SideBackend(kwargs['single_hand_side'])
                clients.append(client)
                return client
            with patch('tianji_teleop.recording.pico_hand_command_check.pico_hand_replay_asset_hashes',
                       return_value={'test-asset': 'digest'}), patch('socket.socket', side_effect=AssertionError('network')):
                report = check_pico_hand_commands(path, root=ROOT, client_factory=factory)
            self.assertTrue(report['passed'], report)
            self.assertEqual(report['raw_frames'], 3)
            self.assertEqual(report['replayed_consumed_frames'], 2)
            self.assertEqual(report['matched_commands'], 4)
            self.assertEqual([client.sequences for client in clients], [[7, 8], [7, 8]])
            self.assertTrue(all(client.closed for client in clients))
            self.assertEqual(report['operator_events_executed'], 0)

    def test_missing_first_consumption_is_not_guessed_from_raw_or_commands(self):
        from tianji_teleop.recording.pico_hand_command_check import check_pico_hand_commands
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.h5'
            self.recording(path, omit_first_consumption=True)
            with patch('tianji_teleop.recording.pico_hand_command_check.pico_hand_replay_asset_hashes',
                       return_value={'test-asset': 'digest'}), patch(
                       'tianji_teleop.recording.pico_hand_command_check.OfficialHandClient') as factory:
                with self.assertRaisesRegex(ValueError, 'consumption sequence'):
                    check_pico_hand_commands(path, root=ROOT)
                factory.assert_not_called()
