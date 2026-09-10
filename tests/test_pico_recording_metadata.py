import json
import unittest
from unittest.mock import patch
from tianji_teleop.recording import session_recorder as recorder


class PicoRecordingMetadataTest(unittest.TestCase):
    def test_pico_records_resolved_route_and_legacy_ignores_new_environment(self):
        self.assertTrue(hasattr(recorder, '_session_metadata'))
        resolved = dict(profile='pico2_hands_sim', config=dict(input_mode='pico2_hands',
            hand_retarget_backend='official_wuji_hand2', ik_backend='pico_ee_dexhand_qp'))
        env = dict(TIANJI_RESOLVED_DUAL_SESSION=json.dumps(resolved), TIANJI_RUN_ID='run')
        value = recorder._session_metadata('pico2_hands_sim', env)
        self.assertEqual(value['resolved_configuration'], resolved)
        self.assertEqual(value['run_id'], 'run')
        self.assertEqual(value['pico_hand_adapter'], 'pico26_to_official_hand2_yflip_scale_v2')
        self.assertIsNone(recorder._session_metadata('hand_tracking_sim', env))
        with self.assertRaises(ValueError):
            recorder._session_metadata('pico2_hands_sim', {})

    def test_enabled_pico_hands_snapshot_replay_assets_but_disabled_hands_do_not(self):
        resolved = dict(profile='pico2_hands_sim', config=dict(input_mode='pico2_hands',
                        active_hand_sides=['left', 'right']))
        env = dict(TIANJI_RESOLVED_DUAL_SESSION=json.dumps(resolved), TIANJI_RUN_ID='run')
        with patch('tianji_teleop.recording.hand_command_check.pico_hand_replay_asset_hashes',
                   return_value={'pinned': 'digest'}) as hashes:
            value = recorder._session_metadata('pico2_hands_sim', env)
            self.assertEqual(value['hand_retarget_asset_sha256'], {'pinned': 'digest'})
            self.assertEqual(hashes.call_count, 1)
            resolved['config']['active_hand_sides'] = []
            env['TIANJI_RESOLVED_DUAL_SESSION'] = json.dumps(resolved)
            value = recorder._session_metadata('pico2_hands_sim', env)
            self.assertNotIn('hand_retarget_asset_sha256', value)
            self.assertEqual(hashes.call_count, 1)

    def test_xr_manus_records_parser_contract_and_official_asset_snapshot(self):
        resolved = dict(
            profile='vr_manus_xr_sim',
            config=dict(input_mode='vr_manus', active_hand_sides=['left', 'right']),
        )
        env = dict(
            TIANJI_RESOLVED_DUAL_SESSION=json.dumps(resolved),
            TIANJI_RUN_ID='run',
            TIANJI_MANUS_RIGHT_GLOVE='right-glove',
            TIANJI_MANUS_LEFT_GLOVE='left-glove',
        )
        with patch(
            'tianji_teleop.recording.hand_command_check.hand_replay_asset_hashes',
            return_value={'pinned': 'digest'},
        ) as hashes:
            value = recorder._session_metadata('vr_manus_xr_sim', env)
        configuration = value['resolved_configuration']
        self.assertEqual(
            configuration['manus_input_contract'],
            dict(
                version=1,
                sides=['right', 'left'],
                right_glove='right-glove',
                left_glove='left-glove',
                callback_order='right_then_left',
                callback_trigger='each_accepted_pose',
            ),
        )
        self.assertEqual(configuration['asset_sha256'], {'pinned': 'digest'})
        self.assertEqual(hashes.call_count, 1)

    def test_xr_manus_disabled_hands_do_not_snapshot_hand_assets(self):
        resolved = dict(
            profile='vr_manus_xr_sim',
            config=dict(input_mode='vr_manus', active_hand_sides=[]),
        )
        env = dict(TIANJI_RESOLVED_DUAL_SESSION=json.dumps(resolved), TIANJI_RUN_ID='run')
        with patch('tianji_teleop.recording.hand_command_check.hand_replay_asset_hashes') as hashes:
            value = recorder._session_metadata('vr_manus_xr_sim', env)
        self.assertEqual(value['resolved_configuration']['manus_input_contract']['sides'], [])
        self.assertNotIn('asset_sha256', value['resolved_configuration'])
        hashes.assert_not_called()
