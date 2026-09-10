from copy import deepcopy
import unittest

from tianji_teleop import config_loader
from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND


class DualSessionConfigTest(unittest.TestCase):
    def test_pico_runtime_does_not_silently_replace_gesture_with_keyboard(self):
        from tianji_teleop.hand_tracking import session_config
        self.assertTrue(hasattr(session_config, 'validate_pico_runtime'))
        value = config_loader.load_yaml(config_loader.component_path('sessions/pico2_hands_sim.yaml'))
        resolved = self.resolve(value)
        session_config.validate_pico_runtime(resolved)
        gesture = dict(resolved, operator_input='gesture')
        session_config.validate_pico_runtime(gesture)
        self.assertEqual(gesture['operator_input'], 'gesture')
        for change in ({'operator_input': 'controller'}, {'active_sides': ['right']},
                       {'active_hand_sides': ['right']}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                session_config.validate_pico_runtime(dict(resolved, **change))

    def value(self):
        return dict(input_mode='vr_manus', hand_input='manus', arm_input='tjvr_corrected_palm',
            operator_input='keyboard', required_capability='simulation', active_sides=['left', 'right'],
            active_hand_sides=['left', 'right'], ik_backend=SPARK_BACKEND,
            arm_pose_mapper='none', retarget_owner='spark', hand_retarget_backend='official_wuji_hand2',
            arm_target_processor='passthrough', joint_trajectory='passthrough',
            command_step_clipping=False, joint_limit_source='urdf',
            reference_execution_mode='reference_direct', rate_hz=200.)

    def resolve(self, value, **options):
        self.assertTrue(hasattr(config_loader, 'resolve_dual_session_config'))
        return config_loader.resolve_dual_session_config(value, **options)

    def test_disable_hands_removes_only_manus_receiver(self):
        original = self.value()
        snapshot = deepcopy(original)
        result = self.resolve(original, disable_hands=True)
        self.assertEqual(result['receivers'], ['tjvr'])
        self.assertEqual(result['active_hand_sides'], [])
        self.assertEqual(original, snapshot)

    def test_source_mapper_owner_and_processing_mismatch_rejected(self):
        for key, value in (('arm_input', 'xr_controller'), ('arm_pose_mapper', 'head_direct'),
                           ('retarget_owner', 'mapper'), ('joint_trajectory', 'ruckig'),
                           ('command_step_clipping', True), ('active_sides', ['right']),
                           ('required_capability', 'real'), ('rate_hz', True), ('extra', 1)):
            with self.subTest(key=key):
                config = self.value()
                config[key] = value
                with self.assertRaises(ValueError):
                    self.resolve(config)

    def test_canonical_new_sim_profiles_resolve_without_starting_processes(self):
        for profile, receiver in (('pico2_hands_sim', 'pico2'), ('vr_manus_sim', 'tjvr')):
            value = config_loader.load_yaml(config_loader.component_path(f'sessions/{profile}.yaml'))
            resolved = self.resolve(value, disable_hands=True)
            self.assertEqual(resolved['receivers'], [receiver])
            self.assertEqual(resolved['required_capability'], 'simulation')
