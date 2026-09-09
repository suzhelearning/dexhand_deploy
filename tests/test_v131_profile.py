from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'src/tianji_teleop/config'


class V131ProfileTest(unittest.TestCase):
    def test_producer_preserves_model_reference_and_teleop_gate(self):
        source = (ROOT / 'src/tianji_teleop/src/producers/arm_ik_producer_node.cpp').read_text()
        self.assertIn('solver_->owns_reference_state() && !teleop_active_[index]', source)
        self.assertIn('result.model_state_only ? result.reference_velocity_rad_s', source)
        self.assertIn('trajectory_limiters_[index].update_position(result.joints_rad)', source)
        self.assertIn('result.model_state_only && trajectory_processor_ == "passthrough"', source)
        self.assertIn('"model_state_only"', source.replace('\\"', '"'))

    def test_full_profile_step_contract_is_isolated(self):
        original = yaml.safe_load((CONFIG / 'coordinator/arm.yaml').read_text())
        producer = yaml.safe_load((CONFIG / 'producers/ik_dexhand_qp.yaml').read_text())
        self.assertAlmostEqual(producer['maximum_joint_step_rad'], 4 / producer['rate_hz'])
        dedicated = yaml.safe_load((CONFIG / 'coordinator/arm_v131.yaml').read_text())
        self.assertEqual(dedicated['maximum_command_step_rad'], producer['maximum_joint_step_rad'])
        self.assertEqual(dedicated['command_step_time_window_s'], .05)
        self.assertNotIn('command_step_time_window_s', original)
        self.assertEqual(original['maximum_command_step_rad'], 0.00596902599)
        for key in original:
            if key != 'maximum_command_step_rad':
                self.assertEqual(original[key], dedicated[key])
        launcher = (ROOT / 'scripts/run_session.sh').read_text()
        branch = launcher.split('if [[ "${ik_backend_override}" == pico_ee_dexhand_qp ]]; then')[1].split('\nfi')[0]
        self.assertIn('coordinator_config=coordinator/arm_v131.yaml', branch)


if __name__ == '__main__':
    unittest.main()
