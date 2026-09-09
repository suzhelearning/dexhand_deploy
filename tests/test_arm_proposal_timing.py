from dataclasses import replace
import unittest

import test_arm_coordinator as fixtures


class ProposalTimingTest(unittest.TestCase):
    def setUp(self):
        setup = fixtures.ArmCommandCoordinatorTest()
        setup.setUp()
        setup.test_one_tick_proposal_lag_is_clipped_not_faulted()
        self.c = setup.coordinator
        self.c.config.update(maximum_command_step_rad=.02,
                             command_step_clipping_enabled=False,
                             command_step_time_window_s=.05)
        self.t0 = 1_000_000_000
        self.template = self.c._proposals['right'].value
        # Adopt a baseline proposal, exactly as the real command loop does.
        self.q0 = list(self.c._safe_command['right'])
        self.submit(3, self.t0, 0)
        self.c._command('right', 3, self.t0)

    def submit(self, sequence, stamp, delta, received=None):
        position = list(self.q0)
        position[0] += delta
        return self.c.update_proposal(replace(self.template, sequence=sequence,
            timestamp_ns=stamp, position_rad=position),
            received_ns=stamp if received is None else received)

    def test_three_legal_model_steps_survive_dropped_proposals(self):
        self.assertTrue(self.submit(12, self.t0+15_000_000, .045))
        self.c._validate_proposals(self.t0+15_000_000)
        self.assertEqual(self.c.state.state, 'teleop')
        out = self.c._command('right', 12, self.t0+15_000_000)
        self.assertAlmostEqual(out.position_rad[0]-self.q0[0], .045)

    def test_received_but_undelivered_proposal_does_not_move_anchor(self):
        self.submit(6, self.t0+5_000_000, .015)
        self.submit(9, self.t0+10_000_000, .030)
        self.test_three_legal_model_steps_survive_dropped_proposals()

    def test_first_proposal_can_accumulate_after_teleop_home_command(self):
        self.c._proposals.clear()
        self.c._state = self.c._make_state('idle', 'reset', 1)
        self.c._command('right', 1, self.t0)
        self.q0 = list(self.c._safe_command['right'])
        self.c._state = self.c._make_state('teleop', 'start', 2)
        self.c._command('right', 2, self.t0)
        self.test_three_legal_model_steps_survive_dropped_proposals()

    def test_excessive_speed_still_faults(self):
        self.submit(12, self.t0+15_000_000, .10)
        self.c._validate_proposals(self.t0+15_000_000)
        self.assertEqual(self.c.state.state, 'fault')

    def test_time_budget_is_capped(self):
        self.submit(12, self.t0+150_000_000, .30)
        self.c._validate_proposals(self.t0+150_000_000)
        self.assertEqual(self.c.state.state, 'fault')

    def test_repeated_command_does_not_refresh_proposal_clock(self):
        for i in range(3):
            self.c._command('right', 4+i, self.t0+(i+1)*5_000_000)
        self.test_three_legal_model_steps_survive_dropped_proposals()

    def test_stationary_failure_hold_preserves_reference_clock(self):
        hold = replace(self.template, sequence=9, timestamp_ns=self.t0+10_000_000,
            position_rad=list(self.q0), diagnostics={'hold': True})
        self.assertTrue(self.c.update_proposal(hold, received_ns=self.t0+10_000_000))
        self.c._command('right', 9, self.t0+10_000_000)
        self.test_three_legal_model_steps_survive_dropped_proposals()

    def test_receive_delay_does_not_grant_motion_budget(self):
        self.submit(12, self.t0+1_000_000, .045, received=self.t0+45_000_000)
        self.c._validate_proposals(self.t0+45_000_000)
        self.assertEqual(self.c.state.state, 'fault')

    def test_timestamp_rollback_rejected(self):
        self.assertFalse(self.submit(12, self.t0-1, .001, received=self.t0+1))
        self.assertEqual(self.c.state.state, 'fault')

    def test_legacy_guard_unchanged(self):
        self.c.config['command_step_time_window_s'] = 0
        self.submit(12, self.t0+15_000_000, .045)
        self.c._validate_proposals(self.t0+15_000_000)
        self.assertEqual(self.c.state.state, 'fault')

    def test_window_config_validated(self):
        for window in (-1, float('nan'), True, .3):
            with self.subTest(window=window), self.assertRaises(ValueError):
                self.c._coordinator_config(dict(self.c.config, command_step_time_window_s=window))
        accepted = self.c._coordinator_config(self.c.config)
        self.assertEqual(accepted['command_step_time_window_s'], .05)

    def test_newly_adopted_reference_resets_elapsed_budget(self):
        self.test_three_legal_model_steps_survive_dropped_proposals()
        self.submit(15, self.t0+20_000_000, .095)
        self.c._validate_proposals(self.t0+20_000_000)
        self.assertEqual(self.c.state.state, 'fault')
        self.assertAlmostEqual(self.c._step_rejection['allowed_rad'], .04)

    def test_full_tick_loop_with_two_out_of_three_proposals_dropped(self):
        delta = 0.0
        last_delivered = 0.0
        for tick in range(1, 121):
            now = self.t0+tick*5_000_000
            delta += .015 if (tick-1)%30 < 15 else -.015
            if tick%3 == 0:
                self.assertTrue(self.submit(3+tick*3, now, delta))
                last_delivered = delta
            out = self.c.tick(now_ns=now)['right']
            self.assertEqual(self.c.state.state, 'teleop', self.c.state.reason)
            self.assertAlmostEqual(out.position_rad[0]-self.q0[0], last_delivered)

    def test_old_source_timestamp_rejected_despite_fresh_receive(self):
        self.submit(12, self.t0+15_000_000, .045, received=self.t0+250_000_000)
        self.c._validate_proposals(self.t0+250_000_000)
        self.assertEqual(self.c.state.state, 'fault')
