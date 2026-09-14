import unittest

from scripts.check_mapped_palm_oracle import joint_errors
from scripts import check_mapped_palm_oracle as oracle


class MappedPalmOracleTest(unittest.TestCase):
    def reference_rows(self, count):
        rows = []
        for i in range(count):
            row = self.row(True)
            row.update({k.replace('left', 'right'): v for k, v in row.copy().items()})
            row['sequence'] = str(i)
            rows.append(row)
        return rows

    def test_empty_and_truncated_references_cannot_pass_full_trace(self):
        for count in (0, 1, 2):
            with self.subTest(count=count), self.assertRaises(ValueError):
                oracle.validate_reference_rows(self.reference_rows(count), 10_000_000)
        oracle.validate_reference_rows(self.reference_rows(3), 10_000_000)

    def test_missing_cycle_and_telemetry_mismatch_rejected(self):
        rows = self.reference_rows(3)
        rows[1]['sequence'] = '2'
        with self.assertRaises(ValueError):
            oracle.validate_reference_rows(rows, 10_000_000)
        with self.assertRaises(ValueError):
            oracle.validate_reference_rows(self.reference_rows(3), 10_000_000, [])

    def test_nonfinite_joint_values_rejected_including_invalid_plot_samples(self):
        for valid in (True, False):
            for field in ('q', 'qdot', 'qddot'):
                for value in ('nan', 'inf', '-inf'):
                    row = self.row(valid)
                    row[f'left_j1_reference_{field}'] = value
                    with self.subTest(valid=valid, field=field, value=value), self.assertRaises(ValueError):
                        joint_errors(row, {'q': [0.] * 7, 'qdot': [0.] * 7, 'qddot': [0.] * 7}, 'left')

    def test_nonfinite_diagnostics_rejected_before_comparison(self):
        row = {'sequence': '0', 'pico_live': '1', 'pico_tracking_epoch': '1', 'pico_sequence': '1'}
        for side in ('left', 'right'):
            for col in ('pico_ee_headroom_scale', 'task_scale_position', 'task_scale_orientation'):
                row[f'{side}_{col}'] = '1'
        oracle.validate_reference_rows(self.reference_rows(1), 0, [row])
        row['left_task_scale_position'] = 'nan'
        with self.assertRaises(ValueError):
            oracle.validate_reference_rows(self.reference_rows(1), 0, [row])

    def row(self, valid):
        row = {'left_reference_acceleration_valid': str(int(valid))}
        for field in ('q', 'qdot', 'qddot'):
            for joint in range(1, 8):
                row[f'left_j{joint}_reference_{field}'] = '0'
        return row

    def test_reset_plot_acceleration_is_not_controller_acceleration(self):
        actual = {'q': [0.] * 7, 'qdot': [0.] * 7, 'qddot': [30.] * 7}
        self.assertEqual(joint_errors(self.row(False), actual, 'left'),
                         {'q': 0., 'qdot': 0.})

    def test_valid_acceleration_difference_is_still_checked(self):
        actual = {'q': [.1] * 7, 'qdot': [.2] * 7, 'qddot': [30.] * 7}
        self.assertEqual(joint_errors(self.row(True), actual, 'left'),
                         {'q': .1, 'qdot': .2, 'qddot': 30.})

    def test_missing_validity_is_not_silently_skipped(self):
        row = self.row(True)
        del row['left_reference_acceleration_valid']
        with self.assertRaises(KeyError):
            joint_errors(row, {'q': [0.] * 7, 'qdot': [0.] * 7,
                               'qddot': [0.] * 7}, 'left')
