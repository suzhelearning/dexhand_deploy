from copy import deepcopy
import unittest

from tianji_teleop.protocol import bilateral
from tianji_teleop.protocol.messages import ARM_JOINT_NAMES, ArmJointCommand


def commands():
    return {side: ArmJointCommand(1, 8, 1000, 'coordinator', side, 'teleop', 3, 5,
                 list(ARM_JOINT_NAMES[side]), [0.] * 7, 'coord', 'router')
            for side in ('left', 'right')}


class BilateralArmCommandTest(unittest.TestCase):
    def pair(self):
        self.assertTrue(hasattr(bilateral, 'ArmBilateralCommand'))
        values = commands()
        return bilateral.ArmBilateralCommand('run', 1, 3, values['left'], values['right'])

    def test_roundtrip_snapshot_and_distinct_command_vs_native_tick(self):
        pair = self.pair()
        self.assertEqual(pair.left.sequence, 8)
        self.assertEqual(pair.reference_tick_id, 3)
        row = pair.to_dict()
        restored = bilateral.ArmBilateralCommand.from_dict(row)
        row['left']['position_rad'][0] = 1
        self.assertEqual(restored.left.position_rad, [0.] * 7)

    def test_mixed_sides_generations_or_partial_command_rejected(self):
        row = self.pair().to_dict()
        for key, value in (('sequence', 9), ('publisher_instance_id', 'other'), ('mode', 'idle')):
            bad = deepcopy(row)
            bad['right'][key] = value
            with self.assertRaises(ValueError):
                bilateral.ArmBilateralCommand.from_dict(bad)
        del row['right']
        with self.assertRaises(ValueError):
            bilateral.ArmBilateralCommand.from_dict(row)
