import importlib.util
import unittest

from tianji_teleop.protocol.messages import ArmJointProposal, ARM_JOINT_NAMES


def pair(tick=1, position=0.0):
    return dict(schema_version=1, kind='arm_bilateral_proposal', run_id='run-1',
                execution_epoch=1, tick_id=tick,
                **{side: ArmJointProposal(1, tick, 1_000_000_000, 'ik', side, 10,
                    list(ARM_JOINT_NAMES[side]), [position] * 7, {},
                    'ik-instance', 'router-1').to_dict() for side in ('left', 'right')})


class BilateralProposalTest(unittest.TestCase):
    def parser(self):
        name = 'tianji_teleop.protocol.bilateral'
        self.assertIsNotNone(importlib.util.find_spec(name), 'missing bilateral wire contract')
        from tianji_teleop.protocol.bilateral import ArmBilateralProposal
        return ArmBilateralProposal.from_dict

    def test_roundtrip_preserves_both_sides_and_tick(self):
        value = pair()
        result = self.parser()(value)
        self.assertEqual(result.to_dict(), value)
        value['left']['position_rad'][0] = 99
        self.assertEqual(result.left.position_rad[0], 0.)

    def test_mixed_cycles_or_producers_cannot_form_pair(self):
        parse = self.parser()
        for key, value in (('sequence', 2), ('timestamp_ns', 2), ('target_sequence', 11),
                           ('publisher_instance_id', 'other'), ('producer', 'other'),
                           ('router_zid', 'other'), ('side', 'left')):
            raw = pair()
            raw['right'][key] = value
            with self.assertRaises(ValueError, msg=key):
                parse(raw)
        raw = pair()
        del raw['right']
        with self.assertRaises(ValueError):
            parse(raw)

    def test_invalid_identity_and_tick_rejected(self):
        parse = self.parser()
        for key, value in (('run_id', ''), ('execution_epoch', True), ('tick_id', 0),
                           ('tick_id', 2),
                           ('schema_version', True), ('extra', 1)):
            raw = pair()
            raw[key] = value
            with self.assertRaises(ValueError):
                parse(raw)
