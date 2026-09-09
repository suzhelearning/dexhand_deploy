import unittest
from pathlib import Path

from tianji_teleop.hand_tracking.target_node import _load_config, create_bridge_from_config
from tests.test_hand_tracking_target_bridge import _arm


class PicoArmOnlyTest(unittest.TestCase):
    def test_valid_wrists_start_and_move_without_hand_skeleton(self):
        root = Path(__file__).resolve().parents[1]
        config = _load_config(root / 'src/tianji_teleop/config/sources/hand_tracking_target.yaml')
        config['active_hand_sides'] = ()
        bridge = create_bridge_from_config(config, router_zid='router', observation_publisher_instance_id='obs')
        for side in ('left', 'right'):
            message = _arm(pose=[0, 0, 0, 0, 0, 0, 1])
            message.side = side
            bridge.ingest_arm_observation(message)
        bridge.start(now_ns=1_000_000_000)
        result = bridge.tick(now_ns=1_000_000_000)
        self.assertEqual(len(result.arm), 2)
        self.assertEqual(len(result.hand), 0)
