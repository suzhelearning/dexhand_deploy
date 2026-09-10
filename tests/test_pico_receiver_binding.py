from pathlib import Path
import unittest
from tianji_teleop.hand_tracking import observation_node as node


class PicoReceiverBindingTest(unittest.TestCase):
    def test_explicit_receiver_binding_does_not_change_default_config(self):
        self.assertTrue(hasattr(node, '_runtime_config'))
        path = Path(__file__).resolve().parents[1] / 'src/tianji_teleop/config/sources/hand_tracking_observation.yaml'
        original = node._load_config(path)
        args = node._parser().parse_args(['--config', str(path)])
        self.assertEqual(node._runtime_config(args), original)
        args = node._parser().parse_args(['--config', str(path), '--receiver-instance-id', 'run-pico'])
        self.assertEqual(node._runtime_config(args)['receiver_instance_id'], 'run-pico')
        self.assertEqual(node._load_config(path), original)
        args.receiver_instance_id = ' '
        with self.assertRaises(ValueError):
            node._runtime_config(args)
