import os
import unittest
from unittest.mock import patch

from tianji_teleop.executors.mujoco import node


class PicoMujocoEntryTest(unittest.TestCase):
    def test_default_entry_keeps_original_executor(self):
        self.assertTrue(hasattr(node, '_select_executor'))
        executor, options = node._select_executor(node._parse_args([]))
        self.assertIs(executor, node.MujocoExecutor)
        self.assertEqual(options, {})

    def test_xr_overlay_is_opt_in_and_does_not_change_default_pico_overlay(self):
        default = node._parse_args([])
        self.assertFalse(default.pico_overlay)
        self.assertFalse(default.xr_overlay)
        self.assertTrue(node._parse_args(['--xr-overlay']).xr_overlay)

    def test_official_hand_is_explicit_simulation_only_and_not_passive_overlay(self):
        self.assertTrue(hasattr(node, '_select_executor'))
        from tianji_teleop.executors.mujoco.authorized_hand import AuthorizedHandMujoco
        args = node._parse_args(['--official-hand-producer-instance', 'official-hand',
                                 '--hand-sides', 'left,right'])
        with patch.dict(os.environ, TIANJI_REQUIRED_CAPABILITY='simulation'):
            executor, options = node._select_executor(args)
            self.assertIs(executor, AuthorizedHandMujoco)
            self.assertEqual(options['hand_producer_id'], 'official_wuji_hand2')
            self.assertEqual(options['hand_producer_instance_id'], 'official-hand')
            args.hand_overlay = True
            with self.assertRaises(ValueError):
                node._select_executor(args)
        args.hand_overlay = False
        with patch.dict(os.environ, TIANJI_REQUIRED_CAPABILITY='real'):
            with self.assertRaises(ValueError):
                node._select_executor(args)
