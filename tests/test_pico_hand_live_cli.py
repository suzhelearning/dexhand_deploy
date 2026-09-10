import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PicoHandLiveCliTest(unittest.TestCase):
    def module(self):
        path = ROOT / 'scripts/pico_hand_live.py'
        self.assertTrue(path.is_file(), 'managed PICO hand entry required')
        spec = importlib.util.spec_from_file_location('pico_hand_live', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def environment(self):
        def row(logical, instance):
            return dict(logical_id=logical, publisher_instance_id=instance, router_zid='router')
        return dict(TIANJI_PICO_HAND_MANAGED='1', TIANJI_REQUIRED_CAPABILITY='simulation',
            TIANJI_HAND_PRODUCER_INSTANCE_ID='hand', TIANJI_ROUTER_ZID='router',
            TIANJI_COORDINATOR_INSTANCE_ID='coord', TIANJI_OBSERVATION_PUBLISHER_INSTANCE_ID='pico',
            TIANJI_AUTHORITIES=json.dumps(dict(source=row('hand_tracking_target', 'source'),
                producer_arm=row('arm_ik_producer', 'ik'), producer_hand=row('official_wuji_hand2', 'hand'),
                coordinator_arm=row('arm', 'coord'), executor_arm=row('mujoco', 'sim'),
                executor_hand={side: row('wuji_' + side, 'sim') for side in ('left', 'right')})))

    def test_managed_identity_mapping_and_generation_are_explicit(self):
        result = self.module().bindings(self.environment())
        self.assertEqual(result['receiver_instance_id'], 'pico')
        self.assertEqual(result['connection_generation'], 1)
        self.assertIn('tj/live/executor/hand/wuji_left/sim', result['expected_tokens'])
        self.assertIn('tj/live/producer/hand/official_wuji_hand2/hand', result['expected_tokens'])

    def test_unmanaged_real_and_wrong_router_fail_before_opening_resources(self):
        module = self.module()
        for key, value in (('TIANJI_PICO_HAND_MANAGED', '0'), ('TIANJI_REQUIRED_CAPABILITY', 'real'),
                           ('TIANJI_ROUTER_ZID', 'different')):
            with self.subTest(key=key), self.assertRaises(ValueError):
                module.bindings(dict(self.environment(), **{key: value}))
