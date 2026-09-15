import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

from tests.test_pico_hand_service import Session
from tests.test_pico_official_backend import SideClient


class ManagedSession(Session):
    def __init__(self):
        super().__init__()
        self.tokens = set()
        self.live_callback = None
        self.inventory = []

    def liveliness(self):
        return self

    def declare_token(self, key):
        self.tokens.add(key)
        return SimpleNamespace(undeclare=lambda: self.tokens.remove(key))

    def declare_subscriber(self, key, callback, **kwargs):
        if key == 'tj/live/**':
            self.live_callback = callback
            return SimpleNamespace(undeclare=lambda: setattr(self, 'live_callback', None))
        return super().declare_subscriber(key, callback)

    def get(self, key, **kwargs):
        return [SimpleNamespace(ok=True, result=SimpleNamespace(key_expr=token))
                for token in self.inventory]


class PicoHandComponentTest(unittest.TestCase):
    def make(self, *, fail_side=None, foreign=False, capability='simulation', hand_worker_backend=None):
        module = 'tianji_teleop.producers.pico_hand_component'
        self.assertIsNotNone(importlib.util.find_spec(module))
        from tianji_teleop.producers.pico_hand_component import PicoHandComponent
        self.session = ManagedSession()
        self.clients = []
        self.client_options = []
        def factory(**kwargs):
            side = kwargs['single_hand_side']
            if side == fail_side:
                raise RuntimeError('worker startup failed')
            self.client_options.append(kwargs)
            client = SideClient(side)
            client.closed = False
            client.close = lambda: setattr(client, 'closed', True)
            self.clients.append(client)
            return client
        own = 'tj/live/producer/hand/official_wuji_hand2/hand'
        coord = 'tj/live/coordinator/arm/arm/coord'
        if foreign:
            self.session.inventory.append('tj/live/producer/hand/old/foreign')
        component = PicoHandComponent(self.session, root=Path(__file__).resolve().parents[1],
            publisher_instance_id='hand', router_zid='router', coordinator_instance_id='coord',
            receiver_instance_id='pico', connection_generation=1, expected_tokens={own, coord},
            required_capability=capability, client_factory=factory,
            hand_worker_backend=hand_worker_backend)
        self.addCleanup(component.close)
        return component

    def test_owns_two_workers_subscriptions_and_one_hand_token(self):
        component = self.make()
        self.assertEqual(len(self.clients), 2)
        self.assertEqual(self.session.tokens, {'tj/live/producer/hand/official_wuji_hand2/hand'})
        self.assertIsNone(component.failure)
        self.assertFalse(component.tick().ready)
        component.close()
        component.close()
        self.assertTrue(all(c.closed for c in self.clients))
        self.assertFalse(self.session.tokens)
        self.assertFalse(self.session.callbacks)
        self.assertIsNone(self.session.live_callback)

    def test_explicit_cpp_worker_backend_is_forwarded_to_both_side_workers(self):
        self.make(hand_worker_backend='cpp')
        self.assertEqual(len(self.clients), 2)
        self.assertEqual([item['worker_backend'] for item in self.client_options], ['cpp', 'cpp'])

    def test_second_worker_failure_cleans_first_worker(self):
        with self.assertRaisesRegex(RuntimeError, 'startup failed'):
            self.make(fail_side='right')
        self.assertTrue(self.clients[0].closed)
        self.assertFalse(self.session.tokens)
        self.assertFalse(self.session.callbacks)

    def test_foreign_authority_fails_before_worker_startup(self):
        with self.assertRaisesRegex(RuntimeError, 'conflicting'):
            self.make(foreign=True)
        self.assertEqual(self.clients, [])
        self.assertFalse(self.session.tokens)
        self.assertIsNone(self.session.live_callback)

    def test_real_rejected_before_resources(self):
        with self.assertRaisesRegex(ValueError, 'simulation'):
            self.make(capability='real')
        self.assertEqual(self.clients, [])
        self.assertFalse(self.session.tokens)

    def test_live_coordinator_loss_latches_without_worker_restart(self):
        import zenoh
        component = self.make()
        callback = self.session.live_callback
        token = 'tj/live/coordinator/arm/arm/coord'
        callback(SimpleNamespace(key_expr=token, kind=zenoh.SampleKind.DELETE))
        self.assertIn('authority lost', component.failure)
        self.assertFalse(component.tick().healthy)
        callback(SimpleNamespace(key_expr=token, kind=zenoh.SampleKind.PUT))
        self.assertIn('authority lost', component.failure)
        self.assertEqual(len(self.clients), 2)
