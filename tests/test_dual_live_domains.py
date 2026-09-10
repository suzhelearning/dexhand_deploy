import importlib.util
import unittest


class DualLiveDomainsTest(unittest.TestCase):
    def guard(self):
        module = 'tianji_teleop.coordination.live_domain_guard'
        self.assertIsNotNone(importlib.util.find_spec(module))
        from tianji_teleop.coordination.live_domain_guard import LiveDomainGuard
        return LiveDomainGuard({'tj/live/source/tjvr/ours', 'tj/live/coordinator/arm/arm/coord'})

    def test_foreign_control_authority_latches_even_after_it_leaves(self):
        guard = self.guard()
        guard.observe('tj/live/source/tjvr/ours', present=True)
        guard.observe('tj/live/recorder/recorder/passive', present=True)
        self.assertIsNone(guard.failure)
        guard.observe('tj/live/producer/arm/old/foreign', present=True)
        self.assertIn('conflicting', guard.failure)
        guard.observe('tj/live/producer/arm/old/foreign', present=False)
        self.assertIsNotNone(guard.failure)

    def test_own_liveliness_loss_is_not_silently_recovered(self):
        guard = self.guard()
        guard.observe('tj/live/source/tjvr/ours', present=False)
        self.assertIn('lost', guard.failure)
        guard.observe('tj/live/source/tjvr/ours', present=True)
        self.assertIsNotNone(guard.failure)
