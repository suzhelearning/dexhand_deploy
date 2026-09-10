import importlib.util
import unittest


class SparkExecutionStateTest(unittest.TestCase):
    def make(self, **overrides):
        name = 'tianji_teleop.producers.spark.execution'
        self.assertIsNotNone(importlib.util.find_spec('tianji_teleop.producers.spark'), 'missing SPARK producer package')
        self.assertIsNotNone(importlib.util.find_spec(name), 'missing execution supervision')
        from tianji_teleop.producers.spark.execution import ExecutionGuard
        config = dict(run_id='run', execution_epoch=1, coordinator_instance_id='coord',
                      router_zid='router', maximum_receipt_age_ns=100, max_in_flight=2)
        config.update(overrides)
        return ExecutionGuard(**config)

    def q(self, value=0.):
        return dict(left=[value] * 7, right=[value] * 7)

    def receipt(self, tick=1, accepted=True, q=None):
        return dict(schema_version=1, kind='arm_bilateral_receipt', run_id='run', execution_epoch=1,
            tick_id=tick, timestamp_ns=120, publisher_instance_id='coord', router_zid='router',
            stage='coordinator_command', accepted=accepted, reason='test', command_position_rad=q or self.q())

    def test_direct_receipt_is_associated_and_does_not_claim_feedback(self):
        guard = self.make()
        guard.register(1, 100, self.q())
        self.assertTrue(guard.observe(self.receipt(), 120))
        self.assertFalse(guard.paused)
        self.assertEqual(guard.in_flight, 0)
        self.assertFalse(guard.observe(self.receipt(), 121))

    def test_timeout_latches_and_does_not_resume_on_late_ack(self):
        guard = self.make()
        guard.register(1, 100, self.q())
        self.assertFalse(guard.check(201))
        self.assertTrue(guard.paused)
        self.assertFalse(guard.observe(self.receipt(), 202))
        with self.assertRaises(RuntimeError):
            guard.register(2, 203, self.q())

    def test_direct_rewrite_or_rejection_pauses_both(self):
        for row in (self.receipt(accepted=False), self.receipt(q=self.q(.001))):
            guard = self.make()
            guard.register(1, 100, self.q())
            self.assertFalse(guard.observe(row, 120))
            self.assertTrue(guard.paused)

    def test_foreign_receipt_cannot_clear_inflight(self):
        for key, value in (('run_id', 'old'), ('execution_epoch', 2),
                           ('publisher_instance_id', 'other'), ('router_zid', 'other'), ('tick_id', 9)):
            guard = self.make()
            guard.register(1, 100, self.q())
            row = self.receipt()
            row[key] = value
            self.assertFalse(guard.observe(row, 120))
            self.assertEqual(guard.in_flight, 1)

    def test_inflight_is_bounded(self):
        guard = self.make()
        guard.register(1, 100, self.q())
        guard.register(2, 105, self.q())
        with self.assertRaises(RuntimeError):
            guard.register(3, 110, self.q())
        self.assertTrue(guard.paused)

    def test_processed_mode_without_explicit_limits_is_rejected(self):
        with self.assertRaises(ValueError):
            self.make(mode='processed_guarded')
