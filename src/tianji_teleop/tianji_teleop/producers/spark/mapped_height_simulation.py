"""Opt-in mapped-palm calibration. Original SPARK/PICO2 owners are unchanged."""
from types import SimpleNamespace

from .live_simulation import SparkLiveSimulation
from .live_control import SparkControlLoop
from ...hand_tracking.input_modes import MAPPED_PALM_BACKEND
from ...hand_tracking.mapped_palm_height import MappedPalmHeight, horizontal_tcp_heights


class MappedHeightSimulation(SparkLiveSimulation):
    def __init__(self, *args, **kwargs):
        if kwargs.get('backend') != MAPPED_PALM_BACKEND:
            raise ValueError('height calibration is mapped-palm only')
        super().__init__(*args, **kwargs)
        try:
            self.height = MappedPalmHeight(horizontal_tcp_heights(self.sim.model))
            self.height_events = []
            self.height_pending = False
        except BaseException:
            self.close()
            raise

    def request(self, action):
        if action == 'calibrate':
            if self.height.state == 'collecting':
                return SimpleNamespace(accepted=False, reason='height calibration already collecting')
            try:
                if self._closed or self._failure:
                    raise ValueError('height calibration requires healthy idle/Home')
                epoch = self.producer.guard.execution_epoch + 1
                self.coordinator.validate_bilateral_home_rearm(epoch)
                self.sim.validate_bilateral_home_rearm(epoch)
            except ValueError as exc:
                return SimpleNamespace(accepted=False, reason=str(exc))
            self.height.begin(self.clock())
            return SimpleNamespace(accepted=True, reason='hold both arms horizontal and steady for 2 seconds')
        if action == 'start' and (not self.height.ready or self.height_pending):
            return SimpleNamespace(accepted=False, reason='press c and complete height calibration before s')
        if action in ('return', 'shutdown') and self.height.state == 'collecting':
            self.height.sampler.fail('calibration cancelled by operator')
        return super().request(action)

    def step(self, sample=None, *, source_failure=None):
        previous = self.height.state
        if previous == 'collecting':
            if source_failure or self.coordinator.state.state != 'idle':
                self.height.sampler.fail('session/input unhealthy during calibration')
            elif self.height.update(sample, self.clock()):
                self.height_pending = True
            if self.height.state != previous and not self.height_pending:
                self.height_events.append(dict(kind='mapped_palm_height_calibration',
                    execution_epoch=self.producer.guard.execution_epoch,
                    **self.height.status()))
        return super().step(sample, source_failure=source_failure)

    def finish_height_calibration(self):
        if not self.height_pending:
            return False
        epoch = self.producer.guard.execution_epoch + 1
        self.coordinator.validate_bilateral_home_rearm(epoch)
        self.sim.validate_bilateral_home_rearm(epoch)
        # Initial idle has not executed any native tick and is not paused yet.
        # Pause only inside this validated barrier, never during sampling ticks.
        self.producer.guard.pause('explicit height calibration Home reset')
        ack = self.rearm_at_home()
        self.producer.backend.configure_height(self.height.offsets)
        self.height_pending = False
        # Must precede the first snapshot of the new epoch in the HDF5 journal.
        self.height_events.append(dict(kind='mapped_palm_height_calibration',
            action='rearm', accepted=True, reset_ack=ack,
            execution_epoch=ack['execution_epoch'], **self.height.status()))
        return True


class MappedHeightControlLoop(SparkControlLoop):
    _ACTIONS = SparkControlLoop._ACTIONS | {'calibrate'}

    def _deadline_after_barrier(self, deadline):
        # Called only after a successful reset/calibration barrier. Never
        # carry reconstruction-time idle debt into the new execution epoch.
        # A queued start may already have entered teleop: preserve its schedule.
        if self._core.coordinator.state.state == 'idle':
            return self._clock() + self._period_s
        return super()._deadline_after_barrier(deadline)

    def _process_actions(self):
        applied = self._core.finish_height_calibration()
        for event in self._core.height_events:
            self._emit_report(event)
        self._core.height_events.clear()
        return super()._process_actions() or applied
