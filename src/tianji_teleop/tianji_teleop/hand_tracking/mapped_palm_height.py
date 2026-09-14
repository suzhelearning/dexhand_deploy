"""Session-local, bilateral Z calibration for mapped corrected palms only."""
from types import SimpleNamespace

import numpy as np

from .height_calibration import HeightCalibration


def horizontal_tcp_heights(model):
    import mujoco
    data = mujoco.MjData(model)
    for side in ('L', 'R'):
        for j in range(1, 8):
            data.qpos[int(model.joint(f'Joint{j}_{side}').qposadr[0])] = 0.
    mujoco.mj_forward(model, data)
    return {side: float(data.site('hand_tcp_frame_' + suffix).xpos[2])
            for side, suffix in (('left','L'), ('right','R'))}


class MappedPalmHeight:
    def __init__(self, reference):
        self.reference = dict(reference)
        if set(reference) != {'left','right'} or not np.isfinite(list(reference.values())).all():
            raise ValueError('finite bilateral horizontal TCP heights required')
        self.sampler = HeightCalibration(('left','right'))
        self.offsets = None
        self._identity = None
        self._sequence = 0

    @property
    def state(self):
        return self.sampler.state

    @property
    def ready(self):
        return self.offsets is not None and self.state != 'collecting'

    def begin(self, now):
        self._identity, self._sequence = None, 0
        self.sampler.begin(now)

    def update(self, sample, now):
        if self.state != 'collecting':
            return False
        if sample is not None:
            frame = sample.observation.frame
            identity = (frame.receiver_instance_id, frame.tracking_epoch)
            if self._identity is not None and identity != self._identity:
                self.sampler.fail('tracking identity/epoch changed during calibration')
                return False
            self._identity = identity
            if frame.sequence > self._sequence:
                self._sequence = frame.sequence
                for side, i in (('left',3), ('right',7)):
                    self.sampler.add(SimpleNamespace(side=side,
                        valid=frame.upper_limb_skeleton_valid and frame.upper_limb_rotations_valid,
                        pose=frame.upper_limb_points[i],
                        received_timestamp_ns=frame.received_timestamp_ns), now)
        means = self.sampler.tick(now)
        if means is None:
            return False
        offsets = [self.reference[s] - means[s] for s in ('left','right')]
        if not np.isfinite(offsets).all() or max(map(abs,offsets)) > 1.:
            self.sampler.fail('height offset exceeds 1 m; check pose and calibration')
            return False
        self.offsets = offsets
        self.sampler.accept(means)
        return True

    def status(self):
        return dict(self.sampler.status(), mapping='mapped_palm_height',
                    reference_height_m=self.reference,
                    source_identity=(dict(receiver_instance_id=self._identity[0], tracking_epoch=self._identity[1])
                                     if self._identity else None),
                    last_source_sequence=self._sequence,
                    started_timestamp_ns=getattr(self.sampler, 'started', None),
                    target_height_offsets_m=self.offsets)
