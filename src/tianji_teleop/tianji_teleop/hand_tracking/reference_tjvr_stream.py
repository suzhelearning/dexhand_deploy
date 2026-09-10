"""Reference TJVR stream acceptance policy, isolated from old receivers."""
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .models import LegacyPicoPalmFrame


@dataclass(frozen=True)
class StreamDecision:
    accepted: bool
    epoch_changed: bool = False
    reason: str = 'none'
    stream_discontinuity: bool = False


class ReferenceTjvrStreamGate:
    """PicoTeleopStreamGate at c022b177; call once per decoded packet.

    This is not a control tick or an authorization gate. In particular, rejected
    jump candidates still advance the observed sequence. An epoch transition
    does not imply an IK/OTG reset. No timestamp sorting or latest-frame
    coalescing may be performed ahead of this gate.
    """

    def __init__(self, max_position_jump_m: float, max_orientation_jump_rad: float):
        if any(not math.isfinite(v) or v <= 0 for v in
               (max_position_jump_m, max_orientation_jump_rad)):
            raise ValueError('TJVR jump thresholds must be finite and positive')
        self.max_position_jump_m = max_position_jump_m
        self.max_orientation_jump_rad = max_orientation_jump_rad
        self.reset()

    def reset(self) -> None:
        self._epoch = 0
        self._sequence = 0
        self._anchor = None
        self._candidate = None
        self._candidate_count = 0

    def _accept(self, frame, poses) -> None:
        self._epoch = frame.tracking_epoch
        self._sequence = frame.sequence
        self._anchor = poses
        self._candidate = None
        self._candidate_count = 0

    def _jump_reason(self, poses, previous) -> str:
        if any(np.linalg.norm(p[:3] - q[:3]) > self.max_position_jump_m
               for p, q in zip(poses, previous)):
            return 'position_jump'
        if any((Rotation.from_quat(p[3:]) * Rotation.from_quat(q[3:]).inv()).magnitude()
               > self.max_orientation_jump_rad for p, q in zip(poses, previous)):
            return 'orientation_jump'
        return 'none'

    def evaluate(self, frame: LegacyPicoPalmFrame) -> StreamDecision:
        if frame.tracking_epoch == 0:
            return StreamDecision(False, reason='zero_epoch')
        if self._anchor is not None and frame.tracking_epoch < self._epoch:
            return StreamDecision(False, reason='epoch_rollback')
        poses = (frame.left_pose.copy(), frame.right_pose.copy())
        if self._anchor is None or frame.tracking_epoch > self._epoch:
            if frame.sequence == 0:
                return StreamDecision(False, reason='out_of_order')
            self._accept(frame, poses)
            return StreamDecision(True, epoch_changed=True)
        if frame.sequence <= self._sequence:
            return StreamDecision(False, reason='out_of_order')
        self._sequence = frame.sequence
        reason = self._jump_reason(poses, self._anchor)
        if reason == 'none':
            self._accept(frame, poses)
            return StreamDecision(True)
        if self._candidate is None or self._jump_reason(poses, self._candidate) != 'none':
            self._candidate = poses
            self._candidate_count = 1
            return StreamDecision(False, reason=reason)
        self._candidate = poses
        self._candidate_count += 1
        if self._candidate_count < 3:
            return StreamDecision(False, reason=reason)
        self._accept(frame, poses)
        return StreamDecision(True, stream_discontinuity=True)
