"""Mapped-only display of consumed skeletons and native targets; no control writes."""
from threading import RLock

import numpy as np

from .spark_overlay import SparkOverlay
from ...hand_tracking.input_modes import MAPPED_PALM_BACKEND
from ...hand_tracking.reference_tjvr_receiver import ReceivedTjvrFrame


class MappedPalmOverlay(SparkOverlay):
    def __init__(self, receiver_instance_id):
        super().__init__(receiver_instance_id, algorithm=MAPPED_PALM_BACKEND)
        self._lock = RLock()
        self._active = False

    def ingest_raw(self, value):
        # The raw callback runs before stream/IK acceptance. It is not the
        # source of the reference Viewer's applied-skeleton layer.
        return False

    def ingest_cycle(self, native, attempt, *, execution_epoch, active):
        if type(execution_epoch) is not int or not 0 < execution_epoch < 2**63:
            return False
        if native is not None and 'target_height_offsets_m' in native:
            offsets = native['target_height_offsets_m']
            if (not isinstance(offsets, list) or len(offsets) != 2 or
                    any(type(v) not in (int,float) or not np.isfinite(v) or abs(v)>1 for v in offsets)):
                return False
        candidate = None
        if native is not None and attempt is not None:
            try:
                if (attempt['tick_id'] == native['tick_id'] and
                        attempt['timestamp_ns'] == native['timestamp_ns'] and
                        attempt['sample'] is not None):
                    candidate = ReceivedTjvrFrame.from_dict(attempt['sample']).observation
            except (ValueError, TypeError, KeyError):
                pass  # Bad diagnostics must not manufacture an applied skeleton.
        with self._lock:
            if execution_epoch < self._epoch:
                return False
            new_execution = execution_epoch > self._epoch
            if native is not None and not super().ingest_native(native, execution_epoch=execution_epoch):
                return False
            if new_execution:
                self._raw = None
                if native is None:
                    self._native = None
            self._epoch = execution_epoch
            self._active = bool(active)
            if not self._active:
                self._raw = None
                self._native = None
            elif candidate is not None:
                frame = candidate.frame
                if (frame.receiver_instance_id == self._receiver and
                        frame.sequence == native.get('applied_sequence') and
                        frame.tracking_epoch == native.get('applied_epoch')):
                    self._raw = candidate
            # A cached skeleton from a different source epoch is never drawn.
            if self._raw is not None and self._native is not None:
                frame = self._raw.frame
                if (frame.sequence != self._native.get('applied_sequence') or
                        frame.tracking_epoch != self._native.get('applied_epoch')):
                    self._raw = None
        return True

    def geometry(self, now_ns):
        with self._lock:
            if not self._active:
                return [], []
            markers, bones = super().geometry(now_ns)
            live = (self._native is not None and self._native.get('input_live') is True and
                    self._raw is not None and
                    0 <= now_ns - self._raw.frame.received_timestamp_ns <= 50_000_000 and
                    0 <= now_ns - self._native['timestamp_ns'] <= 50_000_000)
            result = []
            offsets = self._native.get('target_height_offsets_m') if self._native else None
            for marker in markers:
                if marker['label'].startswith('TJVR packet target'):
                    continue
                if marker['label'].startswith('TJVR corrected'):
                    if not live:
                        continue
                    marker['label'] = marker['label'].replace('TJVR corrected', 'Applied corrected', 1)
                    marker['rotation'] = None
                result.append(marker)
            if live and offsets is not None:
                # Match the reference Viewer's applied skeleton: its shoulder
                # anchor is not a calibrated end-effector target. Show the
                # optional Z offset separately, before native target processing.
                for side, index, offset in (('left', 3, offsets[0]), ('right', 7, offsets[1])):
                    position = self._raw.frame.upper_limb_points[index].copy()
                    position[2] += offset
                    result.append(dict(label=f'Z calibrated palm position {side}',
                        position=position, color=(.9, .2, .9, 1.), rotation=None))
            return result, bones if live else []

    def update_render_targets(self, model, render_data, mj, now_ns, *, fk_current=False):
        """Caller supplies the independent Viewer MjData, never executor data.

        Update XML mocap markers too, so no fixed default targets remain. Outside
        teleop (or when diagnostics expire), place them on the rendered TCPs.
        No qpos, model geometry, authority or native worker state is changed.
        """
        with self._lock:
            native = self._native if (self._active and self._native is not None and
                0 <= now_ns - self._native['timestamp_ns'] <= 200_000_000) else None
            if not fk_current:
                mj.mj_forward(model, render_data)
            for side, suffix in (('left', 'L'), ('right', 'R')):
                index = int(model.body('target_' + suffix).mocapid[0])
                if index < 0:
                    continue
                if native is not None:
                    position = native[side]['target_position']
                    xyzw = native[side]['target_quaternion_xyzw']
                    quaternion = np.array(xyzw)[[3, 0, 1, 2]]
                else:
                    site = render_data.site('hand_tcp_frame_' + suffix)
                    position = site.xpos
                    quaternion = np.empty(4)
                    mj.mju_mat2Quat(quaternion, site.xmat)
                render_data.mocap_pos[index] = position
                render_data.mocap_quat[index] = quaternion
            mj.mj_forward(model, render_data)
