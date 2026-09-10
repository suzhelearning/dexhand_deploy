"""Passive new-route diagnostics in the reference model's world frame.

TJVR corrected points follow the original Viewer convention directly. Native
control.target comes from world-frame MuJoCo TCP kinematics; neither gets an
extra Base_L/R transform. Packet targets are a separate diagnostic, not IK input.
"""
from copy import deepcopy
from threading import Lock

import numpy as np
from scipy.spatial.transform import Rotation

from ...hand_tracking.input_modes import SPARK_BACKEND
from ...hand_tracking.reference_tjvr import ReferenceTjvrFrame


class SparkOverlay:
    def __init__(self, receiver_instance_id):
        if not isinstance(receiver_instance_id, str) or not receiver_instance_id:
            raise ValueError('overlay receiver identity required')
        self._receiver = receiver_instance_id
        self._lock = Lock()
        self._raw = None
        self._native = None
        self._epoch = 0

    def ingest_raw(self, value):
        if not isinstance(value, ReferenceTjvrFrame) or value.frame.receiver_instance_id != self._receiver:
            return False
        with self._lock:
            if self._raw and value.frame.receiver_frame_sequence <= self._raw.frame.receiver_frame_sequence:
                return False
            self._raw = deepcopy(value)
        return True

    def ingest_native(self, value, *, execution_epoch):
        try:
            if (type(execution_epoch) is not int or not 0 < execution_epoch < 2**63 or
                    value['algorithm'] != SPARK_BACKEND or type(value['tick_id']) is not int or
                    value['tick_id'] <= 0 or type(value['timestamp_ns']) is not int or value['timestamp_ns'] <= 0):
                return False
            for side in ('left', 'right'):
                position = np.asarray(value[side]['target_position'])
                quat = np.asarray(value[side]['target_quaternion_xyzw'])
                if (position.shape != (3,) or quat.shape != (4,) or not np.isfinite(position).all() or
                        not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1.) > 1e-3):
                    return False
        except (TypeError, ValueError, KeyError):
            return False
        with self._lock:
            if execution_epoch < self._epoch or (execution_epoch == self._epoch and self._native and
                                                value['tick_id'] <= self._native['tick_id']):
                return False
            self._native, self._epoch = deepcopy(value), execution_epoch
        return True

    def geometry(self, now_ns):
        with self._lock:
            raw, native = deepcopy(self._raw), deepcopy(self._native)
        markers, bones = [], []
        def marker(label, position, color, quat=None):
            markers.append(dict(label=label, position=np.asarray(position), color=color,
                rotation=Rotation.from_quat(quat).as_matrix() if quat is not None else None))
        if raw and 0 <= now_ns - raw.frame.received_timestamp_ns <= 200_000_000:
            frame = raw.frame
            if frame.upper_limb_skeleton_valid:
                points = frame.upper_limb_points
                for start, end in ((0, 4), (0, 1), (1, 2), (2, 3), (4, 5), (5, 6), (6, 7)):
                    bones.append((points[start], points[end]))
                for index, point in enumerate(points):
                    side = 'left' if index < 4 else 'right'
                    name = ('shoulder', 'elbow', 'wrist', 'palm')[index % 4]
                    quat = frame.upper_limb_rotations_xyzw[index] if index in (3, 7) and frame.upper_limb_rotations_valid else None
                    marker(f'TJVR corrected {name} {side}', point, (.1, .8, 1., .8), quat)
            for side in ('left', 'right'):
                pose = getattr(frame, side + '_pose')
                marker(f'TJVR packet target {side}', pose[:3], (.9, .2, .9, .8), pose[3:])
        if native:
            stale = not 0 <= now_ns - native['timestamp_ns'] <= 200_000_000
            for side in ('left', 'right'):
                arm = native[side]
                marker(f'SPARK IK target {side}' + (' [stale]' if stale else ''), arm['target_position'],
                       (.5, .5, .5, 1.) if stale else (1., .75, .05, 1.), arm['target_quaternion_xyzw'])
        return markers, bones

    def append(self, scene, mj, now_ns):
        markers, bones = self.geometry(now_ns)
        capacity = min(int(scene.maxgeom), len(scene.geoms))
        def geom(kind, position, color, label=''):
            if scene.ngeom >= capacity:
                return None
            result = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(result, kind, [.013] * 3, position, np.eye(3).ravel(), color)
            result.label = label
            scene.ngeom += 1
            return result
        for start, end in bones:
            item = geom(mj.mjtGeom.mjGEOM_CAPSULE, start, (.1, .8, 1., .65))
            if item is None:
                return
            mj.mjv_connector(item, mj.mjtGeom.mjGEOM_CAPSULE, .004, start, end)
        for marker in markers:
            position = marker['position']
            if geom(mj.mjtGeom.mjGEOM_SPHERE, position, marker['color'], marker['label']) is None:
                return
            if marker['rotation'] is not None:
                for axis, color in enumerate(((1., 0., 0., 1.), (0., 1., 0., 1.), (0., .3, 1., 1.))):
                    item = geom(mj.mjtGeom.mjGEOM_CAPSULE, position, color)
                    if item is None:
                        return
                    mj.mjv_connector(item, mj.mjtGeom.mjGEOM_CAPSULE, .003, position,
                                     position + .1 * marker['rotation'][:, axis])
