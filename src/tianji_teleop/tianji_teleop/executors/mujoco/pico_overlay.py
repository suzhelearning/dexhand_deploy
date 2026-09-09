"""Passive raw tracking-space visualization; never produces robot commands."""
from threading import Lock
from typing import Any, Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from ...hand_tracking.models import PicoRawFrame


DISPLAY_OFFSET_M = np.array([0.0, 1.2, 0.8])
STALE_NS = 500_000_000
# Wrist -> palm and five anatomical chains, including metacarpals.
EDGES = ((1, 0),) + tuple(
    edge
    for chain in ((1, 2, 3, 4, 5), (1, 6, 7, 8, 9, 10),
                  (1, 11, 12, 13, 14, 15), (1, 16, 17, 18, 19, 20),
                  (1, 21, 22, 23, 24, 25))
    for edge in zip(chain, chain[1:])
)


class PicoRawOverlay:
    def __init__(self, router_zid: str):
        self.router_zid = router_zid
        self._lock = Lock()
        self._frame: PicoRawFrame | None = None
        self._error: str | None = None
        self._count = 0

    def ingest(self, payload: Mapping[str, Any], now_ns: int) -> bool:
        try:
            value = dict(payload)
            if value.pop('router_zid', None) != self.router_zid:
                raise ValueError('raw PICO router mismatch')
            frame = PicoRawFrame.from_dict(value)
            if not 0 <= now_ns - frame.received_timestamp_ns <= STALE_NS:
                raise ValueError('raw PICO timestamp stale or future')
            with self._lock:
                previous = self._frame
                if previous is not None:
                    if frame.receiver_instance_id != previous.receiver_instance_id:
                        raise ValueError('raw PICO receiver mismatch')
                    if frame.received_timestamp_ns < previous.received_timestamp_ns:
                        raise ValueError('raw PICO timestamp rollback')
                    old_order = (previous.connection_generation, previous.receiver_frame_sequence)
                    new_order = (frame.connection_generation, frame.receiver_frame_sequence)
                    if new_order <= old_order and now_ns - previous.received_timestamp_ns <= STALE_NS:
                        raise ValueError('raw PICO frame out of order')
                self._frame = frame
                self._count += 1
                self._error = None
            return True
        except (ValueError, TypeError, KeyError) as exc:
            self.reject(str(exc))
            return False

    def reject(self, error: str) -> None:
        with self._lock:
            self._error = error

    def snapshot(self, now_ns: int):
        with self._lock:
            frame = self._frame
            state = ('waiting' if frame is None else 'live'
                     if 0 <= now_ns - frame.received_timestamp_ns <= STALE_NS else 'stale')
            diagnostics = dict(state=state, frames_received=self._count, error=self._error,
                               association_id=None if frame is None else frame.association_id)
            return frame if state == 'live' else None, diagnostics

    def append(self, scene: Any, mj: Any, now_ns: int) -> None:
        """Append under the caller's viewer lock; caller owns scene clearing."""
        frame, diagnostics = self.snapshot(now_ns)
        capacity = min(int(scene.maxgeom), len(scene.geoms))

        def point(position, color, label='', radius=0.005):
            if scene.ngeom >= capacity:
                return
            geom = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_SPHERE, [radius] * 3,
                           position, np.eye(3).ravel(), color)
            geom.label = label
            scene.ngeom += 1

        def line(start, end, color):
            if scene.ngeom >= capacity or np.linalg.norm(end - start) < 1e-8:
                return
            geom = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                           np.zeros(3), np.eye(3).ravel(), color)
            geom.label = ''
            mj.mjv_connector(geom, mj.mjtGeom.mjGEOM_CAPSULE, 0.002, start, end)
            scene.ngeom += 1

        def pose(value, color, label):
            position = value[:3] + DISPLAY_OFFSET_M
            point(position, color, label, 0.012)
            if np.linalg.norm(value[3:]) < 1e-12:
                return  # Never invent axes for an invalid quaternion.
            rotation = Rotation.from_quat(value[3:]).as_matrix()
            for axis, rgba in enumerate(((1, 0, 0, 1), (0, 1, 0, 1), (0, 0.3, 1, 1))):
                line(position, position + rotation[:, axis] * 0.08, rgba)

        label = 'PICO raw FLU ' + diagnostics['state']
        if diagnostics['error']:
            label += ' / rejected frame'
        # The device starts with its head at the tracking origin. Keep the
        # reference label above it so head and status text do not overlap.
        point(DISPLAY_OFFSET_M + [0, 0, 0.2], (0.8, 0.8, 0.8, 1), label, 0.003)
        if frame is None:
            return
        for axis, rgba in enumerate(((1, 0, 0, 1), (0, 1, 0, 1), (0, 0.3, 1, 1))):
            line(DISPLAY_OFFSET_M, DISPLAY_OFFSET_M + np.eye(3)[axis] * 0.08, rgba)
        if frame.head_valid:
            pose(frame.head_pose, (0.9, 0.9, 0.9, 1), 'PICO head')
        for side, color in (('left', (0, 0.8, 1, 1)), ('right', (1, 0.5, 0, 1))):
            hand = frame.hands[side]
            if hand.wrist_valid:
                pose(hand.wrist_pose, color, 'PICO ' + side + ' wrist')
            if not hand.valid:
                continue
            for joint in hand.joints:
                if joint.valid:
                    point(joint.pose[:3] + DISPLAY_OFFSET_M, color)
            for parent, child in EDGES:
                a, b = hand.joints[parent], hand.joints[child]
                if a.valid and b.valid:
                    line(a.pose[:3] + DISPLAY_OFFSET_M, b.pose[:3] + DISPLAY_OFFSET_M, color)
