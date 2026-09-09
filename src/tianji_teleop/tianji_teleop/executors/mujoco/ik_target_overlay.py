"""Read-only display of the actual Base_L/R IK input, not raw tracker poses."""
from pathlib import Path
from threading import Lock
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from ...protocol.messages import ArmTargetCommand


def fixed_base_transforms(urdf: Path) -> dict[str, np.ndarray]:
    """World-from-base for the fixed-base URDF loaded by this MuJoCo viewer.

    URDF fixed links can be fused away by MuJoCo; derive the exact fixed chain
    from the asset instead of depending on surviving body names or hardcoding
    shoulder offsets. Reject moving bases rather than showing a false frame.
    """
    root = ET.parse(urdf).getroot()
    links = {item.get('name') for item in root.findall('link')}
    parents = {j.find('child').get('link'): j for j in root.findall('joint')}
    roots = links - parents.keys()
    if len(roots) != 1:
        raise ValueError('IK overlay needs one fixed URDF root')
    result = {}
    for side, link in (('left', 'Base_L'), ('right', 'Base_R')):
        if link not in links:
            raise ValueError(f'IK overlay missing {link}')
        transform = np.eye(4)
        seen = set()
        while link in parents:
            if link in seen:
                raise ValueError('URDF fixed chain cycle')
            seen.add(link)
            joint = parents[link]
            if joint.get('type') != 'fixed':
                raise ValueError('IK overlay requires fixed Base_L/R')
            origin = joint.find('origin')
            step = np.eye(4)
            if origin is not None:
                xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
                if len(xyz) != 3 or len(rpy) != 3 or not np.isfinite(xyz + rpy).all():
                    raise ValueError('invalid URDF origin')
                step[:3, 3] = xyz
                step[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
            transform = step @ transform
            link = joint.find('parent').get('link')
        if link not in roots:
            raise ValueError('IK overlay base disconnected from root')
        result[side] = transform
    return result


class IkTargetOverlay:
    def __init__(self, urdf, router_zid, source_instance_id):
        if not source_instance_id:
            raise ValueError('IK target overlay requires source instance identity')
        self.bases = fixed_base_transforms(urdf)
        self.router = router_zid
        self.source = source_instance_id
        self._lock = Lock()
        self._targets = {}

    def ingest(self, payload, now_ns):
        target = ArmTargetCommand.from_dict(payload)
        envelope = target.envelope
        if envelope.router_zid != self.router or envelope.publisher_instance_id != self.source:
            return False
        if not 0 <= now_ns - envelope.timestamp_ns <= 500_000_000:
            return False
        with self._lock:
            previous = self._targets.get(target.side)
            if previous is not None and envelope.sequence <= previous.envelope.sequence:
                return False
            self._targets[target.side] = target
        return True

    def append(self, scene, mj, now_ns):
        with self._lock:
            targets = dict(self._targets)
        capacity = min(int(scene.maxgeom), len(scene.geoms))
        for side in ('left', 'right'):
            target = targets.get(side)
            if target is None:
                continue
            base = self.bases[side]
            position = base[:3, :3] @ target.position_m + base[:3, 3]
            rotation = base[:3, :3] @ Rotation.from_quat(target.orientation_xyzw).as_matrix()
            stale = not 0 <= now_ns - target.envelope.timestamp_ns <= 500_000_000
            if scene.ngeom >= capacity:
                return
            color = (0.6, 0.6, 0.6, 1) if stale else (
                (0, 1, 1, 1) if side == 'left' else (1, 0.5, 0, 1))
            geom = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_SPHERE, [0.018]*3,
                           position, np.eye(3).ravel(), color)
            geom.label = f'IK desired TCP {side}' + (' [stale]' if stale else '')
            scene.ngeom += 1
            for axis, rgba in enumerate(((1, 0, 0, 1), (0, 1, 0, 1), (0, 0.3, 1, 1))):
                if scene.ngeom >= capacity:
                    return
                geom = scene.geoms[scene.ngeom]
                mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                    np.zeros(3), np.eye(3).ravel(), color if stale else rgba)
                geom.label = ''
                mj.mjv_connector(geom, mj.mjtGeom.mjGEOM_CAPSULE, 0.004,
                                 position, position + rotation[:, axis] * 0.12)
                scene.ngeom += 1

    def diagnostics(self, now_ns):
        with self._lock:
            return {side: ({'state': 'waiting'} if side not in self._targets else {
                'state': 'live' if 0 <= now_ns - self._targets[side].envelope.timestamp_ns <= 500_000_000 else 'stale',
                'sequence': self._targets[side].envelope.sequence,
                'frame_id': self._targets[side].frame_id,
            }) for side in ('left', 'right')}
