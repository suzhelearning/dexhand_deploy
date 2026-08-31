"""H5 v4 wrist diagnostic overlay preparation。

该工具只读外部 acquisition v4 文件；可选 MuJoCo passive viewer 启动时显示
canonical Home，随后镜像 Marvin/Wuji 的权威关节反馈。它不会声明 Zenoh
publisher，不发布 SessionState、JointState 或 final command。
"""
from __future__ import annotations

import argparse
import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from ..coordination.arm_command_coordinator import ArmRobotConfig
from ..executors.mujoco.node import (
    _configure_viewer_platform,
    _frame_from_wrist_axis_geoms,
    _validate_model_joints,
)
from ..executors.wuji_hand2.config import WujiHandConfig
from ..protocol import topics
from ..protocol.messages import (
    ArmJointState,
    Frame0HandSkeleton,
    HandJointState,
    ProtocolError,
    strict_loads,
)
from ..sources.mocap.h5 import apply_yaw_world, load_mocap_h5
from ..zenoh_util import open_session, require_single_router
from tianji_world_output.config_loader import get_config
from tianji_world_output.transform_utils import get_chest_to_world_rotation


def _frame_from_axis_geoms(
    data: object,
    axis_x_geom_id: int,
    axis_z_geom_id: int,
    axis_half_length: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Recover a right-handed frame from MuJoCo X/Z axis geoms."""
    geom_xmat = np.asarray(data.geom_xmat)
    geom_xpos = np.asarray(data.geom_xpos)
    axis_x = geom_xmat[axis_x_geom_id].reshape(3, 3)[:, 2].copy()
    axis_z = geom_xmat[axis_z_geom_id].reshape(3, 3)[:, 2].copy()
    axis_x /= np.linalg.norm(axis_x)
    axis_z /= np.linalg.norm(axis_z)
    axis_y = np.cross(axis_z, axis_x)
    axis_y /= np.linalg.norm(axis_y)
    axis_z = np.cross(axis_x, axis_y)
    rotation = np.column_stack((axis_x, axis_y, axis_z))
    origin_x = geom_xpos[axis_x_geom_id] - axis_half_length * axis_x
    origin_z = geom_xpos[axis_z_geom_id] - axis_half_length * axis_z
    return 0.5 * (origin_x + origin_z), rotation


def _sim_from_motive_rotation(
    home_tcp_rotation_mj: np.ndarray,
    tianji_config: object | None = None,
) -> np.ndarray:
    """Return the configured fixed Motive-world to MuJoCo-world rotation."""
    config = tianji_config or get_config()
    tcp_rotation_chest = Rotation.from_quat(config.init_quat["right"]).as_matrix()
    tcp_rotation_world = get_chest_to_world_rotation("right") @ tcp_rotation_chest
    rotation_sim_from_world = home_tcp_rotation_mj @ tcp_rotation_world.T
    result = rotation_sim_from_world @ np.asarray(config.mocap_to_robot, dtype=np.float64)
    if not np.allclose(result @ result.T, np.eye(3), atol=1.0e-4) or not np.isclose(
        np.linalg.det(result), 1.0, atol=1.0e-4
    ):
        raise ValueError("Motive→MuJoCo 世界轴映射不是 det=+1 正交矩阵")
    return result


def _authority_instances(router_zid: str) -> tuple[str, str, str]:
    authorities = strict_loads(os.environ.get("TIANJI_AUTHORITIES", ""))
    source = authorities.get("source")
    arm = authorities.get("executor_arm")
    hands = authorities.get("executor_hand")
    hand = hands.get("right") if isinstance(hands, Mapping) else None
    if not all(isinstance(item, Mapping) for item in (source, arm, hand)):
        raise ValueError("H5 mirror requires source/arm/right-hand authorities")
    if (
        source.get("logical_id") != "h5_replay"
        or source.get("router_zid") != router_zid
        or arm.get("logical_id") != "marvin"
        or hand.get("logical_id") != "wuji_right"
        or arm.get("router_zid") != router_zid
        or hand.get("router_zid") != router_zid
    ):
        raise ValueError("H5 mirror authority mismatch")
    source_instance = str(source.get("publisher_instance_id", ""))
    arm_instance = str(arm.get("publisher_instance_id", ""))
    hand_instance = str(hand.get("publisher_instance_id", ""))
    instances = {source_instance, arm_instance, hand_instance}
    if not all(instances) or "disabled" in instances:
        raise ValueError("H5 mirror authorities are disabled")
    return source_instance, arm_instance, hand_instance


class RealStateMirror:
    """Read-only Marvin/Wuji state cache applied to MuJoCo qpos by the viewer."""

    def __init__(
        self,
        model: Any,
        data: Any,
        *,
        router_zid: str,
        arm_instance: str,
        hand_instance: str,
    ) -> None:
        self.data = data
        self.router_zid = router_zid
        self.arm_instance = arm_instance
        self.hand_instance = hand_instance
        self.robot = ArmRobotConfig.load()
        self.hand = WujiHandConfig.load()
        self._arm_addresses = {
            side: _validate_model_joints(
                model,
                tuple(getattr(self.robot, f"{side}_joint_names")),
                self.robot.lower_limits_rad,
                self.robot.upper_limits_rad,
            )
            for side in ("left", "right")
        }
        names = self.hand.sdk_joint_names(side="right")
        aliases = {
            name: name.replace("_mcp_", "_finger_mcp_")
            .replace("_pip", "_finger_pip")
            .replace("_dip", "_finger_dip")
            for name in names
            if "thumb_" not in name
        }
        self._hand_addresses = _validate_model_joints(
            model,
            names,
            self.hand.lower_limits_rad,
            self.hand.upper_limits_rad,
            aliases=aliases,
        )
        self._arm = np.asarray(self.robot.home_all, dtype=np.float64)
        self._hand = np.asarray(self.hand.zero_position_rad, dtype=np.float64)
        self._arm_sequence = -1
        self._hand_sequence = -1
        self._lock = threading.Lock()

    @staticmethod
    def _payload(sample: Any) -> Mapping[str, Any]:
        payload = getattr(sample, "payload", sample)
        return payload if isinstance(payload, Mapping) else strict_loads(bytes(payload))

    def on_arm_state(self, sample: Any) -> bool:
        try:
            state = ArmJointState.from_dict(self._payload(sample))
            if (
                state.executor != "marvin"
                or state.router_zid != self.router_zid
                or state.publisher_instance_id != self.arm_instance
            ):
                return False
            values = np.asarray(state.position_rad, dtype=np.float64)
            lower = np.asarray(self.robot.lower_limits_rad * 2)
            upper = np.asarray(self.robot.upper_limits_rad * 2)
            if np.any(values < lower) or np.any(values > upper):
                return False
            with self._lock:
                if state.sequence <= self._arm_sequence:
                    return False
                self._arm = values
                self._arm_sequence = state.sequence
            return True
        except (ProtocolError, TypeError, ValueError):
            return False

    def on_hand_state(self, sample: Any) -> bool:
        try:
            state = HandJointState.from_dict(self._payload(sample))
            if (
                state.executor != "wuji_hand2"
                or state.side != "right"
                or state.router_zid != self.router_zid
                or state.publisher_instance_id != self.hand_instance
            ):
                return False
            values = np.asarray(self.hand.validate_positions(state.position_rad))
            with self._lock:
                if state.sequence <= self._hand_sequence:
                    return False
                self._hand = values
                self._hand_sequence = state.sequence
            return True
        except (ProtocolError, TypeError, ValueError):
            return False

    def apply(self) -> None:
        with self._lock:
            arm = self._arm.copy()
            hand = self._hand.copy()
        index = 0
        for side in ("left", "right"):
            for name in getattr(self.robot, f"{side}_joint_names"):
                self.data.qpos[self._arm_addresses[side][name]] = arm[index]
                index += 1
        for name, value in zip(self._hand_addresses, hand):
            self.data.qpos[self._hand_addresses[name]] = value


class ExpectedH5Overlay:
    """Render the source-authorized H5 wrist path and current hand skeleton."""

    _POINT_SIZE = np.asarray([0.009, 0.0, 0.0], dtype=np.float64)
    _WRIST_SIZE = np.asarray([0.016, 0.0, 0.0], dtype=np.float64)
    _EMPTY_SIZE = np.zeros(3, dtype=np.float64)
    _IDENTITY = np.eye(3, dtype=np.float64).reshape(-1)
    _PATH_RGBA = np.asarray([1.0, 0.82, 0.08, 0.72], dtype=np.float32)
    _WRIST_RGBA = np.asarray([0.2, 1.0, 0.35, 1.0], dtype=np.float32)
    _POINT_RGBA = np.asarray([1.0, 0.35, 0.08, 0.96], dtype=np.float32)
    _EDGE_RGBA = np.asarray([0.1, 0.75, 1.0, 0.82], dtype=np.float32)

    def __init__(
        self,
        model: Any,
        data: Any,
        recording: Any,
        *,
        router_zid: str,
        source_instance: str,
        yaw_deg: float = 0.0,
    ) -> None:
        self.router_zid = router_zid
        self.source_instance = source_instance
        self._home_position, self._home_rotation = (
            _frame_from_wrist_axis_geoms(model, data)
        )
        right = recording.hands["right"]
        self._path_motive = apply_yaw_world(
            right.wrist[np.asarray(right.valid, dtype=bool)], yaw_deg
        )[:, :3]
        self._sequence = -1
        self._path_mujoco: np.ndarray | None = None
        self._points_mujoco: np.ndarray | None = None
        self._wrist_mujoco: np.ndarray | None = None
        self._edges: np.ndarray | None = None
        self._lock = threading.Lock()

    def on_skeleton(self, sample: Any) -> bool:
        try:
            skeleton = Frame0HandSkeleton.from_dict(
                RealStateMirror._payload(sample)
            )
            if (
                skeleton.side != "right"
                or skeleton.router_zid != self.router_zid
                or skeleton.publisher_instance_id != self.source_instance
            ):
                return False
            home = np.asarray(skeleton.robot_wrist_home_pose, dtype=np.float64)
            rotation = self._home_rotation @ Rotation.from_quat(
                home[3:]
            ).as_matrix().T
            translation = self._home_position - rotation @ home[:3]
            points = (
                np.asarray(skeleton.keypoints_world_m) @ rotation.T
                + translation
            )
            path = self._path_motive @ rotation.T + translation
            wrist = rotation @ np.asarray(skeleton.target_wrist_pose[:3]) + translation
            edges = np.asarray(skeleton.edges, dtype=np.int32)
            if not all(np.isfinite(item).all() for item in (points, path, wrist)):
                raise ProtocolError("expected H5 overlay contains non-finite geometry")
            with self._lock:
                if skeleton.sequence <= self._sequence:
                    return False
                self._path_mujoco = path
                self._points_mujoco = points
                self._wrist_mujoco = wrist
                self._edges = edges
                self._sequence = skeleton.sequence
            return True
        except (ProtocolError, TypeError, ValueError):
            return False

    def snapshot(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
        with self._lock:
            if self._path_mujoco is None:
                return None
            return (
                self._path_mujoco.copy(),
                self._points_mujoco.copy(),
                self._wrist_mujoco.copy(),
                self._edges.copy(),
            )

    def draw(self, scene: Any, mujoco_module: Any) -> None:
        geometry = self.snapshot()
        scene.ngeom = 0
        if geometry is None:
            return
        path, points, wrist, edges = geometry
        capacity = min(int(scene.maxgeom), len(scene.geoms))
        reserve = 1 + len(points) + len(edges)
        path_capacity = max(0, capacity - reserve)
        if len(path) > path_capacity + 1:
            path = path[
                np.linspace(0, len(path) - 1, path_capacity + 1).astype(int)
            ]
        for start, end in zip(path[:-1], path[1:]):
            if scene.ngeom >= capacity:
                break
            geom = scene.geoms[scene.ngeom]
            mujoco_module.mjv_initGeom(
                geom, mujoco_module.mjtGeom.mjGEOM_CAPSULE,
                self._EMPTY_SIZE, self._EMPTY_SIZE, self._IDENTITY,
                self._PATH_RGBA,
            )
            mujoco_module.mjv_connector(
                geom, mujoco_module.mjtGeom.mjGEOM_CAPSULE,
                0.0025, start, end,
            )
            scene.ngeom += 1
        for point in points:
            if scene.ngeom >= capacity:
                break
            mujoco_module.mjv_initGeom(
                scene.geoms[scene.ngeom], mujoco_module.mjtGeom.mjGEOM_SPHERE,
                self._POINT_SIZE, point, self._IDENTITY, self._POINT_RGBA,
            )
            scene.ngeom += 1
        for parent, child in edges:
            if scene.ngeom >= capacity:
                break
            start, end = points[int(parent)], points[int(child)]
            if float(np.linalg.norm(end - start)) < 1.0e-8:
                continue
            geom = scene.geoms[scene.ngeom]
            mujoco_module.mjv_initGeom(
                geom, mujoco_module.mjtGeom.mjGEOM_CAPSULE,
                self._EMPTY_SIZE, self._EMPTY_SIZE, self._IDENTITY,
                self._EDGE_RGBA,
            )
            mujoco_module.mjv_connector(
                geom, mujoco_module.mjtGeom.mjGEOM_CAPSULE,
                0.0035, start, end,
            )
            scene.ngeom += 1
        if scene.ngeom < capacity:
            mujoco_module.mjv_initGeom(
                scene.geoms[scene.ngeom], mujoco_module.mjtGeom.mjGEOM_SPHERE,
                self._WRIST_SIZE, wrist, self._IDENTITY, self._WRIST_RGBA,
            )
            scene.ngeom += 1


def _run_viewer(recording) -> None:
    import mujoco
    _configure_viewer_platform()
    import mujoco.viewer
    from ..mujoco_urdf import portable_mujoco_urdf

    root = Path(__file__).resolve().parents[4]
    urdf = root / "src" / "tianji_teleop" / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
    xml, assets = portable_mujoco_urdf(urdf)
    model = mujoco.MjModel.from_xml_string(xml, assets)
    data = mujoco.MjData(model)
    session = open_session()
    subscriptions = []
    try:
        router_zid = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID"))
        source_instance, arm_instance, hand_instance = _authority_instances(
            router_zid
        )
        mirror = RealStateMirror(
            model,
            data,
            router_zid=router_zid,
            arm_instance=arm_instance,
            hand_instance=hand_instance,
        )
        mirror.apply()
        mujoco.mj_forward(model, data)
        expected = ExpectedH5Overlay(
            model,
            data,
            recording,
            router_zid=router_zid,
            source_instance=source_instance,
            yaw_deg=float(os.environ.get("TIANJI_REAL_YAW_DEG", "0")),
        )
        subscriptions.extend(
            (
                session.declare_subscriber(topics.ARM_STATE, mirror.on_arm_state),
                session.declare_subscriber(topics.hand_state("right"), mirror.on_hand_state),
                session.declare_subscriber(
                    topics.FRAME0_HAND_SKELETON, expected.on_skeleton
                ),
            )
        )
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                with viewer.lock():
                    mirror.apply()
                    mujoco.mj_forward(model, data)
                    expected.draw(viewer.user_scn, mujoco)
                viewer.sync()
                time.sleep(1.0 / max(float(recording.output_hz), 1.0))
    finally:
        for subscription in subscriptions:
            try:
                subscription.undeclare()
            except Exception:
                pass
        session.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="read-only H5 wrist MuJoCo diagnostic")
    parser.add_argument("h5", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--viewer", action="store_true", help="open passive MuJoCo overlay")
    args = parser.parse_args(argv)
    recording = load_mocap_h5(args.h5)
    summary = recording.summary()
    summary.update({"overlay": "real_state_and_h5_expected", "executor_authority": None, "viewer": bool(args.viewer)})
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.viewer:
        _run_viewer(recording)
    return 0


__all__ = ["RealStateMirror", "main"]

if __name__ == "__main__":
    raise SystemExit(main())
