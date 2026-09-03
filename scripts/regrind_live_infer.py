#!/usr/bin/env python3
"""Run CPU Regrind inference from live Motive wrist and hammer poses only."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from tianji_teleop.diagnostics.mujoco_h5_wrist_replay import (
    RealStateMirror,
    _authority_instances,
)
from tianji_teleop.coordination.arm_command_coordinator import ArmRobotConfig
from tianji_teleop.executors.mujoco.node import (
    _configure_viewer_platform,
    _frame_from_wrist_axis_geoms,
)
from tianji_teleop.executors.wuji_hand2.config import WujiHandConfig
from tianji_teleop.joint_state_model import urdf_joint_names
from tianji_teleop.mujoco_urdf import portable_mujoco_urdf
from tianji_teleop.protocol import topics
from tianji_teleop.protocol.messages import ComponentStatus, ProtocolError
from tianji_teleop.regrind_policy import action_to_targets, build_observation, infer, load_actor, load_reference
from tianji_teleop.sources.mocap.h5 import compose_pose, invert_pose
from tianji_teleop.sources.mocap.regrind import RegrindMotiveTracker
from tianji_teleop.zenoh_util import open_session, require_single_router


_HAMMER_START_POSITION_TOLERANCE_M = 0.02
_HAMMER_START_ORIENTATION_TOLERANCE_DEG = 10.0


def _reference_speed(value: str) -> float:
    speed = float(value)
    if not 0.0 < speed <= 1.0:
        raise argparse.ArgumentTypeError(
            "--reference-speed must be finite and in (0, 1]"
        )
    return speed


def _hammer_pose_is_aligned(
    position_error_m: float,
    orientation_error_deg: float,
) -> bool:
    return (
        position_error_m <= _HAMMER_START_POSITION_TOLERANCE_M
        and orientation_error_deg <= _HAMMER_START_ORIENTATION_TOLERANCE_DEG
    )


def _alignment_scene_poses(
    scene_wrist: np.ndarray,
    live_home_wrist: np.ndarray,
    reference_wrist: np.ndarray,
    reference_hammer: np.ndarray,
    live_wrist: np.ndarray,
    live_hammer: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    scene_from_training = compose_pose(scene_wrist, invert_pose(live_home_wrist))
    return (
        compose_pose(scene_from_training, reference_wrist),
        compose_pose(scene_from_training, reference_hammer),
        compose_pose(scene_from_training, live_wrist),
        compose_pose(scene_from_training, live_hammer),
    )


def _build_alignment_model(reference_path: Path, mujoco_module):
    root_dir = Path(__file__).resolve().parents[1]
    urdf = root_dir / "src/tianji_teleop/assets/tianji_wuji2/tianji_wuji2.urdf"
    hammer_obj = reference_path.parent.parent / "objects/hammer/visual.obj"
    if not hammer_obj.is_file():
        raise RuntimeError(f"hammer OBJ not found beside reference package: {hammer_obj}")
    xml, assets = portable_mujoco_urdf(urdf)
    root = ET.fromstring(xml)
    asset_name = "regrind_hammer.obj"
    assets[asset_name] = hammer_obj.read_bytes()
    for name, rgba in (
        ("expected_hammer", "0.1 1 0.2 0.25"),
        ("live_hammer", "1 0.35 0.05 0.9"),
    ):
        link = ET.SubElement(root, "link", {"name": name})
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass", {"value": "0.0458"})
        ET.SubElement(inertial, "inertia", {
            "ixx": "1.4601e-05", "ixy": "0", "ixz": "0",
            "iyy": "0.000213526", "iyz": "0", "izz": "0.000224736",
        })
        visual = ET.SubElement(link, "visual")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", {"filename": asset_name})
        material = ET.SubElement(visual, "material", {"name": name})
        ET.SubElement(material, "color", {"rgba": rgba})
        joint = ET.SubElement(root, "joint", {"name": f"{name}_free", "type": "floating"})
        ET.SubElement(joint, "parent", {"link": "Link_Base"})
        ET.SubElement(joint, "child", {"link": name})
    return mujoco_module.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"), assets)


class _ExpectedHandOverlay:
    """Reference Wuji2 hand geometry expressed at the expected wrist pose."""

    _RGBA = np.asarray([0.1, 1.0, 0.2, 0.32], dtype=np.float32)

    def __init__(self, model, qpos, home_wrist, joints, mujoco_module) -> None:
        self._model = model
        self._data = mujoco_module.MjData(model)
        self._data.qpos[:] = qpos
        self._hand = WujiHandConfig.load()
        joint_frames = np.asarray(joints, dtype=np.float64)
        if joint_frames.shape == (20,):
            joint_frames = joint_frames[None, :]
        if joint_frames.ndim != 2 or joint_frames.shape[1] != 20:
            raise ValueError("reference joints must have shape [frames,20]")
        self._joint_frames = joint_frames
        self._addresses = []
        for name in self._hand.sdk_joint_names(side="right"):
            model_name = (
                name.replace("_mcp_", "_finger_mcp_")
                .replace("_pip", "_finger_pip")
                .replace("_dip", "_finger_dip")
                if "thumb_" not in name else name
            )
            joint_id = mujoco_module.mj_name2id(
                model, mujoco_module.mjtObj.mjOBJ_JOINT, model_name
            )
            self._addresses.append(int(model.jnt_qposadr[joint_id]))
        self._wrist_from_world = invert_pose(home_wrist)
        mujoco_module.mj_forward(model, self._data)
        native_scene = mujoco_module.MjvScene(model, model.ngeom)
        mujoco_module.mjv_updateScene(
            model, self._data, mujoco_module.MjvOption(), None,
            mujoco_module.MjvCamera(), mujoco_module.mjtCatBit.mjCAT_ALL,
            native_scene,
        )
        render_data_ids = {
            int(geom.objid): int(geom.dataid)
            for geom in native_scene.geoms[:native_scene.ngeom]
            if int(geom.objtype) == int(mujoco_module.mjtObj.mjOBJ_GEOM)
        }
        self._geom_ids = []
        self._geoms = []
        for geom_id in range(model.ngeom):
            mesh_id = int(model.geom_dataid[geom_id])
            mesh_name = (
                mujoco_module.mj_id2name(
                    model, mujoco_module.mjtObj.mjOBJ_MESH, mesh_id
                )
                if mesh_id >= 0 else None
            )
            if (
                int(model.geom_group[geom_id]) != 1
                or not (mesh_name or "").startswith("wuji2_r_")
                or mesh_name == "wuji2_r_mount"
            ):
                continue
            self._geom_ids.append(geom_id)
            self._geoms.append((
                int(model.geom_type[geom_id]),
                model.geom_size[geom_id].copy(),
                render_data_ids[geom_id],
            ))
        self._frame_index = -1
        self._frame_geoms = []

    def _select_frame(self, frame_index, mujoco_module) -> None:
        index = min(max(int(frame_index), 0), len(self._joint_frames) - 1)
        if index == self._frame_index:
            return
        values = self._hand.validate_positions(
            self._joint_frames[index], field=f"reference joints[{index}]"
        )
        for address, value in zip(self._addresses, values):
            self._data.qpos[address] = value
        mujoco_module.mj_forward(self._model, self._data)
        self._frame_geoms = [
            compose_pose(
                self._wrist_from_world,
                np.concatenate((
                    self._data.geom_xpos[geom_id],
                    Rotation.from_matrix(
                        self._data.geom_xmat[geom_id].reshape(3, 3)
                    ).as_quat(),
                )),
            )
            for geom_id in self._geom_ids
        ]
        self._frame_index = index

    def draw(self, scene, mujoco_module, expected_wrist, frame_index=0) -> None:
        self._select_frame(frame_index, mujoco_module)
        for (geom_type, size, data_id), wrist_from_geom in zip(
            self._geoms, self._frame_geoms
        ):
            if scene.ngeom >= scene.maxgeom:
                break
            pose = compose_pose(expected_wrist, wrist_from_geom)
            geom = scene.geoms[scene.ngeom]
            mujoco_module.mjv_initGeom(
                geom,
                geom_type,
                size,
                pose[:3],
                Rotation.from_quat(pose[3:]).as_matrix().reshape(-1),
                self._RGBA,
            )
            geom.dataid = data_id
            geom.category = mujoco_module.mjtCatBit.mjCAT_DECOR
            geom.transparent = 1
            scene.ngeom += 1


class _PolicyFrameTracker:
    def __init__(
        self, *, router_zid: str, source_instance: str,
        frame_count: int, start_frame: int = 0,
    ) -> None:
        self._router_zid = router_zid
        self._source_instance = source_instance
        self._frame_count = int(frame_count)
        self._start_frame = int(start_frame)
        self._sequence = -1
        self._current = ("waiting", self._start_frame)

    def on_status(self, sample) -> bool:
        try:
            status = ComponentStatus.from_dict(RealStateMirror._payload(sample))
            frame_index = status.diagnostics.get("frame_index")
            if (
                status.component_role != "source"
                or status.component_id != "regrind_policy"
                or status.router_zid != self._router_zid
                or status.publisher_instance_id != self._source_instance
                or isinstance(frame_index, bool)
                or not isinstance(frame_index, int)
                or not 0 <= frame_index <= self._frame_count
                or status.sequence <= self._sequence
            ):
                return False
        except (ProtocolError, TypeError, ValueError):
            return False
        self._sequence = status.sequence
        reference_index = (
            min(max(frame_index - 1, self._start_frame), self._frame_count - 1)
            if status.phase == "running" else self._start_frame
        )
        self._current = (status.phase, reference_index)
        return True

    def current(self) -> tuple[str, int]:
        return self._current


def _run_alignment_viewer(
    reference_path: Path,
    reference,
    live: RegrindMotiveTracker,
    stale_s: float,
    session,
    start_frame: int,
    hold_enter: bool = False,
) -> bool:
    import mujoco
    import mujoco.viewer

    model = _build_alignment_model(reference_path, mujoco)
    data = mujoco.MjData(model)
    router_zid = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID"))
    source_instance, arm_instance, hand_instance = _authority_instances(
        router_zid, source_logical_id="regrind_policy"
    )
    mirror = RealStateMirror(
        model,
        data,
        router_zid=router_zid,
        arm_instance=arm_instance,
        hand_instance=hand_instance,
    )
    mirror.apply()
    free_qpos = {
        name: int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{name}_free")])
        for name in ("expected_hammer", "live_hammer")
    }
    mujoco.mj_forward(model, data)
    home_position, home_rotation = _frame_from_wrist_axis_geoms(model, data)
    home_wrist = np.concatenate((home_position, Rotation.from_matrix(home_rotation).as_quat()))

    reference_wrist = np.concatenate((reference.wrist_pos[start_frame], np.roll(reference.wrist_quat_wxyz[start_frame], -1)))
    reference_hammer = np.concatenate((reference.object_pos[start_frame], np.roll(reference.object_quat_wxyz[start_frame], -1)))
    initial = live.latest()
    if initial is None:
        raise RuntimeError("lost Motive wrist+hammer before opening viewer")
    live_home_wrist = initial.wrist_xyzw.copy()
    expected_wrist, expected_hammer, current_wrist, current_hammer = _alignment_scene_poses(
        home_wrist,
        live_home_wrist,
        reference_wrist,
        reference_hammer,
        initial.wrist_xyzw,
        initial.hammer_xyzw,
    )

    def set_pose(name: str, pose: np.ndarray) -> None:
        address = free_qpos[name]
        data.qpos[address:address + 3] = pose[:3]
        data.qpos[address + 3:address + 7] = np.roll(pose[3:], 1)

    set_pose("expected_hammer", expected_hammer)
    set_pose("live_hammer", current_hammer)
    mujoco.mj_forward(model, data)
    expected_hand = _ExpectedHandOverlay(
        model, data.qpos, home_wrist, reference.joints, mujoco
    )
    policy_frame = _PolicyFrameTracker(
        router_zid=router_zid,
        source_instance=source_instance,
        frame_count=reference.frame_count,
        start_frame=start_frame,
    )
    subscriptions = (
        session.declare_subscriber(topics.ARM_STATE, mirror.on_arm_state),
        session.declare_subscriber(topics.hand_state("right"), mirror.on_hand_state),
        session.declare_subscriber(topics.SOURCE_STATUS, policy_frame.on_status),
    )
    last_passed = False
    try:
        with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
            viewer.cam.lookat[:] = np.mean(
                np.stack((current_wrist[:3], current_hammer[:3], expected_hammer[:3])), axis=0
            )
            viewer.cam.distance = max(
                1.2,
                2.2 * max(
                    np.linalg.norm(current_hammer[:3] - expected_hammer[:3]),
                    np.linalg.norm(current_hammer[:3] - current_wrist[:3]),
                ),
            )
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -25.0
            while viewer.is_running():
                sample = live.latest()
                age_s = float("inf") if sample is None else time.monotonic() - sample.received_at
                if sample is not None:
                    policy_phase, reference_index = policy_frame.current()
                    reference_wrist = np.concatenate((
                        reference.wrist_pos[reference_index],
                        np.roll(reference.wrist_quat_wxyz[reference_index], -1),
                    ))
                    reference_hammer = np.concatenate((
                        reference.object_pos[reference_index],
                        np.roll(reference.object_quat_wxyz[reference_index], -1),
                    ))
                    expected_wrist, expected_hammer, current_wrist, current_hammer = _alignment_scene_poses(
                        home_wrist,
                        live_home_wrist,
                        reference_wrist,
                        reference_hammer,
                        sample.wrist_xyzw,
                        sample.hammer_xyzw,
                    )
                    delta_mm = 1000.0 * (expected_hammer[:3] - current_hammer[:3])
                    position_error_mm = float(np.linalg.norm(delta_mm))
                    orientation_error_deg = float(np.rad2deg(np.linalg.norm((
                        Rotation.from_quat(expected_hammer[3:]).inv()
                        * Rotation.from_quat(current_hammer[3:])
                    ).as_rotvec())))
                    fresh = age_s <= stale_s
                    last_passed = fresh and _hammer_pose_is_aligned(
                        position_error_mm / 1000.0,
                        orientation_error_deg,
                    )
                    with viewer.lock():
                        mirror.apply()
                        set_pose("expected_hammer", expected_hammer)
                        set_pose("live_hammer", current_hammer)
                        mujoco.mj_forward(model, data)
                        scene = viewer.user_scn
                        scene.ngeom = 0
                        arrows = []
                        if position_error_mm > 0.1:
                            arrows.append((
                                current_hammer[:3], expected_hammer[:3], 0.008,
                                np.asarray([1.0, 0.9, 0.05, 1.0], dtype=np.float32),
                            ))
                        for start, end, width, color in arrows[: int(scene.maxgeom)]:
                            geom = scene.geoms[scene.ngeom]
                            mujoco.mjv_initGeom(
                                geom, mujoco.mjtGeom.mjGEOM_ARROW,
                                np.zeros(3), np.zeros(3), np.eye(3).reshape(-1),
                                color,
                            )
                            mujoco.mjv_connector(
                                geom, mujoco.mjtGeom.mjGEOM_ARROW, width, start, end,
                            )
                            scene.ngeom += 1
                        expected_hand.draw(
                            scene, mujoco, expected_wrist, reference_index
                        )
                    status = "PASS" if last_passed else ("STALE" if not fresh else "ADJUST")
                    viewer.set_texts((
                        mujoco.mjtFontScale.mjFONTSCALE_150,
                        mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        "READ-ONLY  Solid robot: LIVE state\n"
                        "Orange hammer: LIVE  Green hammer/hand: EXPECTED reference\n"
                        f"Control terminal: s + "
                        f"{'hold Enter' if hold_enter else 'Enter once'} -> frame {start_frame}\n"
                        + (
                            "Then release, align hammer, press i; hold Enter -> infer\n"
                            if hold_enter
                            else "Then align hammer, press i; Enter once -> infer, again -> pause\n"
                        )
                        + "Yellow arrow: move live hammer toward target\n"
                        "Robot/Zenoh: +X forward(red), +Y left(green), +Z up(blue)\n"
                        "Correction in robot world XYZ (mm)\n"
                        "Position / orientation error\n"
                        "Motive age\nPolicy phase / reference frame\nStatus",
                        f"\n\n\n\n\n\n[{delta_mm[0]:+7.1f}, {delta_mm[1]:+7.1f}, {delta_mm[2]:+7.1f}]\n"
                        f"{position_error_mm:7.1f} mm / {orientation_error_deg:6.1f} deg\n"
                        f"{age_s * 1000.0:7.1f} ms\n"
                        f"{policy_phase} / {reference_index}\n{status}",
                    ))
                else:
                    viewer.set_texts((
                        mujoco.mjtFontScale.mjFONTSCALE_150,
                        mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        "READ-ONLY Regrind alignment",
                        live.error or "Waiting for valid Motive wrist + hammer",
                    ))
                viewer.sync()
                time.sleep(1.0 / 30.0)
    finally:
        for subscription in subscriptions:
            try:
                subscription.undeclare()
            except Exception:
                pass
    return last_passed


def _run_reference_hand_replay(
    reference_path: Path,
    reference,
    rate_hz: float,
    live: RegrindMotiveTracker,
    stale_s: float,
    start_frame: int,
) -> int:
    import mujoco

    _configure_viewer_platform()
    import mujoco.viewer

    model = _build_alignment_model(reference_path, mujoco)
    data = mujoco.MjData(model)
    for name, value in zip(urdf_joint_names(), ArmRobotConfig.load().home_all):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint_id]] = value
    mujoco.mj_forward(model, data)
    position, rotation = _frame_from_wrist_axis_geoms(model, data)
    home_wrist = np.concatenate((position, Rotation.from_matrix(rotation).as_quat()))
    hand = _ExpectedHandOverlay(
        model, data.qpos, home_wrist, reference.joints, mujoco
    )
    hand._RGBA = np.asarray([0.1, 1.0, 0.2, 0.9], dtype=np.float32)
    expected_hammer_qpos = int(model.jnt_qposadr[
        mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "expected_hammer_free"
        )
    ])
    hammer_body_ids = set()
    for name, alpha in (("expected_hammer", 0.9), ("live_hammer", 0.0)):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        hammer_body_ids.add(body_id)
        model.geom_rgba[np.asarray(model.geom_bodyid) == body_id, 3] = alpha
    robot_geom_ids = np.asarray([
        geom_id for geom_id in range(model.ngeom)
        if int(model.geom_bodyid[geom_id]) not in hammer_body_ids
    ], dtype=np.int32)
    sample = live.latest()
    if sample is None:
        raise RuntimeError("lost Motive wrist+hammer before opening viewer")
    current_arm_wrist = sample.wrist_xyzw.copy()
    center = np.mean(
        np.stack((current_arm_wrist[:3], reference.wrist_pos[start_frame], reference.object_pos[start_frame])),
        axis=0,
    )
    started = time.monotonic()
    with mujoco.viewer.launch_passive(
        model, data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        viewer.cam.lookat[:] = center
        viewer.cam.distance = max(
            1.2,
            2.2 * max(
                np.linalg.norm(current_arm_wrist[:3] - reference.wrist_pos[start_frame]),
                np.linalg.norm(current_arm_wrist[:3] - reference.object_pos[start_frame]),
            ),
        )
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -25.0
        while viewer.is_running():
            sample = live.latest()
            if sample is not None:
                current_arm_wrist = sample.wrist_xyzw
            age_s = (
                float("inf") if sample is None
                else time.monotonic() - sample.received_at
            )
            motive_from_scene = compose_pose(
                current_arm_wrist, invert_pose(home_wrist)
            )
            motive_rotation = Rotation.from_quat(
                motive_from_scene[3:]
            ).as_matrix()
            index = start_frame + (
                int((time.monotonic() - started) * rate_hz)
                % (reference.frame_count - start_frame)
            )
            wrist = np.concatenate((
                reference.wrist_pos[index], np.roll(reference.wrist_quat_wxyz[index], -1),
            ))
            hammer = np.concatenate((
                reference.object_pos[index], np.roll(reference.object_quat_wxyz[index], -1),
            ))
            with viewer.lock():
                data.qpos[expected_hammer_qpos:expected_hammer_qpos + 3] = hammer[:3]
                data.qpos[expected_hammer_qpos + 3:expected_hammer_qpos + 7] = np.roll(hammer[3:], 1)
                mujoco.mj_forward(model, data)
                data.geom_xpos[robot_geom_ids] = (
                    data.geom_xpos[robot_geom_ids] @ motive_rotation.T
                    + motive_from_scene[:3]
                )
                data.geom_xmat[robot_geom_ids] = (
                    motive_rotation
                    @ data.geom_xmat[robot_geom_ids].reshape(-1, 3, 3)
                ).reshape(-1, 9)
                viewer.user_scn.ngeom = 0
                hand.draw(viewer.user_scn, mujoco, wrist, index)
            viewer.set_texts((
                mujoco.mjtFontScale.mjFONTSCALE_150,
                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                "READ-ONLY Regrind reference replay\n"
                "Motive stream world: +X forward, +Y left, +Z up\n"
                "Solid Tianji arm: Home joints, LIVE Motive tianji_wrist pose\n"
                "Green hand/hammer: current H5 frame\n"
                "Motive wrist age / status\n"
                "Close the window to exit",
                f"frame {index + 1}/{reference.frame_count}  {rate_hz:g} Hz\n"
                f"{age_s * 1000.0:.1f} ms / "
                f"{'LIVE' if age_s <= stale_s else 'STALE'}",
            ))
            viewer.sync()
            time.sleep(1.0 / 60.0)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--endpoint", default=os.environ.get("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447"))
    parser.add_argument("--wrist-name", default="tianji_wrist")
    parser.add_argument("--hammer-name", default="hammer")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--reference-speed",
        type=_reference_speed,
        default=1.0,
        help="reference playback ratio in (0, 1]; accepted by the read-only viewer for shared session arguments",
    )
    parser.add_argument("--stale-s", type=float, default=0.25)
    parser.add_argument("--wait-s", type=float, default=10.0)
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument(
        "--hold-enter",
        action="store_true",
        help="viewer text uses the hold-Enter inference mode",
    )
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--viewer", action="store_true", help="open read-only live/expected frame-0 alignment viewer")
    parser.add_argument("--hand-replay", action="store_true", help="replay only the Regrind Wuji hand in MuJoCo")
    args = parser.parse_args()
    if not args.reference.is_file():
        parser.error("--reference must be an existing file")
    if not args.hand_replay and (args.model is None or not args.model.is_file()):
        parser.error("--model must be an existing file")
    if args.rate <= 0.0 or args.stale_s <= 0.0 or args.wait_s <= 0.0 or args.print_every < 1:
        parser.error("rate/timeouts/print-every must be positive")
    if args.viewer and args.preflight_only:
        parser.error("--viewer and --preflight-only are mutually exclusive")

    reference = load_reference(args.reference)
    if not 0 <= args.start_frame < reference.frame_count - 1:
        parser.error(f"--start-frame must be in [0, {reference.frame_count - 2}]")
    if args.hand_replay:
        if args.viewer or args.preflight_only:
            parser.error("--hand-replay cannot be combined with --viewer or --preflight-only")
    if args.viewer or args.hand_replay:
        actor = mean = variance = None
        iteration = None
    else:
        torch.set_num_threads(1)
        actor, mean, variance, iteration = load_actor(args.model)
    session = open_session(args.endpoint)
    live = RegrindMotiveTracker(session, wrist_name=args.wrist_name, hammer_name=args.hammer_name)
    try:
        deadline = time.monotonic() + args.wait_s
        sample = live.latest()
        while sample is None and time.monotonic() < deadline:
            time.sleep(0.01)
            sample = live.latest()
        if sample is None:
            raise RuntimeError("timed out waiting for valid Motive wrist+hammer poses")

        received_at, wrist_zero, hammer_zero = sample.received_at, sample.wrist_xyzw, sample.hammer_xyzw
        if time.monotonic() - received_at > args.stale_s:
            raise RuntimeError("initial Motive frame is stale")
        if args.hand_replay:
            return _run_reference_hand_replay(
                args.reference, reference, args.rate, live, args.stale_s,
                args.start_frame,
            )
        reference_wrist_zero = np.concatenate((reference.wrist_pos[args.start_frame], np.roll(reference.wrist_quat_wxyz[args.start_frame], -1)))
        reference_hammer_zero = np.concatenate((reference.object_pos[args.start_frame], np.roll(reference.object_quat_wxyz[args.start_frame], -1)))
        training_from_motive = np.asarray(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64
        )
        aligned_hammer_zero = compose_pose(training_from_motive, hammer_zero)
        hammer_position_error_m = float(np.linalg.norm(aligned_hammer_zero[:3] - reference_hammer_zero[:3]))
        hammer_rotation_error_deg = float(np.rad2deg(np.linalg.norm((
            Rotation.from_quat(reference_hammer_zero[3:]).inv()
            * Rotation.from_quat(aligned_hammer_zero[3:])
        ).as_rotvec())))
        pose_preflight_passed = _hammer_pose_is_aligned(
            hammer_position_error_m,
            hammer_rotation_error_deg,
        )
        previous_wrist = compose_pose(training_from_motive, wrist_zero)
        previous_wrist_pos = previous_wrist[:3].copy()
        previous_wrist_quat = np.roll(previous_wrist[3:], 1)
        joints = reference.joints[args.start_frame].copy()
        previous_joints = joints.copy()
        last_action = np.zeros(26, dtype=np.float64)
        next_tick = time.monotonic()

        print(json.dumps({
            "event": "started",
            "mode": "live_motive_alignment_viewer" if args.viewer else "live_motive_shadow_inference",
            "publishes_control": False,
            "checkpoint_iteration": iteration,
            "rate_hz": args.rate,
            "reference_speed": args.reference_speed,
            "frames": reference.frame_count - args.start_frame - 1,
            "start_frame": args.start_frame,
            "hand_joint_observation": "previous_policy_target_assuming_perfect_tracking",
            "hammer_start_position_error_mm": round(hammer_position_error_m * 1000.0, 3),
            "hammer_start_orientation_error_deg": round(hammer_rotation_error_deg, 3),
            "real_start_preflight_passed": pose_preflight_passed,
        }, separators=(",", ":")), flush=True)
        if args.preflight_only:
            return 0 if pose_preflight_passed else 1
        if args.viewer:
            passed = _run_alignment_viewer(
                args.reference, reference, live, args.stale_s, session,
                args.start_frame, args.hold_enter,
            )
            print(json.dumps({"event": "viewer_closed", "real_start_preflight_passed": passed}), flush=True)
            return 0

        for index in range(args.start_frame, reference.frame_count - 1):
            next_tick += 1.0 / args.rate
            sample = live.latest()
            if sample is None:
                raise RuntimeError("Motive wrist or hammer tracking invalid")
            motive_frame = sample.frame_number
            received_at, wrist_live, hammer_live = sample.received_at, sample.wrist_xyzw, sample.hammer_xyzw
            age_s = time.monotonic() - received_at
            if age_s > args.stale_s:
                raise RuntimeError(f"Motive frame stale: {age_s:.3f}s")
            wrist = compose_pose(training_from_motive, wrist_live)
            hammer = compose_pose(training_from_motive, hammer_live)
            wrist_quat_wxyz = np.roll(wrist[3:], 1)
            hammer_quat_wxyz = np.roll(hammer[3:], 1)
            observation = build_observation(
                object_pos=hammer[:3],
                object_quat_wxyz=hammer_quat_wxyz,
                previous_wrist_pos=previous_wrist_pos,
                wrist_pos=wrist[:3],
                previous_wrist_quat_wxyz=previous_wrist_quat,
                wrist_quat_wxyz=wrist_quat_wxyz,
                previous_joints=previous_joints,
                joints=joints,
                last_action=last_action,
                phase=index / (reference.frame_count - 1),
                base_wrist_pos=reference.wrist_pos[index],
                base_wrist_quat_wxyz=reference.wrist_quat_wxyz[index],
                base_joints=reference.joints[index],
            )
            started_ns = time.perf_counter_ns()
            raw_action = infer(actor, mean, variance, observation)
            inference_ms = (time.perf_counter_ns() - started_ns) / 1e6
            target_pos, target_quat, target_joints = action_to_targets(
                raw_action,
                reference.wrist_pos[index],
                reference.wrist_quat_wxyz[index],
                reference.joints[index],
            )
            if index % args.print_every == 0:
                print(json.dumps({
                    "frame": index,
                    "motive_frame": motive_frame,
                    "motive_age_ms": round(age_s * 1000.0, 3),
                    "inference_ms": round(inference_ms, 3),
                    "wrist_pos": wrist[:3].tolist(),
                    "wrist_quat_wxyz": wrist_quat_wxyz.tolist(),
                    "hammer_pos": hammer[:3].tolist(),
                    "hammer_quat_wxyz": hammer_quat_wxyz.tolist(),
                    "raw_action": raw_action.tolist(),
                    "target_wrist_pos": target_pos.tolist(),
                    "target_wrist_quat_wxyz": target_quat.tolist(),
                    "target_joints": target_joints.tolist(),
                }, separators=(",", ":")), flush=True)
            previous_wrist_pos, previous_wrist_quat = wrist[:3].copy(), wrist_quat_wxyz.copy()
            previous_joints, joints = joints, target_joints
            last_action = np.clip(raw_action, -1.0, 1.0)
            time.sleep(max(0.0, next_tick - time.monotonic()))
        print(json.dumps({
            "event": "completed",
            "frames": reference.frame_count - args.start_frame - 1,
        }), flush=True)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        live.close()
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
