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
from tianji_teleop.executors.mujoco.node import _frame_from_wrist_axis_geoms
from tianji_teleop.executors.wuji_hand2.config import WujiHandConfig
from tianji_teleop.mujoco_urdf import portable_mujoco_urdf
from tianji_teleop.protocol import topics
from tianji_teleop.regrind_policy import action_to_targets, build_observation, infer, load_actor, load_reference
from tianji_teleop.sources.mocap.h5 import compose_pose, invert_pose
from tianji_teleop.sources.mocap.regrind import RegrindMotiveTracker
from tianji_teleop.zenoh_util import open_session, require_single_router


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
    """Frame0 Wuji2 hand geometry expressed at the expected wrist pose."""

    _RGBA = np.asarray([0.1, 1.0, 0.2, 0.32], dtype=np.float32)

    def __init__(self, model, qpos, home_wrist, joints, mujoco_module) -> None:
        data = mujoco_module.MjData(model)
        data.qpos[:] = qpos
        hand = WujiHandConfig.load()
        values = hand.validate_positions(joints, field="reference frame0 joints")
        for name, value in zip(hand.sdk_joint_names(side="right"), values):
            model_name = (
                name.replace("_mcp_", "_finger_mcp_")
                .replace("_pip", "_finger_pip")
                .replace("_dip", "_finger_dip")
                if "thumb_" not in name else name
            )
            joint_id = mujoco_module.mj_name2id(
                model, mujoco_module.mjtObj.mjOBJ_JOINT, model_name
            )
            data.qpos[model.jnt_qposadr[joint_id]] = value
        mujoco_module.mj_forward(model, data)
        wrist_from_world = invert_pose(home_wrist)
        self._geoms = []
        for geom_id in range(model.ngeom):
            mesh_id = int(model.geom_dataid[geom_id])
            mesh_name = (
                mujoco_module.mj_id2name(
                    model, mujoco_module.mjtObj.mjOBJ_MESH, mesh_id
                )
                if mesh_id >= 0 else None
            )
            if int(model.geom_group[geom_id]) != 1 or not (mesh_name or "").startswith("wuji2_r_"):
                continue
            pose = np.concatenate((
                data.geom_xpos[geom_id],
                Rotation.from_matrix(data.geom_xmat[geom_id].reshape(3, 3)).as_quat(),
            ))
            self._geoms.append((
                int(model.geom_type[geom_id]),
                model.geom_size[geom_id].copy(),
                mesh_id,
                compose_pose(wrist_from_world, pose),
            ))

    def draw(self, scene, mujoco_module, expected_wrist) -> None:
        for geom_type, size, data_id, wrist_from_geom in self._geoms:
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


def _run_alignment_viewer(
    reference_path: Path,
    reference,
    live: RegrindMotiveTracker,
    stale_s: float,
    session,
) -> bool:
    import mujoco
    import mujoco.viewer

    model = _build_alignment_model(reference_path, mujoco)
    data = mujoco.MjData(model)
    router_zid = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID"))
    _, arm_instance, hand_instance = _authority_instances(
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

    reference_wrist = np.concatenate((reference.wrist_pos[0], np.roll(reference.wrist_quat_wxyz[0], -1)))
    reference_hammer = np.concatenate((reference.object_pos[0], np.roll(reference.object_quat_wxyz[0], -1)))
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
        model, data.qpos, home_wrist, reference.joints[0], mujoco
    )
    subscriptions = (
        session.declare_subscriber(topics.ARM_STATE, mirror.on_arm_state),
        session.declare_subscriber(topics.hand_state("right"), mirror.on_hand_state),
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
                    last_passed = fresh and position_error_mm <= 10.0 and orientation_error_deg <= 5.0
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
                        axis_origin = current_wrist[:3] + np.asarray([-0.15, -0.15, 0.15])
                        for axis, color in zip(
                            np.eye(3) * 0.12,
                            (
                                [1.0, 0.0, 0.0, 1.0],
                                [0.0, 1.0, 0.0, 1.0],
                                [0.0, 0.4, 1.0, 1.0],
                            ),
                        ):
                            arrows.append((axis_origin, axis_origin + axis, 0.005, np.asarray(color, dtype=np.float32)))
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
                        expected_hand.draw(scene, mujoco, expected_wrist)
                    status = "PASS" if last_passed else ("STALE" if not fresh else "ADJUST")
                    viewer.set_texts((
                        mujoco.mjtFontScale.mjFONTSCALE_150,
                        mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        "READ-ONLY  Solid robot: LIVE state\n"
                        "Orange hammer: LIVE  Green hammer/hand: EXPECTED frame0\n"
                        "Control terminal: s + hold Enter -> frame0\n"
                        "Then release, align hammer, press i; hold Enter -> infer\n"
                        "Yellow arrow: move live hammer toward target\n"
                        "Robot/Zenoh: +X forward(red), +Y left(green), +Z up(blue)\n"
                        "Correction in robot world XYZ (mm)\n"
                        "Position / orientation error\n"
                        "Motive age\nStatus",
                        f"\n\n\n\n\n\n[{delta_mm[0]:+7.1f}, {delta_mm[1]:+7.1f}, {delta_mm[2]:+7.1f}]\n"
                        f"{position_error_mm:7.1f} mm / {orientation_error_deg:6.1f} deg\n"
                        f"{age_s * 1000.0:7.1f} ms\n{status}",
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--endpoint", default=os.environ.get("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447"))
    parser.add_argument("--wrist-name", default="tianji_wrist")
    parser.add_argument("--hammer-name", default="hammer")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--stale-s", type=float, default=0.25)
    parser.add_argument("--wait-s", type=float, default=10.0)
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--viewer", action="store_true", help="open read-only live/expected frame-0 alignment viewer")
    args = parser.parse_args()
    if not args.model.is_file() or not args.reference.is_file():
        parser.error("--model and --reference must be existing files")
    if args.rate <= 0.0 or args.stale_s <= 0.0 or args.wait_s <= 0.0 or args.print_every < 1:
        parser.error("rate/timeouts/print-every must be positive")
    if args.viewer and args.preflight_only:
        parser.error("--viewer and --preflight-only are mutually exclusive")

    reference = load_reference(args.reference)
    if args.viewer:
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
        reference_wrist_zero = np.concatenate((reference.wrist_pos[0], np.roll(reference.wrist_quat_wxyz[0], -1)))
        reference_hammer_zero = np.concatenate((reference.object_pos[0], np.roll(reference.object_quat_wxyz[0], -1)))
        training_from_motive = np.asarray(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64
        )
        aligned_hammer_zero = compose_pose(training_from_motive, hammer_zero)
        hammer_position_error_m = float(np.linalg.norm(aligned_hammer_zero[:3] - reference_hammer_zero[:3]))
        hammer_rotation_error_deg = float(np.rad2deg(np.linalg.norm((
            Rotation.from_quat(reference_hammer_zero[3:]).inv()
            * Rotation.from_quat(aligned_hammer_zero[3:])
        ).as_rotvec())))
        pose_preflight_passed = hammer_position_error_m <= 0.01 and hammer_rotation_error_deg <= 5.0
        previous_wrist = compose_pose(training_from_motive, wrist_zero)
        previous_wrist_pos = previous_wrist[:3].copy()
        previous_wrist_quat = np.roll(previous_wrist[3:], 1)
        joints = reference.joints[0].copy()
        previous_joints = joints.copy()
        last_action = np.zeros(26, dtype=np.float64)
        next_tick = time.monotonic()

        print(json.dumps({
            "event": "started",
            "mode": "live_motive_alignment_viewer" if args.viewer else "live_motive_shadow_inference",
            "publishes_control": False,
            "checkpoint_iteration": iteration,
            "rate_hz": args.rate,
            "frames": reference.frame_count - 1,
            "hand_joint_observation": "previous_policy_target_assuming_perfect_tracking",
            "hammer_start_position_error_mm": round(hammer_position_error_m * 1000.0, 3),
            "hammer_start_orientation_error_deg": round(hammer_rotation_error_deg, 3),
            "real_start_preflight_passed": pose_preflight_passed,
        }, separators=(",", ":")), flush=True)
        if args.preflight_only:
            return 0 if pose_preflight_passed else 1
        if args.viewer:
            passed = _run_alignment_viewer(args.reference, reference, live, args.stale_s, session)
            print(json.dumps({"event": "viewer_closed", "real_start_preflight_passed": passed}), flush=True)
            return 0

        for index in range(reference.frame_count - 1):
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
        print(json.dumps({"event": "completed", "frames": reference.frame_count - 1}), flush=True)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        live.close()
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
