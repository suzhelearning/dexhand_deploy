#!/usr/bin/env python3
"""Run CPU Regrind inference from live Motive wrist and hammer poses only."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from tianji_teleop.protocol import topics
from tianji_teleop.regrind_policy import action_to_targets, build_observation, infer, load_actor, load_reference
from tianji_teleop.sources.mocap.motive import MotiveFrame, MotiveFrameSource
from tianji_teleop.zenoh_util import ZenohJsonSub, open_session


IDENTITY_POSE = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
HAMMER_RIGID_TO_OBJECT = np.asarray(
    [0.002, -0.005, 0.0, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476]
)
WRIST_RIGID_TO_MARKER = np.asarray(
    [0.001, -0.004, 0.002, -0.0086933284, 0.0871524241, 0.0007605677, 0.9961567661]
)
MARKER_TO_MOUNT = np.asarray(
    [0.004, 0.0, 0.0, 0.0, -0.7071067811865476, 0.0, 0.7071067811865476]
)
MOUNT_TO_WRIST = np.asarray(
    [0.003, 0.00025016, -0.0285, 0.0, 0.0, 0.0000081994999999, 0.9999999999663841]
)


def compose_pose(parent_from_middle: np.ndarray, middle_from_child: np.ndarray) -> np.ndarray:
    first = np.asarray(parent_from_middle, dtype=np.float64)
    second = np.asarray(middle_from_child, dtype=np.float64)
    if first.shape != (7,) or second.shape != (7,) or not np.isfinite(first).all() or not np.isfinite(second).all():
        raise ValueError("poses must be finite xyz+xyzw vectors")
    first_rotation = Rotation.from_quat(first[3:])
    return np.concatenate(
        (first[:3] + first_rotation.apply(second[:3]), (first_rotation * Rotation.from_quat(second[3:])).as_quat())
    )


def invert_pose(parent_from_child: np.ndarray) -> np.ndarray:
    pose = np.asarray(parent_from_child, dtype=np.float64)
    rotation = Rotation.from_quat(pose[3:]).inv()
    return np.concatenate((rotation.apply(-pose[:3]), rotation.as_quat()))


class LiveMotivePoses:
    def __init__(self, session: Any, wrist_name: str, hammer_name: str) -> None:
        self._parser = MotiveFrameSource()
        self._wrist_name = wrist_name
        self._hammer_name = hammer_name
        self._lock = threading.Lock()
        self._names: dict[int, str] = {}
        self._frame: MotiveFrame | None = None
        self._received_at = 0.0
        self._names_sub = ZenohJsonSub(session, topics.MOCAP_RIGID_BODY_NAMES, self._on_names)
        self._frame_sub = ZenohJsonSub(session, topics.MOCAP_HANDS_FRAME, self._on_frame)

    def _on_names(self, payload: Any) -> None:
        names = self._parser.parse_names(payload)
        for wanted in (self._wrist_name, self._hammer_name):
            if list(names.values()).count(wanted) != 1:
                raise ValueError(f"Motive must contain exactly one rigid body named {wanted!r}")
        with self._lock:
            self._names = names

    def _on_frame(self, payload: Any) -> None:
        frame = self._parser.parse(payload)
        with self._lock:
            self._frame = frame
            self._received_at = time.monotonic()

    def latest(self) -> tuple[int, float, np.ndarray, np.ndarray] | None:
        with self._lock:
            frame, names, received_at = self._frame, dict(self._names), self._received_at
        if frame is None:
            return None
        ids = {name: rigid_id for rigid_id, name in names.items()}
        if self._wrist_name not in ids or self._hammer_name not in ids:
            return None
        wrist_rigid = frame.rigid_pose(ids[self._wrist_name])
        hammer_rigid = frame.rigid_pose(ids[self._hammer_name])
        if wrist_rigid is None or hammer_rigid is None:
            return None
        rigid_to_wrist = compose_pose(compose_pose(WRIST_RIGID_TO_MARKER, MARKER_TO_MOUNT), MOUNT_TO_WRIST)
        return (
            frame.frame_number,
            received_at,
            compose_pose(wrist_rigid, rigid_to_wrist),
            compose_pose(hammer_rigid, HAMMER_RIGID_TO_OBJECT),
        )

    def close(self) -> None:
        self._names_sub.close()
        self._frame_sub.close()


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
    args = parser.parse_args()
    if not args.model.is_file() or not args.reference.is_file():
        parser.error("--model and --reference must be existing files")
    if args.rate <= 0.0 or args.stale_s <= 0.0 or args.wait_s <= 0.0 or args.print_every < 1:
        parser.error("rate/timeouts/print-every must be positive")

    torch.set_num_threads(1)
    actor, mean, variance, iteration = load_actor(args.model)
    reference = load_reference(args.reference)
    session = open_session(args.endpoint)
    live = LiveMotivePoses(session, args.wrist_name, args.hammer_name)
    try:
        deadline = time.monotonic() + args.wait_s
        sample = live.latest()
        while sample is None and time.monotonic() < deadline:
            time.sleep(0.01)
            sample = live.latest()
        if sample is None:
            raise RuntimeError("timed out waiting for valid Motive wrist+hammer poses")

        _, received_at, wrist_zero, hammer_zero = sample
        if time.monotonic() - received_at > args.stale_s:
            raise RuntimeError("initial Motive frame is stale")
        reference_hammer_zero = np.concatenate((reference.object_pos[0], np.roll(reference.object_quat_wxyz[0], -1)))
        training_from_motive = compose_pose(reference_hammer_zero, invert_pose(hammer_zero))
        previous_wrist = compose_pose(training_from_motive, wrist_zero)
        previous_wrist_pos = previous_wrist[:3].copy()
        previous_wrist_quat = np.roll(previous_wrist[3:], 1)
        joints = reference.joints[0].copy()
        previous_joints = joints.copy()
        last_action = np.zeros(26, dtype=np.float64)
        next_tick = time.monotonic()

        print(json.dumps({
            "event": "started",
            "mode": "live_motive_shadow_inference",
            "publishes_control": False,
            "checkpoint_iteration": iteration,
            "rate_hz": args.rate,
            "frames": reference.frame_count - 1,
            "hand_joint_observation": "previous_policy_target_assuming_perfect_tracking",
        }, separators=(",", ":")), flush=True)

        for index in range(reference.frame_count - 1):
            next_tick += 1.0 / args.rate
            sample = live.latest()
            if sample is None:
                raise RuntimeError("Motive wrist or hammer tracking invalid")
            motive_frame, received_at, wrist_live, hammer_live = sample
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
