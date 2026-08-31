#!/usr/bin/env python3
"""Run CPU Regrind inference from live Motive wrist and hammer poses only."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from tianji_teleop.regrind_policy import action_to_targets, build_observation, infer, load_actor, load_reference
from tianji_teleop.sources.mocap.h5 import compose_pose, invert_pose
from tianji_teleop.sources.mocap.regrind import RegrindMotiveTracker
from tianji_teleop.zenoh_util import open_session


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
    args = parser.parse_args()
    if not args.model.is_file() or not args.reference.is_file():
        parser.error("--model and --reference must be existing files")
    if args.rate <= 0.0 or args.stale_s <= 0.0 or args.wait_s <= 0.0 or args.print_every < 1:
        parser.error("rate/timeouts/print-every must be positive")

    torch.set_num_threads(1)
    actor, mean, variance, iteration = load_actor(args.model)
    reference = load_reference(args.reference)
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
        training_from_motive = compose_pose(reference_wrist_zero, invert_pose(wrist_zero))
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
            "mode": "live_motive_shadow_inference",
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
