#!/usr/bin/env python3
"""Load the Regrind RSL-RL actor and run a CPU-only smoke inference."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import h5py
import numpy as np
import torch
from torch import nn


DIMS = (123, 1024, 512, 256, 128, 26)
NORMALIZATION_EPS = 1e-8
DEFAULT_JOINT_POS = np.asarray([0.28, *([0.0] * 19)], dtype=np.float64)


def load_actor(
    checkpoint: Path,
) -> tuple[nn.Sequential, torch.Tensor, torch.Tensor, int]:
    payload = torch.load(checkpoint, map_location="cpu", weights_only=True)
    state = payload.get("actor_state_dict")
    if not isinstance(state, dict):
        raise ValueError("checkpoint has no actor_state_dict")

    layers: list[nn.Module] = []
    for index, (input_dim, output_dim) in enumerate(zip(DIMS[:-1], DIMS[1:])):
        layers.append(nn.Linear(input_dim, output_dim))
        if index < len(DIMS) - 2:
            layers.append(nn.ELU())
    actor = nn.Sequential(*layers)
    actor.load_state_dict(
        {
            key.removeprefix("mlp."): value
            for key, value in state.items()
            if key.startswith("mlp.")
        },
        strict=True,
    )

    mean = state.get("obs_normalizer._mean")
    variance = state.get("obs_normalizer._var")
    if not isinstance(mean, torch.Tensor) or not isinstance(variance, torch.Tensor):
        raise ValueError("checkpoint has no observation normalization statistics")
    mean = mean.reshape(-1)
    variance = variance.reshape(-1)
    if tuple(mean.shape) != (DIMS[0],) or tuple(variance.shape) != (DIMS[0],):
        raise ValueError(
            f"normalizer must have shape ({DIMS[0]},), got {tuple(mean.shape)} and {tuple(variance.shape)}"
        )
    actor.eval()
    return actor, mean, variance, int(payload.get("iter", -1))


def quat_to_rot6d(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion) + 1e-12
    w, x, y, z = quaternion
    rotation = np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    return rotation[:, :2].reshape(-1)


def load_frame_zero_observation(trajectory: Path) -> np.ndarray:
    with h5py.File(trajectory, "r") as stream:
        wrist_pos = np.asarray(stream["regrind_retargeting_root_pos"][0], dtype=np.float64)
        wrist_quat = np.asarray(stream["regrind_retargeting_root_quat"][0], dtype=np.float64)
        joints = np.asarray(stream["regrind_retargeting_joints"][0], dtype=np.float64)
        object_pos = np.asarray(stream["object_pos"][0], dtype=np.float64)
        object_quat = np.asarray(stream["object_quat"][0], dtype=np.float64)
    if wrist_pos.shape != (3,) or wrist_quat.shape != (4,) or joints.shape != (20,):
        raise ValueError("trajectory frame must contain wrist (3+4) and 20 hand joints")

    wrist_rot6d = quat_to_rot6d(wrist_quat)
    relative_joints = joints - DEFAULT_JOINT_POS
    observation = np.concatenate(
        [
            object_pos,
            quat_to_rot6d(object_quat),
            wrist_pos,
            wrist_pos,
            wrist_rot6d,
            wrist_rot6d,
            relative_joints,
            relative_joints,
            np.zeros(DIMS[-1]),
            np.zeros(1),
            wrist_pos,
            wrist_rot6d,
            joints,
        ]
    )
    if observation.shape != (DIMS[0],) or not np.isfinite(observation).all():
        raise ValueError(f"frame-zero observation must be {DIMS[0]} finite values")
    return observation.astype(np.float32)


def infer(
    actor: nn.Sequential,
    mean: torch.Tensor,
    variance: torch.Tensor,
    observation: np.ndarray,
) -> np.ndarray:
    if observation.shape != (DIMS[0],) or not np.isfinite(observation).all():
        raise ValueError(f"observation must be {DIMS[0]} finite values")
    obs = torch.from_numpy(observation.astype(np.float32, copy=False)).reshape(1, -1)
    with torch.inference_mode():
        action = actor((obs - mean) / torch.sqrt(variance + NORMALIZATION_EPS))
    result = action.squeeze(0).numpy()
    if result.shape != (DIMS[-1],) or not np.isfinite(result).all():
        raise ValueError(f"actor output must be {DIMS[-1]} finite values")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--h5", type=Path, required=True)
    parser.add_argument("--iterations", type=int, default=1000)
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error("--iterations must be positive")
    if not args.model.is_file():
        parser.error(f"model not found: {args.model}")
    if not args.h5.is_file():
        parser.error(f"trajectory not found: {args.h5}")

    torch.set_num_threads(1)
    started = time.perf_counter_ns()
    actor, mean, variance, iteration = load_actor(args.model)
    load_ms = (time.perf_counter_ns() - started) / 1e6
    observation = load_frame_zero_observation(args.h5)

    for _ in range(20):
        infer(actor, mean, variance, observation)
    timings_ms = []
    for _ in range(args.iterations):
        started = time.perf_counter_ns()
        action = infer(actor, mean, variance, observation)
        timings_ms.append((time.perf_counter_ns() - started) / 1e6)

    print(
        json.dumps(
            {
                "device": "cpu",
                "input": f"{args.h5.resolve()}:frame0",
                "checkpoint": str(args.model.resolve()),
                "checkpoint_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
                "checkpoint_iteration": iteration,
                "obs_dim": DIMS[0],
                "action_dim": DIMS[-1],
                "load_ms": round(load_ms, 3),
                "inference_mean_ms": round(float(np.mean(timings_ms)), 3),
                "inference_p99_ms": round(float(np.percentile(timings_ms, 99)), 3),
                "within_20ms": bool(np.percentile(timings_ms, 99) < 20.0),
                "raw_action": action.tolist(),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
