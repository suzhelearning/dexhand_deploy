#!/usr/bin/env python3
"""生成 mocap-acquisition v4.0 格式的合成台阶轨迹 HDF5。

用于轨迹跟踪验收：生成一条「+axis 方向移动 N mm → 保持 → 回程」的
手腕轨迹，交给 ``sim_mocap`` 回放，即可验证机器人目标/求解位移与
命令位移严格 1:1（例如 +x 移动 50mm，机器人目标即移动 50mm）。

生成文件与采集端 v4.0 布局一致（schema 常量复用 ``mocap_h5``），
保证 ``load_mocap_h5`` 可直接读取。左右手轨迹相同，参考位姿取
(0.10, 0.20, -0.10) + 单位四元数（相对增量映射下参考值本身无关）。

用法：

    mocap_step_h5 --output step50mm_x.h5 --axis x --mm 50 \\
        --ramp-s 1.0 --hold-s 1.5 --return-s 1.0

    mocap_step_h5 --output robot_forward_50mm.h5 --axis z --dir neg --mm 50

依赖：h5py、numpy（无 ROS 依赖，可独立运行）。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np

from .mocap_h5 import SIDES, SUPPORTED_H5_VERSION, SUPPORTED_SCHEMA_LAYOUT

_AXES = {"x": 0, "y": 1, "z": 2}
_BASE_POSE = np.array([0.10, 0.20, -0.10, 0.0, 0.0, 0.0, 1.0])


def _ramp_profile(
    frames: int, ramp_frames: int, hold_frames: int
) -> np.ndarray:
    """0→1（ramp）、1（hold）、1→0（return）的归一化幅值轨迹。"""
    amplitude = np.ones(frames, dtype=np.float64)
    if ramp_frames > 0:
        amplitude[:ramp_frames] = np.linspace(
            0.0, 1.0, ramp_frames, endpoint=True
        )
    return_frames = frames - ramp_frames - hold_frames
    if return_frames > 0:
        amplitude[ramp_frames + hold_frames:] = np.linspace(
            1.0, 0.0, return_frames, endpoint=True
        )
    return amplitude


def generate_step_h5(
    output: Path,
    *,
    axis: str = "x",
    mm: float = 50.0,
    direction: str = "pos",
    ramp_s: float = 1.0,
    hold_s: float = 1.5,
    return_s: float = 1.0,
    rate: float = 60.0,
) -> Path:
    """生成台阶轨迹 HDF5 并返回输出路径。

    输入（手腕/Motive）系与机器人 chest 系的轴映射（pico_to_robot ∘
    world→chest，见 docs/mocap_replay.md）：输入 +z → 机器人 −x；
    因此要在机器人 +x（前）方向移动，应使用 ``axis=z, direction=neg``。
    """
    if axis not in _AXES:
        raise ValueError(f"axis 必须是 {sorted(_AXES)} 之一，实际 {axis!r}")
    if direction not in ("pos", "neg"):
        raise ValueError(f"direction 必须是 pos/neg 之一，实际 {direction!r}")
    if not np.isfinite(mm) or mm <= 0.0:
        raise ValueError("mm 必须为正有限数值")
    if min(ramp_s, hold_s, return_s) < 0.0 or rate <= 0.0:
        raise ValueError("ramp_s/hold_s/return_s/rate 必须非负/为正")

    axis_index = _AXES[axis]
    sign = -1.0 if direction == "neg" else 1.0
    ramp_frames = int(round(ramp_s * rate))
    hold_frames = int(round(hold_s * rate))
    return_frames = int(round(return_s * rate))
    frames = ramp_frames + hold_frames + return_frames
    if frames < 2:
        raise ValueError("总帧数不足 2，无法生成有效时间轴")

    amplitude = _ramp_profile(frames, ramp_frames, hold_frames)
    displacement = np.zeros((frames, 3), dtype=np.float64)
    displacement[:, axis_index] = sign * amplitude * (mm / 1000.0)

    position = np.tile(_BASE_POSE[:3], (frames, 1)) + displacement
    quaternion = np.tile(_BASE_POSE[3:], (frames, 1))
    wrist = np.concatenate((position, quaternion), axis=1)
    time_ns = (
        np.arange(frames, dtype=np.int64) * int(1.0e9 / rate)
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output, "w") as f:
        f.attrs["h5_version"] = SUPPORTED_H5_VERSION
        f.attrs["schema_name"] = "mocap-acquisition"
        f.attrs["schema_layout"] = SUPPORTED_SCHEMA_LAYOUT
        f.attrs["output_hz"] = float(rate)
        f.attrs["take_id"] = 0
        f.attrs["time_domain"] = "linux-clock-monotonic"
        f.create_dataset("time_ns", data=time_ns)
        f.create_dataset("valid", data=np.ones(frames, dtype=np.uint8))
        for side in SIDES:
            group = f.create_group(f"hands/{side}")
            group.attrs["keypoint_count"] = 21
            group.attrs["source"] = "manus"
            group.create_dataset(
                "wrist_position", data=wrist[:, :3].astype(np.float32)
            )
            group.create_dataset(
                "wrist_quaternion_xyzw",
                data=wrist[:, 3:].astype(np.float32),
            )
            group.create_dataset(
                "valid", data=np.ones(frames, dtype=np.uint8)
            )
        f.create_group("objects")
        events = f.create_group("events")
        events.create_dataset("frame_index", data=np.array([0, frames - 1]))
        events.create_dataset("type", data=np.array([1, 2], dtype=np.uint8))
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="生成 mocap v4.0 合成台阶轨迹 HDF5（轨迹跟踪 1:1 验收）"
    )
    parser.add_argument("--output", required=True, type=Path,
                        help="输出 HDF5 路径")
    parser.add_argument("--axis", choices=sorted(_AXES), default="x",
                        help="输入（手腕）系移动轴（默认 x）")
    parser.add_argument("--dir", dest="direction",
                        choices=("pos", "neg"), default="pos",
                        help="移动方向（默认 pos；机器人 +x 用 --axis z --dir neg）")
    parser.add_argument("--mm", type=float, default=50.0,
                        help="移动距离毫米（默认 50）")
    parser.add_argument("--ramp-s", type=float, default=1.0,
                        help="匀加速爬升时长秒（默认 1.0）")
    parser.add_argument("--hold-s", type=float, default=1.5,
                        help="保持时长秒（默认 1.5）")
    parser.add_argument("--return-s", type=float, default=1.0,
                        help="回程时长秒（默认 1.0）")
    parser.add_argument("--rate", type=float, default=60.0,
                        help="帧率 Hz（默认 60，与 mocap 布局一致）")
    args = parser.parse_args(argv)

    output = generate_step_h5(
        args.output,
        axis=args.axis,
        mm=args.mm,
        direction=args.direction,
        ramp_s=args.ramp_s,
        hold_s=args.hold_s,
        return_s=args.return_s,
        rate=args.rate,
    )
    print(f"已生成台阶轨迹：{output} "
          f"axis={args.axis} dir={args.direction} mm={args.mm} "
          f"ramp={args.ramp_s}s hold={args.hold_s}s return={args.return_s}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
