#!/usr/bin/env python3
"""kp800 数据包 → v4.0 标准 H5 转换器。

把 regrind 数据集流水线导出的三类 H5(1_原始动捕 / 2_regrind重定向 /
3_RL回放)批量转换为 mocap-acquisition v4.0 标准
(docs/mocap_h5_v40_format.md),供项目回放链路直接加载。

映射规则(全部经 take001 双版本数值交叉验证):
- 键点:绝对坐标直接搬(0 号点==root 位姿,与 v4.0 语义一致)
- 四元数:kp800 为 wxyz 序 → 重排为 xyzw
- wrist 系:1_原始动捕 经 take001 标定的固定旋转 R(≈90° 轴置换)转到
  v4.0 W 系;2_regrind/3_RL 的 root 系无标定依据,仅重排(回放需目检)
- wuji2_joints(v5 可选):2_regrind 填 regrind_retargeting_joints、
  3_RL 填 rl_joints(关节序均为 wuji2 URDF 序,已与 3_RL 的 joint_order
  属性及 reference 一致性验证);1_原始动捕 的 robot_joints 序未证实,不填
- 左手:全 NaN + valid=0(链路按"保持 Home"处理)
- 3_RL 无键点:keypoints 填 root 位姿广播(保持 wrist 有效以驱动机械臂;
  手指由 wuji2_joints 直通驱动)
- 物体:object_pos/object_quat → objects/hammer/(quat 重排 xyzw)
- 时间轴:50 Hz 合成 time_ns(数据包声明 50 Hz)

用法:
  pixi run python scripts/convert_kp800_to_v40.py [--input DIR] [--output DIR]
  --input  默认 /home/current/Documents/pkg_kp800_20260827
  --output 默认 /home/current/data/kp800_v40(按子目录同名输出)
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

import h5py
import numpy as np
from scipy.spatial.transform import Rotation

OUTPUT_HZ = 50.0
OBJECT_NAME = "hammer"

# take001 标定的 kp800 wrist 系 → v4.0 W 系 固定旋转(手静止帧段均值)。
# kp x→W y, kp y→W x, kp z→W −z(约 90° 轴置换,regrind 手掌参考系)。
_WRIST_W_FROM_KP800 = np.array(
    [
        [0.035, 0.998, -0.03],
        [0.95, -0.02, 0.32],
        [0.32, -0.04, -0.95],
    ]
)
_WRIST_ROT = Rotation.from_matrix(_WRIST_W_FROM_KP800)

TAKE_ID_RE = re.compile(r"take(\d+)")

KIND_RAW = "1_原始动捕"
KIND_REGRIND = "2_regrind重定向"
KIND_RL = "3_RL回放"
KIND_NAMES = {
    KIND_RAW: "robot_keypoints",
    KIND_REGRIND: "regrind_retargeting_joints",
    KIND_RL: "rl_joints",
}


def _detect_kind(group: h5py.Group) -> str | None:
    # 按特异性优先:regrind/RL 字段最特异;robot_keypoints 三个目录都有。
    for kind, marker in (
        (KIND_REGRIND, "regrind_retargeting_joints"),
        (KIND_RL, "rl_joints"),
        (KIND_RAW, "robot_keypoints"),
    ):
        if marker in group:
            return kind
    return None


def _wxyz_to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    return quat_wxyz[:, [1, 2, 3, 0]]


def _take_id_from_name(path: Path) -> int | None:
    match = TAKE_ID_RE.search(path.stem)
    return int(match.group(1)) if match else None


def _write_left_nan(f: h5py.File, frames: int) -> None:
    group = f.create_group("hands/left")
    group.create_dataset(
        "keypoints_world",
        data=np.full((frames, 21, 3), np.nan, dtype=np.float32),
    )
    group.create_dataset(
        "wrist_position",
        data=np.full((frames, 3), np.nan, dtype=np.float32),
    )
    group.create_dataset(
        "wrist_quaternion_xyzw",
        data=np.full((frames, 4), np.nan, dtype=np.float32),
    )
    group.create_dataset("valid", data=np.zeros(frames, dtype=np.uint8))


def _write_object(f: h5py.File, group: h5py.Group, frames: int) -> None:
    obj = f.create_group(f"objects/{OBJECT_NAME}")
    if "object_pos" in group and "object_quat" in group:
        obj.create_dataset("object_position", data=group["object_pos"][:])
        obj.create_dataset(
            "object_quaternion_xyzw",
            data=_wxyz_to_xyzw(group["object_quat"][:]),
        )
        obj.create_dataset("valid", data=np.ones(frames, dtype=np.uint8))
    else:
        # 无物体数据:全 NaN + 无效(格式合规)
        obj.create_dataset(
            "object_position",
            data=np.full((frames, 3), np.nan, dtype=np.float32),
        )
        obj.create_dataset(
            "object_quaternion_xyzw",
            data=np.full((frames, 4), np.nan, dtype=np.float32),
        )
        obj.create_dataset("valid", data=np.zeros(frames, dtype=np.uint8))


def _fill_right(f: h5py.File, kind: str, group: h5py.Group, frames: int) -> None:
    """写 hands/right;返回 (键点全 NaN 标记)。"""
    right = f.create_group("hands/right")

    if kind == KIND_RAW:
        keypoints = group["robot_keypoints"][:]
        position = group["robot_pos"][:]
        quat = _wxyz_to_xyzw(group["robot_quat"][:])
        # 标定旋转:kp800 系 → v4.0 W 系
        rotated = _WRIST_ROT * Rotation.from_quat(quat)
        quat = rotated.as_quat()
        wuji2_joints = None
    elif kind == KIND_REGRIND:
        keypoints = group["robot_keypoints"][:]
        position = group["regrind_retargeting_root_pos"][:]
        quat = _wxyz_to_xyzw(group["regrind_retargeting_root_quat"][:])
        wuji2_joints = group["regrind_retargeting_joints"][:]
    else:  # KIND_RL
        # 无键点数据:root 位姿广播(保持 wrist 有效驱动机械臂;手指直通)
        position = group["rl_root_pos"][:]
        keypoints = np.repeat(position[:, None, :], 21, axis=1)
        quat = _wxyz_to_xyzw(group["rl_root_quat"][:])
        wuji2_joints = group["rl_joints"][:]

    finite = (
        np.isfinite(keypoints).all(axis=(1, 2))
        & np.isfinite(position).all(axis=1)
        & np.isfinite(quat).all(axis=1)
    )
    right.create_dataset(
        "keypoints_world", data=keypoints.astype(np.float32)
    )
    right.create_dataset(
        "wrist_position", data=position.astype(np.float32)
    )
    right.create_dataset(
        "wrist_quaternion_xyzw", data=quat.astype(np.float32)
    )
    right.create_dataset("valid", data=finite.astype(np.uint8))
    if wuji2_joints is not None:
        right.create_dataset(
            "wuji2_joints", data=wuji2_joints.astype(np.float32)
        )


def convert(
    src: Path, dst: Path, kind_hint: str | None = None
) -> dict[str, object]:
    """单个 kp800 H5 → v4.0 H5;返回摘要。"""
    with h5py.File(src, "r") as f:
        kind = kind_hint if kind_hint is not None else _detect_kind(f)
        if kind is None:
            raise ValueError(f"{src.name}: 无法识别数据类别(字段缺失)")
        group = f  # 数据集都在根
        marker = KIND_NAMES[kind]
        frames = int(group[marker].shape[0])

        with h5py.File(dst, "w") as out:
            out.attrs["h5_version"] = "4.0"
            out.attrs["schema_name"] = "mocap-acquisition"
            out.attrs["schema_layout"] = "compact-aligned-60hz-v1"
            out.attrs["output_hz"] = OUTPUT_HZ
            take_id = _take_id_from_name(src)
            if take_id is not None:
                out.attrs["take_id"] = take_id
            out.attrs["source"] = str(src)
            out.create_dataset(
                "time_ns",
                data=np.arange(frames, dtype=np.int64)
                * int(1.0e9 / OUTPUT_HZ),
            )
            out.create_dataset(
                "valid", data=np.ones(frames, dtype=np.uint8)
            )
            _write_left_nan(out, frames)
            _write_object(out, f, frames)
            _fill_right(out, kind, group, frames)

    with h5py.File(dst, "r") as check:
        right = check["hands/right"]
        has_joints = "wuji2_joints" in right
        valid_ratio = float(
            check["hands/right/valid"][:].mean()
        )
    return {
        "source": src.name,
        "kind": kind,
        "frames": frames,
        "wuji2_joints": has_joints,
        "right_valid_ratio": round(valid_ratio, 3),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("/home/current/Documents/pkg_kp800_20260827"),
        help="kp800 数据包目录(默认取该路径)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/home/current/data/kp800_v40"),
        help="输出根目录(默认 /home/current/data/kp800_v40)",
    )
    args = parser.parse_args()

    source_dirs = [d for d in args.input.iterdir() if d.is_dir()]
    converted = 0
    for src_dir in sorted(source_dirs):
        if src_dir.name not in (KIND_RAW, KIND_REGRIND, KIND_RL):
            continue
        out_dir = args.output / src_dir.name
        out_dir.mkdir(parents=True, exist_ok=True)
        for src in sorted(src_dir.glob("*.h5")):
            dst = out_dir / src.name
            summary = convert(src, dst, kind_hint=src_dir.name)
            print(
                f"  [{summary['kind']}] {src.name}: "
                f"{summary['frames']}帧 "
                f"wuji2_joints={'有' if summary['wuji2_joints'] else '无'} "
                f"右手有效率={summary['right_valid_ratio']}"
            )
            converted += 1
    print(f"完成:{converted} 个文件 → {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
