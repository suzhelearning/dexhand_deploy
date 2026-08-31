from __future__ import annotations

import numpy as np


def apply_joint_positions(
    qpos,
    qpos_address_by_name: dict[str, int],
    names,
    positions,
) -> int:
    """按关节名把 ROS JointState 写入 MuJoCo qpos。"""
    joint_names = list(names)
    values = np.asarray(positions, dtype=float)
    if len(joint_names) != len(values):
        raise ValueError("joint names and positions have different lengths")
    if not np.isfinite(values).all():
        raise ValueError("joint positions must be finite")

    applied = 0
    for name, value in zip(joint_names, values):
        address = qpos_address_by_name.get(name)
        if address is None:
            continue
        qpos[address] = float(value)
        applied += 1
    return applied
