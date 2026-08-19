#!/usr/bin/env python3
"""键盘步进控制的纯逻辑：转义序列解析 + 动捕系位移累积（无 ROS 依赖）。

按键映射（动捕/Motive 系，y-up）：

    上 ← ↑（ESC [ A）→ 动捕 +z     下 ← ↓（ESC [ B）→ 动捕 -z
    左 ← ←（ESC [ D）→ 动捕 +x     右 ← →（ESC [ C）→ 动捕 -x
    '1' → +y                        '0' → -y
    's' → 开始/结束（由节点状态机决定）

方向键在 termios raw 模式下是 3 字节转义序列 ``\\x1b [ A/B/C/D``，
``ArrowKeyParser.feed`` 逐字节喂入后返回完整按键事件；普通单字节键
直接返回。``StepAccumulator`` 把按键事件累积为动捕系位姿增量
（每次按键 +step_mm），供映射链路（ControllerOnlyTeleopMapper）
转换为机器人目标——每次按键机器人末端移动 step_mm。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 按键事件 → 动捕系单位方向（x, y, z）
AXIS_STEPS: dict[str, tuple[float, float, float]] = {
    "up": (0.0, 0.0, 1.0),
    "down": (0.0, 0.0, -1.0),
    "left": (1.0, 0.0, 0.0),
    "right": (-1.0, 0.0, 0.0),
    "1": (0.0, 1.0, 0.0),
    "0": (0.0, -1.0, 0.0),
}

_ESCAPE_MAP = {
    "A": "up",
    "B": "down",
    "C": "right",
    "D": "left",
}


class ArrowKeyParser:
    """逐字节解析键盘输入，返回按键事件名（无事件返回 None）。"""

    def __init__(self) -> None:
        self._state = "normal"  # normal | esc | bracket

    def feed(self, byte: str) -> str | None:
        if self._state == "normal":
            if byte == "\x1b":
                self._state = "esc"
                return None
            return byte  # 单字节键（s/1/0/其他）
        if self._state == "esc":
            if byte == "[":
                self._state = "bracket"
                return None
            self._state = "normal"
            return None  # 非法转义序列，丢弃
        # bracket
        self._state = "normal"
        return _ESCAPE_MAP.get(byte)


@dataclass(frozen=True)
class StepAccumulator:
    """参考位姿 + 按键累积位移 → 当前位姿（动捕系 7 向量）。"""

    reference_pose: np.ndarray  # [x, y, z, qx, qy, qz, qw]
    step_mm: float = 10.0

    def __post_init__(self) -> None:
        pose = np.asarray(self.reference_pose, dtype=np.float64)
        if pose.shape != (7,) or not np.isfinite(pose).all():
            raise ValueError("reference_pose 必须是非空 7 向量")
        quaternion_norm = float(np.linalg.norm(pose[3:]))
        if not 0.95 <= quaternion_norm <= 1.05:
            raise ValueError("reference_pose 四元数无效")
        if not np.isfinite(self.step_mm) or self.step_mm <= 0.0:
            raise ValueError("step_mm 必须为正有限数值")
        object.__setattr__(
            self, "reference_pose", pose.copy()
        )
        object.__setattr__(
            self, "_delta", np.zeros(3, dtype=np.float64)
        )

    def reset(self) -> None:
        """清零累积位移（回到参考位姿）。"""
        object.__setattr__(
            self,
            "_delta",
            np.zeros(3, dtype=np.float64),
        )

    def pose(self) -> np.ndarray:
        """当前位姿 = 参考位姿 + 累积位移（动捕系）。"""
        result = self.reference_pose.copy()
        result[:3] += self._delta
        return result

    def delta_m(self) -> np.ndarray:
        return self._delta.copy()

    def step(self, event: str) -> np.ndarray:
        """按一次键：累积 step_mm 位移并返回当前位姿。"""
        if event not in AXIS_STEPS:
            raise ValueError(f"未知按键事件：{event!r}")
        direction = np.asarray(AXIS_STEPS[event], dtype=np.float64)
        object.__setattr__(
            self,
            "_delta",
            self._delta + direction * (self.step_mm / 1000.0),
        )
        return self.pose()
