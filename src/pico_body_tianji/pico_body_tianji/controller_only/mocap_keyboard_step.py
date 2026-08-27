#!/usr/bin/env python3
"""动捕键盘控制纯逻辑：按键解析、位移累积和正面圆轨迹。

按键映射（动捕/Motive 系，x-forward / z-up）：

    上 ← ↑（ESC [ A）→ 动捕 +z     下 ← ↓（ESC [ B）→ 动捕 -z
    左 ← ←（ESC [ D）→ 动捕 +y     右 ← →（ESC [ C）→ 动捕 -y
    '1' → +x                        '0' → -x
    's' → 开始/结束                 'c' → 运行正面圆轨迹

``StepAccumulator`` 在冻结参考上保存动捕系位移。``MotiveFrontCircleTrajectory``
生成 x-y 平面的最小加加速度轨迹：先从零点上移至 ``2r``，再从
Motive ``+z`` 一侧看顺时针画整圆；半圈经过零点，整圈结束于 ``2r``。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 按键事件 → 动捕系单位方向（x, y, z）
AXIS_STEPS: dict[str, tuple[float, float, float]] = {
    "up": (0.0, 0.0, 1.0),
    "down": (0.0, 0.0, -1.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "1": (1.0, 0.0, 0.0),
    "0": (-1.0, 0.0, 0.0),
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

    def set_delta_m(self, delta_m: np.ndarray) -> np.ndarray:
        """设置动捕系累计位移并返回当前位姿。"""
        delta = np.asarray(delta_m, dtype=np.float64)
        if delta.shape != (3,) or not np.isfinite(delta).all():
            raise ValueError("delta_m 必须是有限 3 向量")
        object.__setattr__(self, "_delta", delta.copy())
        return self.pose()

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



@dataclass
class HoldToRunClock:
    """只累计 deadman 按下周期、且限制单周期跃进的可暂停时钟。"""

    elapsed_s: float = 0.0
    running: bool = False
    maximum_step_s: float | None = None
    _last_update_s: float | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.elapsed_s) or self.elapsed_s < 0.0:
            raise ValueError("elapsed_s 必须为非负有限数值")
        if (
            self.maximum_step_s is not None
            and (
                not np.isfinite(self.maximum_step_s)
                or self.maximum_step_s <= 0.0
            )
        ):
            raise ValueError("maximum_step_s 必须为正有限数值")

    def update(self, now_s: float, pressed: bool) -> float:
        now = float(now_s)
        if not np.isfinite(now):
            raise ValueError("now_s 必须为有限数值")
        if self._last_update_s is None:
            self._last_update_s = now
            self.running = bool(pressed)
            return self.elapsed_s
        if now < self._last_update_s:
            raise ValueError("now_s 不能倒退")
        interval_s = now - self._last_update_s
        if self.running:
            if self.maximum_step_s is not None:
                interval_s = min(interval_s, self.maximum_step_s)
            self.elapsed_s += interval_s
        self._last_update_s = now
        self.running = bool(pressed)
        return self.elapsed_s


@dataclass(frozen=True)
class CircleTrajectorySample:
    """正面圆轨迹在某一时刻的动捕系位移。"""

    delta_m: np.ndarray
    segment: str
    segment_progress: float
    complete: bool


@dataclass(frozen=True)
class MotiveFrontCircleTrajectory:
    """上移 ``2r`` 后在 Motive x-y 平面顺时针画整圆。

    观察方向为 Motive ``+z``（竖直上）指向原点。
    直线上升和整圆相位均使用 minimum-jerk 标量曲线，段首尾速度、
    加速度为零；圆的中途不断点。``maximum_speed_mm_s`` 是直线和圆弧上
    的最大笛卡尔速度。
    """

    radius_mm: float = 100.0
    maximum_speed_mm_s: float = 50.0

    _MINIMUM_JERK_PEAK_DERIVATIVE = 1.875

    def __post_init__(self) -> None:
        radius = float(self.radius_mm)
        speed = float(self.maximum_speed_mm_s)
        if not np.isfinite(radius) or radius <= 0.0:
            raise ValueError("radius_mm 必须为正有限数值")
        if not np.isfinite(speed) or speed <= 0.0:
            raise ValueError("maximum_speed_mm_s 必须为正有限数值")
        object.__setattr__(self, "radius_mm", radius)
        object.__setattr__(self, "maximum_speed_mm_s", speed)

    @staticmethod
    def _minimum_jerk(progress: float) -> float:
        u = float(np.clip(progress, 0.0, 1.0))
        return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))

    @property
    def rise_duration_s(self) -> float:
        rise_mm = 2.0 * self.radius_mm
        return (
            rise_mm
            * self._MINIMUM_JERK_PEAK_DERIVATIVE
            / self.maximum_speed_mm_s
        )

    @property
    def circle_duration_s(self) -> float:
        circumference_mm = 2.0 * np.pi * self.radius_mm
        return (
            circumference_mm
            * self._MINIMUM_JERK_PEAK_DERIVATIVE
            / self.maximum_speed_mm_s
        )

    @property
    def total_duration_s(self) -> float:
        return self.rise_duration_s + self.circle_duration_s

    def sample(self, elapsed_s: float) -> CircleTrajectorySample:
        """采样轨迹；负时间钳制到起点，结束后保持圆最高点。"""
        elapsed = float(elapsed_s)
        if not np.isfinite(elapsed):
            raise ValueError("elapsed_s 必须为有限数值")
        elapsed = max(0.0, elapsed)
        radius_m = self.radius_mm / 1000.0

        if elapsed < self.rise_duration_s:
            progress = elapsed / self.rise_duration_s
            blend = self._minimum_jerk(progress)
            delta = np.array(
                [0.0, 2.0 * radius_m * blend, 0.0],
                dtype=np.float64,
            )
            return CircleTrajectorySample(
                delta_m=delta,
                segment="rise",
                segment_progress=progress,
                complete=False,
            )

        complete = elapsed >= self.total_duration_s - 1.0e-9
        progress = (
            1.0
            if complete
            else float(
                np.clip(
                    (elapsed - self.rise_duration_s)
                    / self.circle_duration_s,
                    0.0,
                    1.0,
                )
            )
        )
        angle = 2.0 * np.pi * self._minimum_jerk(progress)
        delta = np.array(
            [
                radius_m * np.sin(angle),
                radius_m * (1.0 + np.cos(angle)),
                0.0,
            ],
            dtype=np.float64,
        )
        return CircleTrajectorySample(
            delta_m=delta,
            segment="complete" if complete else "circle",
            segment_progress=progress,
            complete=complete,
        )
