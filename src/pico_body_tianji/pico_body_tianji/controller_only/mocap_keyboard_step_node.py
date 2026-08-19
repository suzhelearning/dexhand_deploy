#!/usr/bin/env python3
"""mocap 键盘步进控制节点（preview-only / 真机验收主机输入）。

不用 PICO、不回放 h5：键盘在动捕（Motive/y-up）坐标系里给机器人
末端目标增量，每次按键 10mm（可配 --step-mm）：

    上 ← 动捕 +z        下 ← 动捕 -z
    左 ← 动捕 +x        右 ← 动捕 -x
    '1' ← 动捕 +y       '0' ← 动捕 -y
    's' 开始回放（armed 时）/ 结束并回 Home（步进中）

命令经与 mocap 回放/在线 PICO 相同的映射链路（增量相对参考帧 →
pico_to_robot → world→chest → One-Euro → 1:1 目标整形）送入
tianji_kinematic_sim，机器人末端（双臂同步）每次按键移动 10mm。
方向键为 raw 模式转义序列（\\x1b[A/B/C/D），由 ArrowKeyParser 解析。

身份与真机验收：status 含真机 readiness 所需字段，可作为真机桥
主机输入（host_readiness 与 common.sh 显式接受该身份），真机桥
全部安全保护不变；流程见 docs/mocap_real_acceptance.md。

用法（由 scripts/run_mocap_step.sh 启动）：

    mocap_keyboard_step [--step-mm 10] [--rate 60]
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time

import numpy as np

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import String

from tianji_world_output.config_loader import TianjiConfig

from .controller_only_mapper import (
    ControllerOnlyTargets,
    ControllerOnlyTeleopMapper,
)
from .controller_only_trace import _assert_replay_graph_is_safe
from .mocap_keyboard_step import AXIS_STEPS, ArrowKeyParser, StepAccumulator
from .target_conditioner import TargetConditioningSettings
from ..controller_frame import ControllerFrame

_REFERENCE_POSE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

# 按键后保持映射的帧数：One-Euro 滤波与速度/加速度整形按 60Hz 连续
# 流设计，单帧喂入只能走 ~1mm；持续映射 0.5s（30 帧）让目标收敛到
# 完整 step_mm，机器人平滑移动 10mm。
_SETTLE_FRAMES = 30

_AXIS_LABELS = {
    "up": "动捕 +z",
    "down": "动捕 -z",
    "left": "动捕 +x",
    "right": "动捕 -x",
    "1": "动捕 +y",
    "0": "动捕 -y",
}


class MocapKeyboardStepNode:
    """非 Node 子类的步进控制驱动：由调用方创建 rclpy 节点并注入。"""

    def __init__(self, node, *, step_mm: float = 10.0, rate: float = 60.0):
        from rclpy.node import Node

        if step_mm <= 0.0:
            raise ValueError("step_mm must be positive")
        if rate <= 0.0:
            raise ValueError("rate must be positive")
        self.node: Node = node
        self._step_mm = float(step_mm)
        self._rate = rate

        conditioning_settings = TargetConditioningSettings(
            rate_hz=rate,
            translation_gain=self.node.get_parameter(
                "translation_gain"
            ).value,
            rotation_gain=float(
                self.node.get_parameter("rotation_gain").value
            ),
            workspace_relative_radii_m=self.node.get_parameter(
                "workspace_relative_radii_m"
            ).value,
            workspace_soft_zone_ratio=float(
                self.node.get_parameter("workspace_soft_zone_ratio").value
            ),
            maximum_linear_speed_m_s=float(
                self.node.get_parameter("maximum_linear_speed_m_s").value
            ),
            maximum_angular_speed_rad_s=float(
                self.node.get_parameter("maximum_angular_speed_rad_s").value
            ),
            maximum_linear_acceleration_m_s2=float(
                self.node.get_parameter(
                    "maximum_linear_acceleration_m_s2"
                ).value
            ),
            maximum_angular_acceleration_rad_s2=float(
                self.node.get_parameter(
                    "maximum_angular_acceleration_rad_s2"
                ).value
            ),
        )
        self._mapper = ControllerOnlyTeleopMapper(
            TianjiConfig.load(),
            rate=rate,
            min_cutoff=float(self.node.get_parameter("min_cutoff").value),
            beta=float(self.node.get_parameter("beta").value),
            conditioning_settings=conditioning_settings,
            default_zsp_directions={
                side: self.node.get_parameter(
                    f"{side}_default_zsp_direction"
                ).value
                for side in ("left", "right")
            },
        )
        self._accumulator = StepAccumulator(
            reference_pose=_REFERENCE_POSE, step_mm=step_mm
        )
        self._parser = ArrowKeyParser()

        self._pose_publishers = {
            side: self.node.create_publisher(
                PoseStamped, f"/pico_body/{side}_arm_target_pose", 10
            )
            for side in ("left", "right")
        }
        self._elbow_publishers = {
            side: self.node.create_publisher(
                Vector3Stamped,
                f"/pico_body/{side}_arm_elbow_direction",
                10,
            )
            for side in ("left", "right")
        }
        self._state_publisher = self.node.create_publisher(
            String, "/pico_body/teleop_state", 10
        )
        self._status_publisher = self.node.create_publisher(
            String, "/pico_body/status", 10
        )

        self._phase = "armed"
        self._phase_started = time.monotonic()
        self._pending_pose: np.ndarray | None = None
        self._settle_frames = 0
        self._last_conditioning: dict[str, object] = {
            "left": None, "right": None
        }
        self._stop_event = threading.Event()
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True
        )
        self._keyboard_thread.start()

        self.node.create_timer(1.0 / 60.0, self._tick)
        self.node.create_timer(0.5, self._publish_status)
        self.node.get_logger().info(
            f"mocap 键盘步进已就绪：每次按键 {step_mm:g} mm（动捕系），"
            "按 s 开始，步进中再按 s 结束回 Home"
        )

    # -- 键盘 -----------------------------------------------------------------

    def _keyboard_loop(self) -> None:
        from .raw_keyboard import raw_keyboard

        raw_keyboard(self._on_key, self._stop_event)

    def _on_key(self, byte: str) -> None:
        event = self._parser.feed(byte)
        if event is None:
            return
        if event == "s":
            if self._phase == "armed":
                self._phase = "stepping"
                self._phase_started = time.monotonic()
                self._mapper.initialize(
                    ControllerFrame.from_poses(
                        self._accumulator.pose(),
                        self._accumulator.pose(),
                    )
                )
                self._publish_state("teleop")
                self.node.get_logger().info(
                    "键盘 's'：开始步进（参考位姿已记录）"
                )
            elif self._phase == "stepping":
                self._phase = "returning"
                self._phase_started = time.monotonic()
                self.node.get_logger().info(
                    "键盘 's'：请求结束并回 Home"
                )
            return
        if self._phase != "stepping" or event not in AXIS_STEPS:
            return
        pose = self._accumulator.step(event)
        # 进入 settle：_tick 在 60Hz 持续映射该位姿，让滤波/整形收敛。
        self._pending_pose = pose
        self._settle_frames = _SETTLE_FRAMES
        delta_mm = self._accumulator.delta_m() * 1000.0
        self.node.get_logger().info(
            f"按键 {event}（{_AXIS_LABELS[event]}）：+{self._step_mm:g} mm "
            f"→ 累积 ({delta_mm[0]:+.1f}, {delta_mm[1]:+.1f}, "
            f"{delta_mm[2]:+.1f}) mm"
        )

    def stop(self) -> None:
        self._stop_event.set()

    # -- 发布 -----------------------------------------------------------------

    def _publish_state(self, state: str) -> None:
        self._state_publisher.publish(String(data=state))
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "state": state,
                        "source": "offline_replay",
                        "input": "mocap_keyboard_step",
                        "scope": "mocap_keyboard_step",
                        "mapping":
                            "controller_relative_end_pose_conditioned_v1",
                        "body_tracking": "disabled",
                        "motion_trackers_required": False,
                        "elbow_constraint":
                            "published_default_zsp_backend_selected",
                        "smpl_used": False,
                        "at_safe_home": state == "idle",
                        "step_mm": self._step_mm,
                        "error": None,
                    },
                    ensure_ascii=False,
                )
            )
        )

    def _publish_targets(self, targets: ControllerOnlyTargets) -> None:
        stamp = self.node.get_clock().now().to_msg()
        for side in ("left", "right"):
            pose = targets.left_pose if side == "left" else targets.right_pose
            message = PoseStamped()
            message.header.stamp = stamp
            message.header.frame_id = f"{side}_chest"
            (
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ) = map(float, pose[:3])
            (
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ) = map(float, pose[3:7])
            self._pose_publishers[side].publish(message)

            direction = (
                targets.left_default_elbow_direction
                if side == "left"
                else targets.right_default_elbow_direction
            )
            elbow = Vector3Stamped()
            elbow.header.stamp = stamp
            elbow.header.frame_id = f"{side}_chest"
            (
                elbow.vector.x,
                elbow.vector.y,
                elbow.vector.z,
            ) = map(float, direction)
            self._elbow_publishers[side].publish(elbow)

    def _tick(self) -> None:
        if self._phase == "armed":
            self._publish_state("idle")
            return
        if self._phase == "returning":
            self._publish_state("returning")
            if time.monotonic() - self._phase_started >= 3.0:
                self.node.get_logger().info(
                    "键盘步进结束并已请求回 Home"
                )
                raise SystemExit(0)
            return
        self._publish_state("teleop")
        if self._pending_pose is not None:
            # settle：持续映射按键后的目标位姿，直到滤波/整形收敛。
            try:
                targets = self._mapper.map_frame(
                    ControllerFrame.from_poses(
                        self._pending_pose, self._pending_pose
                    )
                )
            except Exception as exc:
                self.node.get_logger().error(f"步进映射失败：{exc}")
                self._pending_pose = None
                return
            self._publish_targets(targets)
            self._last_conditioning = {
                "left": targets.left_conditioning.as_dict(),
                "right": targets.right_conditioning.as_dict(),
            }
            self._settle_frames -= 1
            if self._settle_frames <= 0:
                self._pending_pose = None

    def _publish_status(self) -> None:
        delta_mm = self._accumulator.delta_m() * 1000.0
        status = {
            "phase": self._phase,
            "state": (
                "teleop" if self._phase == "stepping" else self._phase
            ),
            "source": "offline_replay",
            "input": "mocap_keyboard_step",
            "scope": "mocap_keyboard_step",
            "step_mm": self._step_mm,
            "accumulated_delta_mm": [
                float(value) for value in delta_mm
            ],
            "target_conditioning": self._last_conditioning,
            "mapping": "controller_relative_end_pose_conditioned_v1",
            "elbow_constraint": "published_default_zsp_backend_selected",
            "smpl_used": False,
            "motion_trackers_required": False,
            "error": None,
        }
        self._status_publisher.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )


def main(argv=None) -> int:
    import rclpy

    raw_argv = list(argv) if argv is not None else list(sys.argv[1:])
    ros_args: list[str] = []
    app_argv = raw_argv
    try:
        split_at = raw_argv.index("--ros-args")
    except ValueError:
        pass
    else:
        ros_args = raw_argv[split_at:]
        app_argv = raw_argv[:split_at]

    parser = argparse.ArgumentParser(
        description="mocap 键盘步进控制（动捕系 10mm/键，s 启停）"
    )
    parser.add_argument("--step-mm", type=float, default=10.0,
                        help="每次按键位移毫米（默认 10）")
    parser.add_argument("--rate", type=float, default=60.0,
                        help="映射器采样率 Hz（默认 60）")
    args = parser.parse_args(app_argv)

    rclpy.init(args=ros_args or None)
    node = rclpy.create_node("mocap_keyboard_step")
    node.declare_parameter("min_cutoff", 1.0)
    node.declare_parameter("beta", 0.7)
    node.declare_parameter("translation_gain", [1.0, 1.0, 1.0])
    node.declare_parameter("rotation_gain", 1.0)
    node.declare_parameter(
        "workspace_relative_radii_m", [0.32, 0.28, 0.28]
    )
    node.declare_parameter("workspace_soft_zone_ratio", 0.80)
    node.declare_parameter("maximum_linear_speed_m_s", 0.18)
    node.declare_parameter("maximum_angular_speed_rad_s", 0.80)
    node.declare_parameter("maximum_linear_acceleration_m_s2", 1.20)
    node.declare_parameter(
        "maximum_angular_acceleration_rad_s2", 4.0
    )
    node.declare_parameter(
        "left_default_zsp_direction",
        [0.45638698, -0.74604902, -0.48489358],
    )
    node.declare_parameter(
        "right_default_zsp_direction",
        [0.45638698, 0.74604902, -0.48489358],
    )
    driver = None
    try:
        _assert_replay_graph_is_safe(node)
        driver = MocapKeyboardStepNode(
            node, step_mm=args.step_mm, rate=args.rate
        )
        node.get_logger().warning(
            f"等待键盘 's' 开始步进；步进中方向键/1/0 每次移动 "
            f"{args.step_mm:g} mm，再按 's' 结束回 Home；"
            "该身份可配合真机桥做验收"
        )
        try:
            rclpy.spin(node)
        except SystemExit:
            return 0
    finally:
        if driver is not None:
            driver.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
