#!/usr/bin/env python3
"""mocap HDF5 手腕轨迹回放节点（preview-only 轨迹跟踪仿真）。

把 mocap-acquisition HDF5（v4.0）里录制的左右手腕位姿按录制节奏
回放成纯手柄 IK 的目标轨迹：先经过与在线 PICO 完全相同的
``ControllerOnlyTeleopMapper``（增量相对映射 + One-Euro 滤波 +
目标整形），再发布到 ``/pico_body/{left,right}_arm_target_pose``，
由 ``tianji_kinematic_sim`` 跟踪，可在 RViz/MuJoCo 中观察轨迹跟踪。

状态机与 ``controller_only_trace`` 的离线回放一致：

    arming（1s，发布 idle）
      → replaying（按 h5 时间轴回放，发布 teleop）
      → returning（3s，发布 returning）
      → 请求关闭

安全约束：

- 与 trace replay 一样拒绝在存在真机桥/实时输入节点的 ROS 图中启动；
- 默认 preview-only；配合 ``--hold-arm`` 时作为真机桥主机输入做
  确定性轨迹真机验收（host_readiness 显式接受 mocap_h5_replay
  身份，见 docs/mocap_real_acceptance.md），真机桥的全部安全
  保护（回零、新鲜度、跟踪误差、急停）保持不变；
- 单侧完全无效（如 take003 左手全 NaN）时该侧保持机器人 Home，
  不阻碍另一侧回放。

用法（由 scripts/run_mocap_sim.sh 启动，也可直接）：

    mocap_h5_replay --h5 TAKE.h5 [--speed N] [--yaw-deg N]
                    [--reference-frame N] [--rate N] [--control keyboard]
                    [--hold-arm N]
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import String

from tianji_world_output.config_loader import TianjiConfig

from .controller_only_mapper import (
    ControllerOnlyTargets,
    ControllerOnlyTeleopMapper,
)
from .controller_only_trace import _assert_replay_graph_is_safe
from .mocap_h5 import (
    MocapRecording,
    apply_yaw_world,
    load_mocap_h5,
    synthetic_reference_pose,
)
from .target_conditioner import TargetConditioningSettings
from ..controller_frame import ControllerFrame


class MocapH5ReplayNode:
    """非 Node 子类的回放驱动：由调用方创建 rclpy 节点并注入。"""

    def __init__(self, node, recording: MocapRecording, *, speed: float,
                 yaw_deg: float, reference_frame: int, rate: float,
                 hold_arm_s: float = 0.0, control: str = "keyboard"):
        from rclpy.node import Node

        if speed <= 0.0:
            raise ValueError("replay speed must be positive")
        if rate <= 0.0:
            raise ValueError("mapper rate must be positive")
        if hold_arm_s < 0.0:
            raise ValueError("hold_arm_s must be non-negative")
        if control not in ("keyboard", "auto"):
            raise ValueError(
                f"control 必须是 keyboard/auto 之一，实际 {control!r}"
            )
        self.node: Node = node
        self.recording = recording
        self.speed = speed
        self.rate = rate
        self._hold_arm_s = float(hold_arm_s)
        self._control = control

        # 目标整形参数与在线纯手柄输入节点共用 YAML。
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

        # 录制数据（yaw 朝向标定在加载后立即应用）。
        self._yaw_deg = yaw_deg
        wrists = {
            side: apply_yaw_world(recording.hands[side].wrist, yaw_deg)
            for side in ("left", "right")
        }
        self._wrists = wrists
        self._valid = {
            side: recording.hands[side].valid for side in ("left", "right")
        }
        self._frame_count = recording.frame_count
        self._time0_ns = int(recording.time_ns[0])

        # 参考帧：等效于在线链路按下右手柄 A 的时刻。
        if reference_frame < 0:
            reference_frame = recording.reference_index()
        if not 0 <= reference_frame < self._frame_count:
            raise ValueError(
                f"reference_frame={reference_frame} 超出 "
                f"[0, {self._frame_count})"
            )
        self._reference_frame = reference_frame
        self._reference_pose = {
            side: self._side_reference_pose(side)
            for side in ("left", "right")
        }

        has_data = {
            side: bool(np.any(self._valid[side]))
            for side in ("left", "right")
        }
        if not any(has_data.values()):
            raise ValueError(
                "录制中左右手腕均无有效数据，无可回放内容"
            )
        for side in ("left", "right"):
            if not has_data[side]:
                self.node.get_logger().warning(
                    f"{side} 手腕无有效数据（全 NaN/无效帧），"
                    "回放期间该侧保持机器人 Home"
                )

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

        self._phase = "arming"
        self._phase_started = time.monotonic()
        self._index = 0
        self._initialized = False
        self._last_conditioning: dict[str, object] = {
            "left": None, "right": None
        }
        # 键盘控制（默认）：'s' 开始回放，回放中再按 's' 结束。
        self._key_started = False
        self._key_ended = False
        self._stop_event = threading.Event()
        self._keyboard_thread: threading.Thread | None = None
        if control == "keyboard":
            from .raw_keyboard import raw_keyboard

            self._keyboard_thread = threading.Thread(
                target=raw_keyboard,
                args=(self._on_key, self._stop_event),
                daemon=True,
            )
            self._keyboard_thread.start()
        self.node.create_timer(1.0 / 60.0, self._tick)
        self.node.create_timer(0.5, self._publish_status)
        self.node.get_logger().info(
            "mocap HDF5 回放已加载："
            f"{json.dumps(recording.summary(), ensure_ascii=False)}"
            f" yaw_deg={yaw_deg} speed={speed} "
            f"control={control} "
            f"reference_frame={reference_frame}"
        )

    def _side_reference_pose(self, side: str) -> np.ndarray:
        indices = np.flatnonzero(self._valid[side])
        if indices.size:
            return self._wrists[side][indices[0]].copy()
        return synthetic_reference_pose()

    def _frame(self, index: int) -> ControllerFrame:
        """构造第 index 帧的双手腕位姿；无效侧使用其参考位姿。"""
        left = self._wrists["left"][index] if self._valid[
            "left"][index] else self._reference_pose["left"]
        right = self._wrists["right"][index] if self._valid[
            "right"][index] else self._reference_pose["right"]
        return ControllerFrame.from_poses(left, right)

    def _publish_state(self, state: str) -> None:
        self._state_publisher.publish(String(data=state))
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "state": state,
                        "source": "offline_replay",
                        "input": "mocap_h5_replay",
                        "scope": "mocap_replay",
                        "mapping":
                            "controller_relative_end_pose_conditioned_v1",
                        "body_tracking": "disabled",
                        "motion_trackers_required": False,
                        "elbow_constraint":
                            "published_default_zsp_backend_selected",
                        "smpl_used": False,
                        "at_safe_home": state == "idle",
                        "recording": self.recording.summary(),
                        "yaw_deg": self._yaw_deg,
                        "speed": self.speed,
                        "reference_frame": self._reference_frame,
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

    def _on_key(self, key: str) -> None:
        """键盘控制：'s' 开始回放；回放中再按 's' 结束（请求回 Home）。"""
        if key != "s":
            return
        if self._phase == "arming":
            self._key_started = True
            self.node.get_logger().info("键盘 's'：开始 mocap 回放")
        elif self._phase == "replaying":
            self._key_ended = True
            self.node.get_logger().info("键盘 's'：请求结束回放并回 Home")

    def stop(self) -> None:
        """停止键盘监听线程（退出前调用，确保终端恢复）。"""
        self._stop_event.set()

    def _tick(self) -> None:
        now = time.monotonic()
        if self._phase == "arming":
            self._publish_state("idle")
            start_replay = False
            if self._control == "keyboard":
                # 等待键盘 's'（PICO A 键的替代，回放无 A 键概念）。
                start_replay = self._key_started
            else:
                # auto：arming 持续 1s + --hold-arm（等真机桥就绪）。
                start_replay = (
                    now - self._phase_started >= 1.0 + self._hold_arm_s
                )
            if start_replay:
                self._phase = "replaying"
                self._phase_started = now
                # 以参考帧初始化映射器：此后相对增量以该帧为零点。
                self._mapper.initialize(self._frame(self._reference_frame))
                self._initialized = True
                self._publish_state("teleop")
            return
        if self._phase == "returning":
            self._publish_state("returning")
            if now - self._phase_started >= 3.0:
                self.node.get_logger().info(
                    "mocap 回放完成并已请求回 Home"
                )
                # 注意：不能在这里调用 context.try_shutdown()——本机
                # rclpy(Humble) 在 executor 回调内调用它会产生死锁
                # （controller_only_trace 曾因此回放结束不退出）。
                # SystemExit 会直接从 spin 的 while 循环中抛出。
                raise SystemExit(0)
            return
        # replaying
        if self._key_ended:
            self._phase = "returning"
            self._phase_started = now
            return
        elapsed = (now - self._phase_started) * self.speed
        published = False
        while (
            self._index < self._frame_count
            and (
                float(
                    self.recording.time_ns[self._index] - self._time0_ns
                )
                / 1.0e9
                <= elapsed
            )
        ):
            try:
                targets = self._mapper.map_frame(
                    self._frame(self._index)
                )
            except Exception as exc:
                self.node.get_logger().error(
                    f"第 {self._index} 帧映射失败：{exc}"
                )
            else:
                self._publish_targets(targets)
                self._last_conditioning = {
                    "left": targets.left_conditioning.as_dict(),
                    "right": targets.right_conditioning.as_dict(),
                }
                published = True
            self._index += 1
        self._publish_state("teleop")
        if self._index >= self._frame_count:
            if not published:
                self.node.get_logger().warning(
                    "回放全程未发布任何目标（所有帧映射失败）"
                )
            self._phase = "returning"
            self._phase_started = now

    def _publish_status(self) -> None:
        status = {
            "phase": self._phase,
            "state": "teleop" if self._phase == "replaying" else self._phase,
            "source": "offline_replay",
            "input": "mocap_h5_replay",
            "scope": "mocap_replay",
            "frame_index": self._index,
            "frame_count": self._frame_count,
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

    # ROS 参数段（--ros-args ...）交给 rclpy.init 消费，与 launch
    # parameters=[...] 生成的命令行保持一致；其余参数由 argparse 解析。
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
        description="mocap HDF5 手腕轨迹回放（preview-only 轨迹跟踪仿真）"
    )
    parser.add_argument("h5", nargs="?", type=Path,
                        help="mocap-acquisition v4.0 HDF5 文件")
    parser.add_argument("--h5", dest="h5_opt", type=Path,
                        help="mocap-acquisition v4.0 HDF5 文件（等价位置参数）")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="回放倍速（默认 1.0）")
    parser.add_argument("--yaw-deg", type=float, default=0.0,
                        help="绕竖直轴旋转整条轨迹的朝向标定（度，默认 0）")
    parser.add_argument("--reference-frame", type=int, default=-1,
                        help="参考帧下标（等效按 A 时刻，默认第一个有效帧）")
    parser.add_argument("--rate", type=float, default=60.0,
                        help="映射器采样率 Hz（默认 60，与 h5 对齐）")
    parser.add_argument("--hold-arm", type=float, default=0.0,
                        dest="hold_arm_s",
                        help="auto 模式下开始回放前保持 idle 的秒数"
                             "（默认 0；配合 --control auto 使用）")
    parser.add_argument("--control", choices=("keyboard", "auto"),
                        default="keyboard",
                        help="回放开始/结束控制方式（默认 keyboard："
                             "按 's' 开始，回放中再按 's' 结束；"
                             "auto：arming 后自动开始，播完自动结束）")
    args = parser.parse_args(app_argv)

    h5_path = args.h5_opt if args.h5_opt is not None else args.h5
    if h5_path is None:
        parser.error("必须提供 mocap HDF5 文件路径")
    recording = load_mocap_h5(h5_path)

    rclpy.init(args=ros_args or None)
    node = rclpy.create_node("mocap_h5_replay")
    node.declare_parameter("min_cutoff", 1.0)
    node.declare_parameter("beta", 0.7)
    # 默认 1:1：命令位移 → 目标位移严格 1:1（轨迹跟踪验收/标定模式）。
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
    replay_driver = None
    try:
        _assert_replay_graph_is_safe(node)
        replay_driver = MocapH5ReplayNode(
            node,
            recording,
            speed=args.speed,
            yaw_deg=args.yaw_deg,
            reference_frame=args.reference_frame,
            rate=args.rate,
            hold_arm_s=args.hold_arm_s,
            control=args.control,
        )
        if args.control == "keyboard":
            node.get_logger().warning(
                "等待键盘 's' 开始回放（回放中再按 's' 结束回 Home）；"
                "该身份经 --hold-arm/真机桥流程配合时可驱动真机验收"
            )
        else:
            node.get_logger().warning(
                f"auto 模式：{args.hold_arm_s}s 后自动开始回放；"
                "该身份经 --hold-arm/真机桥流程配合时可驱动真机验收"
            )
        try:
            rclpy.spin(node)
        except SystemExit:
            return 0
    finally:
        if replay_driver is not None:
            replay_driver.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
