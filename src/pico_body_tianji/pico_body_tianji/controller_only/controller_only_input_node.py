from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from tianji_world_output.config_loader import TianjiConfig

from .controller_only_mapper import (
    ControllerOnlyTargets,
    ControllerOnlyTeleopMapper,
)
from .controller_only_source import XRoboControllerOnlySource
from ..freshness import FreshnessGate
from ..qos import LATCHED_QOS
from ..teleop_state import TeleopStateMachine


class PicoControllerOnlyInputNode(Node):
    """只使用 PICO 左右手柄生成双臂 IK 目标，不读取 Body。"""

    def __init__(self):
        super().__init__("pico_controller_only_input")
        self.declare_parameter("rate", 90.0)
        self.declare_parameter("stale_timeout", 0.5)
        self.declare_parameter("require_reliable_timestamp", True)
        self.declare_parameter("allow_unstamped_input", False)
        self.declare_parameter("min_cutoff", 1.0)
        self.declare_parameter("beta", 0.7)

        rate = float(self.get_parameter("rate").value)
        if rate <= 0.0:
            raise ValueError("rate must be positive")
        self._require_reliable_timestamp = bool(
            self.get_parameter("require_reliable_timestamp").value
        )

        config = TianjiConfig.load()
        self._mapper = ControllerOnlyTeleopMapper(
            config,
            rate=rate,
            min_cutoff=float(self.get_parameter("min_cutoff").value),
            beta=float(self.get_parameter("beta").value),
        )
        # 该模式从 SDK 层就不访问 Body，避免其状态影响手柄链路。
        self._source = XRoboControllerOnlySource()
        self._source.open()
        self._freshness = FreshnessGate(
            timeout_seconds=float(
                self.get_parameter("stale_timeout").value
            ),
            allow_unstamped=bool(
                self.get_parameter("allow_unstamped_input").value
            ),
        )
        self._state_machine = TeleopStateMachine()

        self._at_home = False
        self._return_complete = False
        self._last_state = None
        self._last_source_state = "unavailable"
        self._last_timestamp_ns = 0
        self._last_error = None
        self._right_a_pressed = False

        self._left_pose_pub = self.create_publisher(
            PoseStamped, "/pico_body/left_arm_target_pose", 10
        )
        self._right_pose_pub = self.create_publisher(
            PoseStamped, "/pico_body/right_arm_target_pose", 10
        )
        # 当前 IK 节点保留该接口；controller-only 配置将 arm_angle_gain
        # 设为 0，因此固定方向不会作为求解约束。
        self._left_elbow_pub = self.create_publisher(
            Vector3Stamped, "/pico_body/left_arm_elbow_direction", 10
        )
        self._right_elbow_pub = self.create_publisher(
            Vector3Stamped, "/pico_body/right_arm_elbow_direction", 10
        )
        self._state_pub = self.create_publisher(
            String, "/pico_body/teleop_state", 10
        )
        self._status_pub = self.create_publisher(
            String, "/pico_body/status", 10
        )
        self.create_subscription(
            Bool,
            "/pico_body_sim/at_home",
            self._on_at_home,
            LATCHED_QOS,
        )
        self.create_subscription(
            Bool,
            "/pico_body_sim/return_complete",
            self._on_return_complete,
            10,
        )

        self._timer = self.create_timer(1.0 / rate, self._tick)
        self._status_timer = self.create_timer(0.5, self._publish_status)
        self._publish_state("idle")
        self.get_logger().info(
            "PICO 纯手柄 IK 输入已启动；不读取 Body/Motion Tracker，"
            "等待 IK 安全初始位后按右手柄 A 开始。"
        )

    def _on_at_home(self, msg: Bool) -> None:
        self._at_home = bool(msg.data)

    def _on_return_complete(self, msg: Bool) -> None:
        if msg.data:
            self._return_complete = True

    def _tick(self) -> None:
        now = time.monotonic()
        sample = None
        signal_live = False
        try:
            sample = self._source.read()
            self._last_error = None
        except Exception as exc:
            self._last_error = str(exc)

        if sample is not None:
            self._right_a_pressed = sample.right_a_pressed
            self._last_timestamp_ns = sample.source_timestamp_ns
            freshness = self._freshness.observe(
                source_timestamp_ns=sample.source_timestamp_ns,
                frame_signature=sample.frame.signature(),
                now=now,
            )
            self._last_source_state = freshness.state
            signal_live = freshness.allow_publish
            if (
                self._require_reliable_timestamp
                and not freshness.reliable_clock
            ):
                signal_live = False
        else:
            self._last_source_state = "unavailable"

        transition = self._state_machine.update(
            right_a_pressed=self._right_a_pressed,
            signal_live=signal_live,
            at_home=self._at_home,
            return_complete=self._return_complete,
            now=now,
        )
        self._return_complete = False

        if transition.action == "start_teleop":
            if sample is None:
                return
            initialized = self._mapper.initialize(sample.frame)
            expected = {"pico_left_wrist", "pico_right_wrist"}
            if initialized != expected:
                self._last_error = (
                    "controller-only reference initialization incomplete: "
                    f"{sorted(initialized)}"
                )
                self.get_logger().error(self._last_error)
                return
            self.get_logger().info(
                "右手柄 A：已记录左右手柄参考位姿，"
                "开始纯手柄 IK 解算"
            )
        elif transition.action == "start_return":
            self.get_logger().warning(
                f"开始缓慢回安全初始位：{transition.reason}"
            )
        elif transition.action == "reject_start":
            self.get_logger().warning(
                f"拒绝启动纯手柄 IK：{transition.reason}"
            )

        if transition.state != self._last_state:
            self._publish_state(transition.state)

        if transition.state == "teleop" and signal_live and sample is not None:
            try:
                self._publish_targets(self._mapper.map_frame(sample.frame))
            except Exception as exc:
                self._last_error = str(exc)
                self.get_logger().error(f"纯手柄末端映射失败：{exc}")

    def _publish_state(self, state: str) -> None:
        self._last_state = state
        self._state_pub.publish(String(data=state))

    def _publish_targets(self, targets: ControllerOnlyTargets) -> None:
        stamp = self.get_clock().now().to_msg()
        self._left_pose_pub.publish(
            self._pose_message(targets.left_pose, "left_chest", stamp)
        )
        self._right_pose_pub.publish(
            self._pose_message(targets.right_pose, "right_chest", stamp)
        )
        self._left_elbow_pub.publish(
            self._vector_message(
                targets.left_default_elbow_direction,
                "left_chest",
                stamp,
            )
        )
        self._right_elbow_pub.publish(
            self._vector_message(
                targets.right_default_elbow_direction,
                "right_chest",
                stamp,
            )
        )

    @staticmethod
    def _pose_message(values, frame_id: str, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(values[0])
        msg.pose.position.y = float(values[1])
        msg.pose.position.z = float(values[2])
        msg.pose.orientation.x = float(values[3])
        msg.pose.orientation.y = float(values[4])
        msg.pose.orientation.z = float(values[5])
        msg.pose.orientation.w = float(values[6])
        return msg

    @staticmethod
    def _vector_message(values, frame_id: str, stamp) -> Vector3Stamped:
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.vector.x = float(values[0])
        msg.vector.y = float(values[1])
        msg.vector.z = float(values[2])
        return msg

    def _publish_status(self) -> None:
        status = {
            "state": self._state_machine.state,
            "source": self._last_source_state,
            "source_timestamp_ns": self._last_timestamp_ns,
            "right_a_pressed": self._right_a_pressed,
            "at_safe_home": self._at_home,
            "error": self._last_error,
            "input": "pico_controllers_only",
            "mapping": "controller_relative_end_pose_fixed_reference",
            "body_tracking": "disabled",
            "motion_trackers_required": False,
            "elbow_constraint": "disabled_in_controller_only_ik_config",
            "smpl_used": False,
            "scope": "controller_only_ik",
        }
        self._status_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )

    def destroy_node(self):
        self._source.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicoControllerOnlyInputNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # SIGTERM 可能先使 context 失效，再让 executor 抛出 RCLError。
        # 只吞掉这种关闭阶段异常，运行期间的真实错误仍继续抛出。
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
