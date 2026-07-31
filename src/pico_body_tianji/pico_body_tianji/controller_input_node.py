from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray

from tianji_world_output.config_loader import TianjiConfig

from .controller_mapper import ControllerTargets, ControllerTeleopMapper
from .controller_source import XRoboControllerSource
from .freshness import FreshnessGate
from .qos import LATCHED_QOS
from .teleop_state import TeleopStateMachine
from .visualization import (
    arm_angle_constraint_markers,
    body_skeleton_markers,
    input_endpoint_markers,
    smpl_skeleton_markers,
)


class PicoControllerInputNode(Node):
    """PICO 双手柄相对末端遥操作输入，仅发布隔离的预览目标。"""

    def __init__(self):
        super().__init__("pico_controller_input")
        self.declare_parameter("rate", 90.0)
        self.declare_parameter("stale_timeout", 0.5)
        self.declare_parameter("require_reliable_timestamp", True)
        self.declare_parameter("allow_unstamped_preview", False)
        self.declare_parameter("min_cutoff", 1.0)
        self.declare_parameter("beta", 0.7)
        self.declare_parameter("elbow_min_cutoff", 0.3)

        rate = float(self.get_parameter("rate").value)
        require_reliable = bool(
            self.get_parameter("require_reliable_timestamp").value
        )
        allow_unstamped = bool(
            self.get_parameter("allow_unstamped_preview").value
        )
        self._require_reliable_timestamp = require_reliable

        config = TianjiConfig.load()
        self._mapper = ControllerTeleopMapper(
            config,
            rate=rate,
            min_cutoff=float(self.get_parameter("min_cutoff").value),
            beta=float(self.get_parameter("beta").value),
            elbow_min_cutoff=float(
                self.get_parameter("elbow_min_cutoff").value
            ),
        )
        self._source = XRoboControllerSource()
        self._source.open()
        self._freshness = FreshnessGate(
            timeout_seconds=float(
                self.get_parameter("stale_timeout").value
            ),
            allow_unstamped=allow_unstamped,
        )
        self._body_freshness = FreshnessGate(
            timeout_seconds=float(
                self.get_parameter("stale_timeout").value
            ),
            # 部分 XRoboToolkit 版本不提供 Body 独立时间戳。
            # 此时必须按骨架签名独立判活，不能借用仍在刷新的手柄时钟。
            allow_unstamped=True,
        )
        self._state_machine = TeleopStateMachine()

        self._at_home = False
        self._return_complete = False
        self._last_state = None
        self._last_source_state = "unavailable"
        self._last_timestamp_ns = 0
        self._last_body_source_state = "unavailable"
        self._last_body_timestamp_ns = 0
        self._body_timestamp_fallback = False
        self._smpl_used = False
        self._last_arm_angle_deg = {"left": None, "right": None}
        self._last_raw_arm_angle_deg = {"left": None, "right": None}
        self._last_arm_angle_source = {"left": None, "right": None}
        self._last_error = None
        self._right_a_pressed = False

        self._left_pose_pub = self.create_publisher(
            PoseStamped, "/pico_body/left_arm_target_pose", 10
        )
        self._right_pose_pub = self.create_publisher(
            PoseStamped, "/pico_body/right_arm_target_pose", 10
        )
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
        self._body_marker_pub = self.create_publisher(
            MarkerArray, "/pico_body/body_keypoints", 10
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
            "PICO 双手柄相对末端预览已启动；"
            "等待仿真回报安全初始位，然后按右手柄 A 开始。"
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
        body_live = False

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
            if sample.body_frame is not None:
                self._last_body_timestamp_ns = sample.body_timestamp_ns
                self._body_timestamp_fallback = (
                    sample.body_timestamp_fallback
                )
                body_freshness = self._body_freshness.observe(
                    source_timestamp_ns=sample.body_timestamp_ns,
                    frame_signature=sample.body_frame.signature(),
                    now=now,
                )
                self._last_body_source_state = body_freshness.state
                body_live = body_freshness.allow_publish
                if (
                    body_live
                    and sample.body_timestamp_fallback
                    and self._last_body_source_state == "live_degraded"
                ):
                    self._last_body_source_state = (
                        "live_signature_fallback"
                    )
                if (
                    self._require_reliable_timestamp
                    and not body_freshness.reliable_clock
                    and not sample.body_timestamp_fallback
                ):
                    body_live = False
            else:
                self._last_body_source_state = "unavailable"
                self._body_timestamp_fallback = False
            signal_live = signal_live and body_live
        else:
            self._last_source_state = "unavailable"
            self._last_body_source_state = "unavailable"
            self._body_timestamp_fallback = False
        self._smpl_used = body_live

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
            if sample.body_frame is None:
                self.get_logger().error(
                    "SMPL 胸廓不可用，不能建立躯干相对参考系"
                )
                return
            initialized = self._mapper.initialize(
                sample.frame,
                sample.body_frame,
            )
            if initialized != {
                "pico_left_wrist",
                "pico_right_wrist",
            }:
                self.get_logger().error(
                    f"双手柄参考姿态初始化不完整：{sorted(initialized)}"
                )
                return
            self.get_logger().info(
                "右手柄 A：在实时 SMPL 胸廓系记录双手柄参考位姿，"
                "启用相对末端预览"
            )
        elif transition.action == "start_return":
            self.get_logger().warning(
                f"开始缓慢回安全初始位：{transition.reason}"
            )
        elif transition.action == "reject_start":
            self.get_logger().warning(
                f"拒绝启动遥操作：{transition.reason}"
            )

        if transition.state != self._last_state:
            self._publish_state(transition.state)

        if (
            transition.state != "teleop"
            and body_live
            and sample is not None
            and sample.body_frame is not None
        ):
            self._publish_skeleton(
                self._mapper.map_skeleton(sample.body_frame),
                self._mapper.map_controller_positions(
                    sample.frame,
                    sample.body_frame,
                ),
            )

        if (
            transition.state == "teleop"
            and signal_live
            and sample is not None
            and sample.body_frame is not None
        ):
            try:
                self._publish_targets(
                    self._mapper.map_frame(
                        sample.frame,
                        sample.body_frame,
                    )
                )
            except Exception as exc:
                self._last_error = str(exc)
                self.get_logger().error(f"手柄相对末端映射失败：{exc}")

    def _publish_state(self, state: str) -> None:
        self._last_state = state
        self._state_pub.publish(String(data=state))

    def _publish_targets(self, targets: ControllerTargets) -> None:
        stamp = self.get_clock().now().to_msg()
        self._left_pose_pub.publish(
            self._pose_message(targets.left_pose, "left_chest", stamp)
        )
        self._right_pose_pub.publish(
            self._pose_message(targets.right_pose, "right_chest", stamp)
        )
        self._left_elbow_pub.publish(
            self._vector_message(
                targets.left_elbow_direction, "left_chest", stamp
            )
        )
        self._right_elbow_pub.publish(
            self._vector_message(
                targets.right_elbow_direction, "right_chest", stamp
            )
        )
        markers = MarkerArray()
        markers.markers.extend(
            smpl_skeleton_markers(
                targets.smpl_skeleton_keypoints,
                stamp,
            )
        )
        markers.markers.extend(
            body_skeleton_markers(
                "left",
                targets.left_body_keypoints,
                stamp,
            )
        )
        markers.markers.extend(
            body_skeleton_markers(
                "right",
                targets.right_body_keypoints,
                stamp,
            )
        )
        markers.markers.extend(
            input_endpoint_markers(
                targets.smpl_skeleton_keypoints,
                targets.controller_positions,
                stamp,
            )
        )
        for side, result in (
            ("left", targets.left_arm_angle),
            ("right", targets.right_arm_angle),
        ):
            markers.markers.extend(
                arm_angle_constraint_markers(
                    side,
                    projection_point=result.projection_point,
                    physical_direction=result.physical_direction,
                    angle_deg=result.constrained_angle_deg,
                    measured_angle_deg=result.measured_angle_deg,
                    source=result.source,
                    stamp=stamp,
                )
            )
            self._last_arm_angle_deg[side] = float(
                result.constrained_angle_deg
            )
            self._last_raw_arm_angle_deg[side] = (
                None
                if result.measured_angle_deg is None
                else float(result.measured_angle_deg)
            )
            self._last_arm_angle_source[side] = result.source
        self._body_marker_pub.publish(markers)

    def _publish_skeleton(
        self,
        keypoints,
        controller_positions,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = smpl_skeleton_markers(keypoints, stamp)
        markers.extend(
            input_endpoint_markers(
                keypoints,
                controller_positions,
                stamp,
            )
        )
        self._body_marker_pub.publish(
            MarkerArray(markers=markers)
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
            "smpl_source": self._last_body_source_state,
            "smpl_timestamp_ns": self._last_body_timestamp_ns,
            "smpl_timestamp_fallback": self._body_timestamp_fallback,
            "right_a_pressed": self._right_a_pressed,
            "at_safe_home": self._at_home,
            "error": self._last_error,
            "input": "pico_controllers_plus_smpl_upper_body",
            "mapping": "controller_relative_end_pose_in_live_smpl_torso",
            "elbow_constraint": "smpl_arm_angle_on_robot_target_axis",
            "left_smpl_arm_angle_deg": self._last_arm_angle_deg["left"],
            "right_smpl_arm_angle_deg": self._last_arm_angle_deg["right"],
            "left_raw_smpl_arm_angle_deg": (
                self._last_raw_arm_angle_deg["left"]
            ),
            "right_raw_smpl_arm_angle_deg": (
                self._last_raw_arm_angle_deg["right"]
            ),
            "left_arm_angle_source": self._last_arm_angle_source["left"],
            "right_arm_angle_source": self._last_arm_angle_source["right"],
            "smpl_used": self._smpl_used,
            "scope": "preview_only",
        }
        self._status_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )

    def destroy_node(self):
        self._source.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicoControllerInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
