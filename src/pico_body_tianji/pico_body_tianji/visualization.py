from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from .body_schema import PICO_BODY_BONES, PICO_BODY_JOINT_INDEX


_MARKER_LIFETIME = Duration(nanosec=250_000_000)


def smpl_skeleton_markers(keypoints, stamp) -> list[Marker]:
    """构造 Link_Stand 坐标下的完整 24 点 SMPL 骨架。"""
    points = np.asarray(keypoints, dtype=float)
    if points.shape != (24, 3):
        raise ValueError(
            f"SMPL keypoints must have shape 24x3, got {points.shape}"
        )
    if not np.isfinite(points).all():
        raise ValueError("SMPL keypoints must be finite")

    bones = _marker(
        frame_id="Link_Stand",
        stamp=stamp,
        namespace="smpl_skeleton_bones",
        marker_id=0,
        marker_type=Marker.LINE_LIST,
    )
    for parent_index, child_index in PICO_BODY_BONES:
        bones.points.append(_point(points[parent_index]))
        bones.points.append(_point(points[child_index]))
    bones.scale.x = 0.014
    _set_color(bones, 1.0, 0.18, 0.82, 0.9)

    joints = _marker(
        frame_id="Link_Stand",
        stamp=stamp,
        namespace="smpl_skeleton_joints",
        marker_id=0,
        marker_type=Marker.SPHERE_LIST,
    )
    joints.points = [_point(position) for position in points]
    joints.scale.x = joints.scale.y = joints.scale.z = 0.034
    _set_color(joints, 0.2, 0.9, 1.0, 0.92)

    label = _marker(
        frame_id="Link_Stand",
        stamp=stamp,
        namespace="smpl_skeleton_label",
        marker_id=0,
        marker_type=Marker.TEXT_VIEW_FACING,
    )
    _set_position(label, points[15] + np.array([0.0, 0.0, 0.1]))
    label.scale.z = 0.055
    label.text = "PICO SMPL 24"
    _set_color(label, 1.0, 0.55, 0.95, 1.0)
    return [bones, joints, label]


def input_endpoint_markers(
    keypoints,
    controller_positions,
    stamp,
) -> list[Marker]:
    """区分 Body Wrist/Hand 与实际 PICO Controller 原点。"""
    points = np.asarray(keypoints, dtype=float)
    controllers = np.asarray(controller_positions, dtype=float)
    if points.shape != (24, 3) or not np.isfinite(points).all():
        raise ValueError("SMPL endpoints require finite 24x3 keypoints")
    if controllers.shape != (2, 3) or not np.isfinite(controllers).all():
        raise ValueError(
            "Controller endpoints require finite left/right 2x3 points"
        )

    wrists = points[
        [
            PICO_BODY_JOINT_INDEX["left_wrist"],
            PICO_BODY_JOINT_INDEX["right_wrist"],
        ]
    ]
    hands = points[
        [
            PICO_BODY_JOINT_INDEX["left_hand"],
            PICO_BODY_JOINT_INDEX["right_hand"],
        ]
    ]

    wrist_marker = _marker(
        frame_id="Link_Stand",
        stamp=stamp,
        namespace="smpl_wrist_endpoints",
        marker_id=0,
        marker_type=Marker.SPHERE_LIST,
    )
    wrist_marker.points = [_point(position) for position in wrists]
    wrist_marker.scale.x = wrist_marker.scale.y = 0.048
    wrist_marker.scale.z = 0.048
    _set_color(wrist_marker, 1.0, 0.9, 0.1, 1.0)

    hand_marker = _marker(
        frame_id="Link_Stand",
        stamp=stamp,
        namespace="smpl_hand_endpoints",
        marker_id=0,
        marker_type=Marker.SPHERE_LIST,
    )
    hand_marker.points = [_point(position) for position in hands]
    hand_marker.scale.x = hand_marker.scale.y = 0.056
    hand_marker.scale.z = 0.056
    _set_color(hand_marker, 1.0, 0.15, 0.8, 1.0)

    controller_marker = _marker(
        frame_id="Link_Stand",
        stamp=stamp,
        namespace="pico_controller_origins",
        marker_id=0,
        marker_type=Marker.SPHERE_LIST,
    )
    controller_marker.points = [
        _point(position) for position in controllers
    ]
    controller_marker.scale.x = controller_marker.scale.y = 0.066
    controller_marker.scale.z = 0.066
    _set_color(controller_marker, 1.0, 0.48, 0.05, 1.0)

    offsets = _marker(
        frame_id="Link_Stand",
        stamp=stamp,
        namespace="body_hand_to_controller",
        marker_id=0,
        marker_type=Marker.LINE_LIST,
    )
    for hand, controller in zip(hands, controllers):
        offsets.points.extend((_point(hand), _point(controller)))
    offsets.scale.x = 0.009
    _set_color(offsets, 1.0, 0.65, 0.2, 0.95)

    markers = [
        wrist_marker,
        hand_marker,
        controller_marker,
        offsets,
    ]
    for side_index, side in enumerate(("L", "R")):
        label = _marker(
            frame_id="Link_Stand",
            stamp=stamp,
            namespace="input_endpoint_labels",
            marker_id=side_index,
            marker_type=Marker.TEXT_VIEW_FACING,
        )
        _set_position(
            label,
            controllers[side_index] + np.array([0.0, 0.0, 0.07]),
        )
        label.scale.z = 0.035
        label.text = (
            f"{side}: Wrist 黄 / Hand 紫 / Controller 橙"
        )
        _set_color(label, 1.0, 0.8, 0.35, 1.0)
        markers.append(label)
    return markers


def body_skeleton_markers(side: str, keypoints, stamp) -> list[Marker]:
    """构造与机械臂肩点对齐的 PICO 肩—肘—腕标记。"""
    side_index = _side_index(side)
    points = np.asarray(keypoints, dtype=float)
    if points.shape != (3, 3):
        raise ValueError(
            f"Body keypoints must have shape 3x3, got {points.shape}"
        )
    if not np.isfinite(points).all():
        raise ValueError("Body keypoints must be finite")
    line = _marker(
        frame_id=f"{side}_chest",
        stamp=stamp,
        namespace="pico_body_skeleton",
        marker_id=side_index,
        marker_type=Marker.LINE_STRIP,
    )
    line.points = [_point(position) for position in points]
    line.scale.x = 0.018
    _set_color(line, 1.0, 0.15, 0.85, 0.95)

    markers = [line]
    joint_styles = (
        ("shoulder", (1.0, 0.85, 0.1)),
        ("elbow", (1.0, 0.1, 0.75)),
        ("wrist", (0.1, 0.9, 1.0)),
    )
    for joint_index, (name, color) in enumerate(joint_styles):
        marker = _marker(
            frame_id=f"{side}_chest",
            stamp=stamp,
            namespace=f"pico_body_{name}",
            marker_id=side_index,
            marker_type=Marker.SPHERE,
        )
        _set_position(marker, points[joint_index])
        marker.scale.x = marker.scale.y = marker.scale.z = 0.052
        _set_color(marker, *color, 0.95)
        markers.append(marker)

    label = _marker(
        frame_id=f"{side}_chest",
        stamp=stamp,
        namespace="pico_body_label",
        marker_id=side_index,
        marker_type=Marker.TEXT_VIEW_FACING,
    )
    _set_position(label, points[1] + np.array([0.0, 0.0, 0.07]))
    label.scale.z = 0.045
    label.text = f"PICO {side} elbow"
    _set_color(label, 1.0, 0.65, 0.95, 1.0)
    markers.append(label)
    return markers


def arm_angle_constraint_markers(
    side: str,
    *,
    projection_point,
    physical_direction,
    angle_deg: float,
    measured_angle_deg: float | None = None,
    source: str,
    stamp,
) -> list[Marker]:
    """显示 SMPL 臂角约束后的物理肘平面方向和角度。"""
    side_index = _side_index(side)
    projection = np.asarray(projection_point, dtype=float)
    direction = np.asarray(physical_direction, dtype=float)
    if projection.shape != (3,) or not np.isfinite(projection).all():
        raise ValueError("projection point must be a finite 3-vector")
    if direction.shape != (3,) or not np.isfinite(direction).all():
        raise ValueError("physical direction must be a finite 3-vector")
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm < 1e-9:
        raise ValueError("physical direction must be non-zero")
    direction = direction / direction_norm
    endpoint = projection + 0.14 * direction

    arrow = _marker(
        frame_id=f"{side}_chest",
        stamp=stamp,
        namespace="smpl_arm_angle_direction",
        marker_id=side_index,
        marker_type=Marker.ARROW,
    )
    arrow.points = [_point(projection), _point(endpoint)]
    arrow.scale.x = 0.012
    arrow.scale.y = 0.028
    arrow.scale.z = 0.04
    if side == "left":
        _set_color(arrow, 0.75, 0.25, 1.0, 0.98)
    else:
        _set_color(arrow, 0.2, 0.75, 1.0, 0.98)

    label = _marker(
        frame_id=f"{side}_chest",
        stamp=stamp,
        namespace="smpl_arm_angle_label",
        marker_id=side_index,
        marker_type=Marker.TEXT_VIEW_FACING,
    )
    _set_position(label, endpoint + np.array([0.0, 0.0, 0.055]))
    label.scale.z = 0.038
    limited = " · 限制中" if source.endswith("_limited") else ""
    if measured_angle_deg is None:
        angle_text = f"{angle_deg:+.1f}°"
    else:
        angle_text = (
            f"原始 {measured_angle_deg:+.1f}° → "
            f"约束 {angle_deg:+.1f}°"
        )
    label.text = f"SMPL {side} 臂角 {angle_text}{limited}"
    _set_color(label, 0.85, 0.7, 1.0, 1.0)
    return [arrow, label]


def robot_joint_reference_markers(side: str, stamp) -> list[Marker]:
    """标出机器人模型中实际的肩、肘和 TCP 关节位置。"""
    side_index = _side_index(side)
    suffix = "L" if side == "left" else "R"
    frames = (
        ("shoulder", f"Link1_{suffix}"),
        ("elbow", f"Link4_{suffix}"),
        ("tcp", f"TCP_Link_{suffix}"),
    )
    markers = []
    for joint_index, (name, frame_id) in enumerate(frames):
        marker = _marker(
            frame_id=frame_id,
            stamp=stamp,
            namespace=f"robot_{name}",
            marker_id=side_index,
            marker_type=Marker.SPHERE,
        )
        marker.scale.x = marker.scale.y = marker.scale.z = 0.044
        _set_color(marker, 0.2, 1.0, 0.25, 0.95)
        markers.append(marker)

    label = _marker(
        frame_id=f"Link4_{suffix}",
        stamp=stamp,
        namespace="robot_elbow_label",
        marker_id=side_index,
        marker_type=Marker.TEXT_VIEW_FACING,
    )
    label.pose.position.z = 0.065
    label.scale.z = 0.042
    label.text = f"Robot {side} elbow"
    _set_color(label, 0.65, 1.0, 0.65, 1.0)
    markers.append(label)
    return markers


def _side_index(side: str) -> int:
    if side == "left":
        return 0
    if side == "right":
        return 1
    raise ValueError(f"Unknown arm side: {side}")


def _marker(
    *,
    frame_id: str,
    stamp,
    namespace: str,
    marker_id: int,
    marker_type: int,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime = _MARKER_LIFETIME
    return marker


def _point(position) -> Point:
    point = Point()
    point.x = float(position[0])
    point.y = float(position[1])
    point.z = float(position[2])
    return point


def _set_position(marker: Marker, position) -> None:
    marker.pose.position.x = float(position[0])
    marker.pose.position.y = float(position[1])
    marker.pose.position.z = float(position[2])


def _set_color(
    marker: Marker,
    red: float,
    green: float,
    blue: float,
    alpha: float,
) -> None:
    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.color.a = alpha
