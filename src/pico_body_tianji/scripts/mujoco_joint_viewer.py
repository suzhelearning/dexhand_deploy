#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pico_body_tianji.joint_state_model import urdf_joint_names
from pico_body_tianji.mujoco_joint_state import apply_joint_positions
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = (
    PROJECT_ROOT
    / "assets"
    / "marvin_m6_ccs"
    / "urdf"
    / "marvin_m6_s_ccs_696_v4_mujoco.urdf"
)


class MujocoJointMirror(Node):
    """把隔离预览关节状态镜像到 MuJoCo qpos，不执行动力学。"""

    def __init__(self, model, topic: str):
        super().__init__("pico_body_mujoco_viewer")
        self._qpos_addresses = _qpos_addresses(model)
        self._pending: tuple[list[str], list[float]] | None = None
        self._received_once = False
        self.create_subscription(JointState, topic, self._on_joint_state, 10)
        self.get_logger().info(f"等待只读关节状态：{topic}")

    @property
    def received_once(self) -> bool:
        return self._received_once

    def _on_joint_state(self, msg: JointState) -> None:
        self._pending = (list(msg.name), list(msg.position))

    def apply_latest(self, data) -> int:
        pending = self._pending
        self._pending = None
        if pending is None:
            return 0
        names, positions = pending
        count = apply_joint_positions(
            data.qpos,
            self._qpos_addresses,
            names,
            positions,
        )
        if count and not self._received_once:
            self._received_once = True
            self.get_logger().info(
                f"已接收 {count} 个关节；MuJoCo 开始镜像预览"
            )
        return count


def _qpos_addresses(model) -> dict[str, int]:
    addresses = {}
    for name in urdf_joint_names():
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name,
        )
        if joint_id < 0:
            raise RuntimeError(f"MuJoCo 模型缺少关节：{name}")
        addresses[name] = int(model.jnt_qposadr[joint_id])
    return addresses


def _parse_args():
    parser = argparse.ArgumentParser(
        description="只读镜像 PICO Body 天机预览关节到 MuJoCo"
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF,
        help="MuJoCo 专用 Marvin URDF",
    )
    parser.add_argument(
        "--topic",
        default="/pico_body_sim/model_joint_states",
        help="sensor_msgs/JointState 输入话题",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    xml, assets = portable_mujoco_urdf(args.urdf)
    model = mujoco.MjModel.from_xml_string(xml, assets)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    rclpy.init()
    node = MujocoJointMirror(model, args.topic)
    started = time.monotonic()
    warned = False
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [0.0, 0.0, 1.05]
            viewer.cam.distance = 2.8
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -18.0

            while viewer.is_running() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
                with viewer.lock():
                    if node.apply_latest(data):
                        mujoco.mj_forward(model, data)
                viewer.sync()
                if (
                    not node.received_once
                    and not warned
                    and time.monotonic() - started > 3.0
                ):
                    node.get_logger().warning(
                        "尚未收到预览关节；请先运行 run_preview.sh"
                    )
                    warned = True
                time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
