#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import zenoh

from pico_body_tianji.joint_state_model import urdf_joint_names
from pico_body_tianji.mujoco_joint_state import apply_joint_positions
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf
from pico_body_tianji.zenoh_util import ZenohJsonSub, key, open_session, parse_cli_args


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = (
    PROJECT_ROOT
    / "assets"
    / "marvin_m6_ccs"
    / "urdf"
    / "marvin_m6_s_ccs_696_v4_mujoco.urdf"
)

_LOG = logging.getLogger("pico_body_mujoco_viewer")


class MujocoJointMirror:
    """把隔离预览关节状态镜像到 MuJoCo qpos，不执行动力学。"""

    def __init__(self, session, model, topic: str):
        self._qpos_addresses = _qpos_addresses(model)
        self._pending: tuple[list[str], list[float]] | None = None
        self._received_once = False
        self._sub = ZenohJsonSub(
            session,
            key(topic),
            self._on_joint_state,
        )
        _LOG.info("等待只读关节状态：%s", topic)

    @property
    def received_once(self) -> bool:
        return self._received_once

    def _on_joint_state(self, msg: dict) -> None:
        self._pending = (list(msg["name"]), list(msg["position"]))

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
            _LOG.info("已接收 %d 个关节；MuJoCo 开始镜像预览", count)
        return count

    def close(self) -> None:
        try:
            self._sub.close()
        except Exception:
            pass


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
    return parse_cli_args(
        extra={
            "--urdf": {
                "type": Path,
                "default": DEFAULT_URDF,
                "help": "MuJoCo 专用 Marvin URDF",
            },
            "--topic": {
                "default": "/pico_body_sim/model_joint_states",
                "help": "JointState JSON 输入话题",
            },
        }
    )


def main() -> None:
    args = _parse_args()
    xml, assets = portable_mujoco_urdf(args.urdf)
    model = mujoco.MjModel.from_xml_string(xml, assets)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    session = open_session()
    mirror = MujocoJointMirror(session, model, args.topic)
    started = time.monotonic()
    warned = False
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [0.0, 0.0, 1.05]
            viewer.cam.distance = 2.8
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -18.0

            while viewer.is_running():
                with viewer.lock():
                    if mirror.apply_latest(data):
                        mujoco.mj_forward(model, data)
                viewer.sync()
                if (
                    not mirror.received_once
                    and not warned
                    and time.monotonic() - started > 3.0
                ):
                    _LOG.warning("尚未收到预览关节；请先运行 run_preview.sh")
                    warned = True
                time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        mirror.close()


if __name__ == "__main__":
    handler = logging.StreamHandler()
    handler.terminator = "\n\n"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=[handler],
    )
    main()
