#!/usr/bin/env python3
"""mocap 键盘步进控制节点（Zenoh 通讯版，替代 ROS 2 链路）。

不用 PICO、不回放 h5：键盘在动捕（Motive/y-up）坐标系里给机器人
末端目标增量，每次按键 10mm（可配 --step-mm）：

    上 ← 动捕 +z        下 ← 动捕 -z
    左 ← 动捕 +x        右 ← 动捕 -x
    '1' ← 动捕 +y       '0' ← 动捕 -y
    's' 开始回放（armed 时）/ 结束并回 Home（步进中）

命令经与在线 PICO 相同的映射链路（增量相对参考帧 → pico_to_robot →
world→chest → One-Euro → 1:1 目标整形）经 Zenoh 发布到
tianji_kinematic_sim（key：/pico_body/{left,right}_arm_target_pose 与
_elbow_direction，JSON 与 zenoh 分支 C++ 节点协议一致），机器人末端
每次按键移动 10mm。方向键为 raw 模式转义序列（\\x1b[A/B/C/D），由
ArrowKeyParser 解析。

默认只控制右臂（--side right）：左臂目标不发布，C++ 节点对无目标的
臂直接跳过解算，左臂保持 Home。--side both 恢复双臂同步。

身份与真机验收：status 含真机桥 host_readiness 所需字段（input/
mapping/body_tracking/motion_trackers_required/elbow_constraint/
smpl_used/scope/at_safe_home/error），liveliness 注册
tj/live/mocap_keyboard_step（不在真机桥冲突名单内），可作为真机桥
主机输入；流程见 docs/mocap_real_acceptance.md。

用法（由 scripts/run_mocap_step.sh 启动）：

    mocap_keyboard_step [--step-mm 10] [--rate 60] [--side right]
                        [--config <yaml>] [--param key:=value ...]
"""

from __future__ import annotations

import json
import logging
import sys
import threading
import time

import numpy as np

from .controller_only_mapper import (
    ControllerOnlyTargets,
    ControllerOnlyTeleopMapper,
)
from .controller_only_trace import _assert_replay_graph_is_safe
from .mocap_keyboard_step import AXIS_STEPS, ArrowKeyParser, StepAccumulator
from .raw_keyboard import raw_keyboard
from .target_conditioner import TargetConditioningSettings
from ..controller_frame import ControllerFrame
from ..zenoh_util import (
    LiveToken,
    ZenohPub,
    key,
    load_node_config,
    load_tianji_config,
    open_session,
    parse_cli_args,
    parse_param_override,
    stamp_now,
)

_LOG = logging.getLogger("mocap_keyboard_step")

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

# 目标整形参数与 mocap 回放一致（1:1 验收/标定模式），修改时同步
# config/mode/controller_only/controller_only_ik.yaml 的
# mocap_keyboard_step 段。
DEFAULT_PARAMETERS = {
    "min_cutoff": 1.2,
    "beta": 0.45,
    "translation_gain": [1.0, 1.0, 1.0],
    "rotation_gain": 1.0,
    "workspace_relative_radii_m": [0.42, 0.38, 0.38],
    "workspace_soft_zone_ratio": 0.90,
    "maximum_linear_speed_m_s": 0.36,
    "maximum_angular_speed_rad_s": 1.55,
    "maximum_linear_acceleration_m_s2": 3.5,
    "maximum_angular_acceleration_rad_s2": 9.0,
    "left_default_zsp_direction": [
        0.45638698,
        -0.74604902,
        -0.48489358,
    ],
    "right_default_zsp_direction": [
        0.45638698,
        0.74604902,
        -0.48489358,
    ],
}


class MocapKeyboardStepNode:
    """非 ROS 的步进控制驱动：由调用方创建 zenoh.Session 并注入。"""

    def __init__(
        self,
        session,
        params: dict,
        *,
        step_mm: float = 10.0,
        rate: float = 60.0,
        side: str = "right",
    ) -> None:
        if step_mm <= 0.0:
            raise ValueError("step_mm must be positive")
        if rate <= 0.0:
            raise ValueError("rate must be positive")
        if side not in ("right", "both"):
            raise ValueError(f"side 必须是 right/both 之一，实际 {side!r}")
        self._session = session
        self._step_mm = float(step_mm)
        self._rate = rate
        self._side = side
        # 默认只控制右臂；both 时双臂同步。
        self._sides = ("right",) if side == "right" else ("left", "right")

        conditioning_settings = TargetConditioningSettings(
            rate_hz=rate,
            translation_gain=params["translation_gain"],
            rotation_gain=float(params["rotation_gain"]),
            workspace_relative_radii_m=params[
                "workspace_relative_radii_m"
            ],
            workspace_soft_zone_ratio=float(
                params["workspace_soft_zone_ratio"]
            ),
            maximum_linear_speed_m_s=float(
                params["maximum_linear_speed_m_s"]
            ),
            maximum_angular_speed_rad_s=float(
                params["maximum_angular_speed_rad_s"]
            ),
            maximum_linear_acceleration_m_s2=float(
                params["maximum_linear_acceleration_m_s2"]
            ),
            maximum_angular_acceleration_rad_s2=float(
                params["maximum_angular_acceleration_rad_s2"]
            ),
        )
        self._mapper = ControllerOnlyTeleopMapper(
            load_tianji_config(),
            rate=rate,
            min_cutoff=float(params["min_cutoff"]),
            beta=float(params["beta"]),
            conditioning_settings=conditioning_settings,
            default_zsp_directions={
                side: params[f"{side}_default_zsp_direction"]
                for side in ("left", "right")
            },
        )
        self._accumulator = StepAccumulator(
            reference_pose=_REFERENCE_POSE, step_mm=step_mm
        )
        self._parser = ArrowKeyParser()

        self._pose_pubs = {
            side: ZenohPub(
                session, key(f"/pico_body/{side}_arm_target_pose")
            )
            for side in ("left", "right")
        }
        self._elbow_pubs = {
            side: ZenohPub(
                session,
                key(f"/pico_body/{side}_arm_elbow_direction"),
            )
            for side in ("left", "right")
        }
        self._state_pub = ZenohPub(session, key("/pico_body/teleop_state"))
        self._status_pub = ZenohPub(session, key("/pico_body/status"))
        self._live = LiveToken(session, "mocap_keyboard_step")

        self._phase = "armed"
        self._phase_started = time.monotonic()
        self._pending_pose: np.ndarray | None = None
        self._settle_frames = 0
        self._last_conditioning: dict[str, object] = {
            "left": None,
            "right": None,
        }
        self._stop_event = threading.Event()
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True
        )
        self._keyboard_thread.start()

        side_label = "仅右臂" if side == "right" else "双臂同步"
        _LOG.info(
            "mocap 键盘步进已就绪：每次按键 %g mm（动捕系），控制 %s；"
            "按 s 开始，步进中再按 s 结束回 Home",
            step_mm,
            side_label,
        )

    # -- 键盘 -----------------------------------------------------------------

    def _keyboard_loop(self) -> None:
        raw_keyboard(self._on_key, self._stop_event)

    def _on_key(self, byte: str) -> None:
        event = self._parser.feed(byte)
        if event is None:
            return
        if event == "s":
            if self._phase == "armed":
                self._phase = "stepping"
                self._phase_started = time.monotonic()
                # 只控制右臂时，左臂恒用参考位姿（增量恒为零，左目标
                # 不发布，IK 左臂保持 Home）。
                self._mapper.initialize(
                    ControllerFrame.from_poses(
                        self._accumulator.pose(),
                        self._accumulator.pose(),
                    )
                )
                self._publish_state("teleop")
                _LOG.info("键盘 's'：开始步进（参考位姿已记录）")
            elif self._phase == "stepping":
                self._phase = "returning"
                self._phase_started = time.monotonic()
                _LOG.info("键盘 's'：请求结束并回 Home")
            return
        if self._phase != "stepping" or event not in AXIS_STEPS:
            return
        pose = self._accumulator.step(event)
        # 进入 settle：_tick 在 60Hz 持续映射该位姿，让滤波/整形收敛。
        self._pending_pose = pose
        self._settle_frames = _SETTLE_FRAMES
        delta_mm = self._accumulator.delta_m() * 1000.0
        _LOG.info(
            "按键 %s（%s）：+%g mm → 累积 (%+.1f, %+.1f, %+.1f) mm",
            event,
            _AXIS_LABELS[event],
            self._step_mm,
            delta_mm[0],
            delta_mm[1],
            delta_mm[2],
        )

    def _stop(self) -> None:
        self._stop_event.set()

    # -- 发布 -----------------------------------------------------------------

    def _publish_state(self, state: str) -> None:
        self._state_pub.put_text(state)
        self._status_pub.put_json(
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
                "side": self._side,
                "error": None,
            }
        )

    def _frame(self, pose: np.ndarray) -> ControllerFrame:
        """构造映射帧：只控制右臂时左臂恒用参考位姿。"""
        left_pose = (
            self._accumulator.reference_pose
            if self._side == "right" else pose
        )
        return ControllerFrame.from_poses(left_pose, pose)

    def _pose_message(
        self, pose: np.ndarray, frame_id: str, stamp: dict
    ) -> dict:
        return {
            "stamp": stamp,
            "frame_id": frame_id,
            "position": {
                "x": float(pose[0]),
                "y": float(pose[1]),
                "z": float(pose[2]),
            },
            "orientation": {
                "x": float(pose[3]),
                "y": float(pose[4]),
                "z": float(pose[5]),
                "w": float(pose[6]),
            },
        }

    def _vector_message(
        self, direction: np.ndarray, frame_id: str, stamp: dict
    ) -> dict:
        return {
            "stamp": stamp,
            "frame_id": frame_id,
            "vector": {
                "x": float(direction[0]),
                "y": float(direction[1]),
                "z": float(direction[2]),
            },
        }

    def _publish_targets(self, targets: ControllerOnlyTargets) -> None:
        stamp = stamp_now()
        for side in self._sides:
            pose = targets.left_pose if side == "left" else targets.right_pose
            self._pose_pubs[side].put_json(
                self._pose_message(pose, f"{side}_chest", stamp)
            )
            direction = (
                targets.left_default_elbow_direction
                if side == "left"
                else targets.right_default_elbow_direction
            )
            self._elbow_pubs[side].put_json(
                self._vector_message(
                    direction, f"{side}_chest", stamp
                )
            )

    def _tick(self) -> bool:
        """60Hz 映射一帧；返回 False 表示流程结束（已请求回 Home）。"""
        if self._phase == "armed":
            self._publish_state("idle")
            return True
        if self._phase == "returning":
            self._publish_state("returning")
            if time.monotonic() - self._phase_started >= 3.0:
                _LOG.info("键盘步进结束并已请求回 Home")
                return False
            return True
        self._publish_state("teleop")
        if self._pending_pose is not None:
            # settle：持续映射按键后的目标位姿，直到滤波/整形收敛。
            try:
                targets = self._mapper.map_frame(
                    self._frame(self._pending_pose)
                )
            except Exception as exc:
                _LOG.error("步进映射失败：%s", exc)
                self._pending_pose = None
                return True
            self._publish_targets(targets)
            self._last_conditioning = {
                "left": targets.left_conditioning.as_dict(),
                "right": targets.right_conditioning.as_dict(),
            }
            self._settle_frames -= 1
            if self._settle_frames <= 0:
                self._pending_pose = None
        return True

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
            "side": self._side,
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
        self._status_pub.put_json(status)

    def run(self) -> int:
        """主循环：rate Hz 映射 + 0.5s 状态；结束返回 0。"""
        tick_interval = 1.0 / self._rate
        status_interval = 0.5
        next_tick = time.monotonic() + tick_interval
        next_status = next_tick + status_interval
        while True:
            now = time.monotonic()
            if now >= next_tick:
                if not self._tick():
                    return 0
                next_tick += tick_interval
            if now >= next_status:
                self._publish_status()
                next_status += status_interval
            time.sleep(
                max(0.001, min(next_tick, next_status) - time.monotonic())
            )

    def close(self) -> None:
        try:
            self._stop()
        finally:
            try:
                for pub in (
                    *self._pose_pubs.values(),
                    *self._elbow_pubs.values(),
                    self._state_pub,
                    self._status_pub,
                ):
                    pub.close()
            finally:
                try:
                    self._live.close()
                finally:
                    self._session.close()


def main(argv=None) -> int:
    args = parse_cli_args(
        extra={
            "--step-mm": {
                "type": float,
                "default": 10.0,
                "help": "每次按键位移毫米（默认 10）",
            },
            "--rate": {
                "type": float,
                "default": 60.0,
                "help": "映射器采样率 Hz（默认 60）",
            },
            "--side": {
                "choices": ("right", "both"),
                "default": "right",
                "help": "控制侧（默认 right：仅右臂，左臂保持 Home；"
                        "both：双臂同步）",
            },
        }
    )
    overrides = {}
    for spec in args.param:
        k, v = parse_param_override(spec)
        overrides[k] = v
    params = load_node_config(
        args.config,
        "mocap_keyboard_step",
        DEFAULT_PARAMETERS,
        overrides,
    )
    session = open_session()
    _assert_replay_graph_is_safe(session)
    node = MocapKeyboardStepNode(
        session,
        params,
        step_mm=args.step_mm,
        rate=args.rate,
        side=args.side,
    )
    try:
        _LOG.warning(
            "等待键盘 's' 开始步进；步进中方向键/1/0 每次移动 %g mm，"
            "再按 's' 结束回 Home；该身份可配合真机桥做验收",
            args.step_mm,
        )
        return node.run()
    except KeyboardInterrupt:
        return 0
    finally:
        node.close()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    raise SystemExit(main())
