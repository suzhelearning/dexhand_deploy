#!/usr/bin/env python3
"""mocap 动捕实时位姿驱动节点（Zenoh 通讯版，无 ROS）。

订阅 Motive 实时手腕刚体位姿（natnet-zenoh publisher →
`mocap/hands/frame`，Motive y-up 右手系、米制），基于动捕系位姿
增量驱动机器人末端：

    's' 记录当前动捕位姿为参考并开始（等效在线链路按 A 键）
    's' 结束并回 Home（步进/跟随中）
    'q' / Ctrl+C 退出（跟随中先回 Home 再退出）

实时帧处理：取最新帧左右腕刚体（--left-rigid-id/--right-rigid-id，
默认 1/2，与采集项目 config.yaml 的物理位置一致）→ 相对参考的
增量 → pico_to_robot → world→chest → One-Euro → 1:1 目标整形 →
经 Zenoh 发布到 tianji_kinematic_sim（/pico_body/{left,right}
_arm_target_pose 与 elbow_direction）。单侧 tracking_valid 为
False 或刚体缺失时该侧不发目标（IK 保持当前关节角，真机桥按
命令超时软停）。

身份与真机验收：liveliness 注册 tj/live/mocap_live；status 含真机
桥 host_readiness 所需字段（input=mocap_live、scope=mocap_live、
source=live、motion_trackers_required=True），可作为真机桥主机输入。

用法（由 scripts/run_mocap_live.sh 启动）：

    mocap_live [--left-rigid-id 1] [--right-rigid-id 2] [--rate 60]
               [--config <yaml>] [--param key:=value ...]
"""

from __future__ import annotations

import json
import logging
import threading
import time

import numpy as np

from .controller_only_mapper import (
    ControllerOnlyTargets,
    ControllerOnlyTeleopMapper,
)
from .controller_only_trace import _assert_replay_graph_is_safe
from .raw_keyboard import raw_keyboard
from .target_conditioner import TargetConditioningSettings
from ..controller_frame import ControllerFrame
from ..zenoh_util import (
    LiveToken,
    ZenohJsonSub,
    ZenohPub,
    key,
    load_node_config,
    load_tianji_config,
    open_session,
    parse_cli_args,
    parse_param_override,
    stamp_now,
)

_LOG = logging.getLogger("mocap_live")

FRAME_KEY = "mocap/hands/frame"
RIGID_BODY_NAMES_KEY = "mocap/rigid_body_names"

# 跟随中单侧失效容忍：超过该秒数未收到有效动捕帧即整体停止映射
# （真机桥侧另有命令超时软停保护）。
_FRAME_STALE_S = 0.5

_ECHO_SYMBOLS = {
    "s": "s",
    "q": "q",
}

# 目标整形参数与 mocap 回放/键盘步进一致（1:1 验收/标定模式），
# 修改时同步 config/mode/controller_only/controller_only_ik.yaml 的
# mocap_live 段。
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


class MocapLiveNode:
    """Motive 实时位姿驱动：订阅动捕帧 → 参考增量映射 → 发布目标。"""

    def __init__(
        self,
        session,
        params: dict,
        *,
        left_rigid_id: int = 1,
        right_rigid_id: int = 2,
        rate: float = 60.0,
        side: str = "right",
    ) -> None:
        if rate <= 0.0:
            raise ValueError("rate must be positive")
        for label, spec in (
            ("left_rigid_id", left_rigid_id),
            ("right_rigid_id", right_rigid_id),
        ):
            if isinstance(spec, int):
                if spec <= 0:
                    raise ValueError(f"{label} must be positive")
            elif not isinstance(spec, str) or not spec.strip():
                raise ValueError(
                    f"{label} 必须是正整数或刚体名，实际 {spec!r}"
                )
        if side not in ("right", "both"):
            raise ValueError(f"side 必须是 right/both 之一，实际 {side!r}")
        self._session = session
        self._rate = rate
        self._side = side
        self._rigid_ids = {
            "left": int(left_rigid_id)
            if isinstance(left_rigid_id, int)
            else str(left_rigid_id).strip(),
            "right": int(right_rigid_id)
            if isinstance(right_rigid_id, int)
            else str(right_rigid_id).strip(),
        }

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
        self._live = LiveToken(session, "mocap_live")

        # 最新动捕帧（订阅回调写入，tick 读取）。
        self._frame_lock = threading.Lock()
        self._latest_frame: dict | None = None
        self._latest_received_monotonic = 0.0
        self._frame_sub = ZenohJsonSub(
            session, FRAME_KEY, self._on_mocap_frame
        )
        # 刚体名 → ID（natnet-zenoh 发布，异步到达；参数可用名字）。
        self._rigid_body_names: dict[int, str] = {}
        self._names_sub = ZenohJsonSub(
            session, RIGID_BODY_NAMES_KEY, self._on_rigid_body_names
        )
        self._missing_rigid_warned: set[str] = set()

        self._phase = "armed"
        self._phase_started = time.monotonic()
        self._reference: dict[str, np.ndarray] | None = None
        self._last_conditioning: dict[str, object] = {
            "left": None,
            "right": None,
        }
        self._quit = False
        self._stop_event = threading.Event()
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True
        )
        self._keyboard_thread.start()

        side_label = "仅右臂" if side == "right" else "双臂同步"
        _LOG.info(
            "mocap 动捕实时驱动已就绪：订阅 %s（刚体 %s/%s），"
            "控制 %s；按 s 记录参考开始，跟随中再按 s 回 Home，q 退出",
            FRAME_KEY,
            self._rigid_ids["left"],
            self._rigid_ids["right"],
            side_label,
        )

    # -- 动捕帧 -------------------------------------------------------------

    def _on_mocap_frame(self, frame: dict) -> None:
        with self._frame_lock:
            self._latest_frame = frame
            self._latest_received_monotonic = time.monotonic()

    def _on_rigid_body_names(self, mapping: dict) -> None:
        # natnet-zenoh 发布格式：{"names": {str(id): name}}
        payload = mapping.get("names", mapping)
        with self._frame_lock:
            self._rigid_body_names = {
                int(rid): str(name)
                for rid, name in payload.items()
            }
        _LOG.info("刚体名映射已更新：%s", self._rigid_body_names)

    def _resolve_rigid_id(self, side: str) -> int | None:
        """刚体参数（int id 或名字）→ 当前 id；名字未发布返回 None。"""
        spec = self._rigid_ids[side]
        if isinstance(spec, int):
            return spec
        with self._frame_lock:
            for rid, name in self._rigid_body_names.items():
                if name == spec:
                    return rid
        if side not in self._missing_rigid_warned:
            self._missing_rigid_warned.add(side)
            _LOG.warning(
                "刚体名 %r 尚未从 %s 解析（确认 Windows natnet-zenoh "
                "发布器已运行）",
                spec,
                RIGID_BODY_NAMES_KEY,
            )
        return None

    def _side_pose(self, frame: dict, side: str) -> np.ndarray | None:
        """取单侧刚体位姿（Motive 系 7 向量）；无效/缺失返回 None。"""
        rigid_id = self._resolve_rigid_id(side)
        if rigid_id is None:
            return None
        for body in frame.get("rigid_bodies", []):
            if body.get("id") != rigid_id:
                continue
            if not body.get("tracking_valid", False):
                return None
            position = body.get("position")
            quat = body.get("quaternion_xyzw")
            if (
                not isinstance(position, (list, tuple))
                or len(position) != 3
                or not isinstance(quat, (list, tuple))
                or len(quat) != 4
            ):
                return None
            values = np.asarray(position + list(quat), dtype=np.float64)
            if not np.isfinite(values).all():
                return None
            return values
        return None

    # -- 键盘 ---------------------------------------------------------------

    # raw 模式终端无 echo；按键事件实时回显到 stdout。
    _ECHO_SYMBOLS = {
        "s": "s",
        "q": "q",
    }

    def _keyboard_loop(self) -> None:
        raw_keyboard(self._on_key, self._stop_event)

    def _echo(self, event: str) -> None:
        try:
            print(self._ECHO_SYMBOLS.get(event, event), end="", flush=True)
        except OSError:
            pass

    def _on_key(self, byte: str) -> None:
        if byte in ("\x03", "q"):
            self._echo("q")
            self._handle_interrupt()
            return
        if byte != "s":
            return
        self._echo("s")
        if self._phase == "armed":
            frame = self._latest_frame
            if frame is None:
                _LOG.warning("键盘 's'：尚未收到动捕帧，无法记录参考")
                return
            poses = {
                side: self._side_pose(frame, side)
                for side in ("left", "right")
            }
            if poses["left"] is None and poses["right"] is None:
                _LOG.warning("键盘 's'：当前帧无有效手腕刚体，拒绝开始")
                return
            # 只控制右臂时左臂恒用零点参考（增量恒零，左目标不发布，
            # IK 左臂保持 Home）。
            self._reference = {
                side: poses[side]
                for side in ("left", "right")
            }
            self._mapper.initialize(
                ControllerFrame.from_poses(
                    self._reference["left"]
                    if self._reference["left"] is not None
                    else self._reference_pose(),
                    self._reference["right"]
                    if self._reference["right"] is not None
                    else self._reference_pose(),
                )
            )
            self._phase = "stepping"
            self._phase_started = time.monotonic()
            self._publish_state("teleop")
            _LOG.info(
                "键盘 's'：参考位姿已记录（左=%s 右=%s），开始动捕跟随",
                "有效" if self._reference["left"] is not None else "缺失",
                "有效" if self._reference["right"] is not None else "缺失",
            )
        elif self._phase == "stepping":
            self._phase = "returning"
            self._phase_started = time.monotonic()
            _LOG.info("键盘 's'：请求结束并回 Home")

    def _reference_pose(self) -> np.ndarray:
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    def _handle_interrupt(self) -> None:
        """q / Ctrl+C 退出：跟随中先回 Home（安全），否则直接退出。"""
        if self._phase == "stepping":
            self._phase = "returning"
            self._phase_started = time.monotonic()
            _LOG.info("按键 q/Ctrl+C：请求结束并回 Home")
            return
        _LOG.info("按键 q/Ctrl+C：退出")
        self._quit = True
        self._stop_event.set()

    # -- 发布 ---------------------------------------------------------------

    def _publish_state(self, state: str) -> None:
        self._state_pub.put_text(state)
        self._status_pub.put_json(
            {
                "state": state,
                "source": "live",
                "input": "mocap_live",
                "scope": "mocap_live",
                "mapping":
                    "controller_relative_end_pose_conditioned_v1",
                "body_tracking": "disabled",
                "motion_trackers_required": True,
                "elbow_constraint":
                    "published_default_zsp_backend_selected",
                "smpl_used": False,
                "at_safe_home": state == "idle",
                "left_rigid_id": self._rigid_ids["left"],
                "right_rigid_id": self._rigid_ids["right"],
                "side": self._side,
                "error": None,
            }
        )

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
        sides = ("right",) if self._side == "right" else ("left", "right")
        for side in sides:
            pose = (
                targets.left_pose
                if side == "left"
                else targets.right_pose
            )
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
        """rate Hz 映射一帧；返回 False 表示流程结束（已请求回 Home）。"""
        if self._phase == "armed":
            self._publish_state("idle")
            return True
        if self._phase == "returning":
            self._publish_state("returning")
            if time.monotonic() - self._phase_started >= 3.0:
                _LOG.info("动捕跟随结束并已请求回 Home")
                return False
            return True
        self._publish_state("teleop")
        with self._frame_lock:
            frame = self._latest_frame
            received_at = self._latest_received_monotonic
        if frame is None:
            return True
        if time.monotonic() - received_at > _FRAME_STALE_S:
            _LOG.warning("动捕帧超时（>%.1fs），暂停映射", _FRAME_STALE_S)
            return True
        poses = {
            side: self._side_pose(frame, side)
            for side in ("left", "right")
        }
        if poses["left"] is None and poses["right"] is None:
            return True
        # 只控制右臂时左臂恒用零点参考（增量恒零，左目标不发布）。
        # 单侧缺失同理：该侧恒用参考位姿（增量恒零）。
        if self._side == "right":
            left_pose = self._reference_pose()
        else:
            left_pose = (
                poses["left"]
                if poses["left"] is not None
                else self._reference_pose()
            )
        right_pose = (
            poses["right"]
            if poses["right"] is not None
            else self._reference_pose()
        )
        try:
            targets = self._mapper.map_frame(
                ControllerFrame.from_poses(left_pose, right_pose)
            )
        except Exception as exc:
            _LOG.error("动捕映射失败：%s", exc)
            return True
        self._publish_targets(targets)
        self._last_conditioning = {
            "left": targets.left_conditioning.as_dict(),
            "right": targets.right_conditioning.as_dict(),
        }
        return True

    def _publish_status(self) -> None:
        with self._frame_lock:
            frame = self._latest_frame
        tracking = {
            side: (
                self._side_pose(frame, side) is not None
                if frame is not None
                else False
            )
            for side in ("left", "right")
        }
        status = {
            "phase": self._phase,
            "state": (
                "teleop" if self._phase == "stepping" else self._phase
            ),
            "source": "live",
            "input": "mocap_live",
            "scope": "mocap_live",
            "mapping": "controller_relative_end_pose_conditioned_v1",
            "elbow_constraint": "published_default_zsp_backend_selected",
            "smpl_used": False,
            "motion_trackers_required": True,
            "left_rigid_id": self._rigid_ids["left"],
            "right_rigid_id": self._rigid_ids["right"],
            "side": self._side,
            "tracking": tracking,
            "target_conditioning": self._last_conditioning,
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
            if self._quit:
                return 0
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
                self._frame_sub.close()
                self._names_sub.close()
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

    def _stop(self) -> None:
        self._stop_event.set()


def main(argv=None) -> int:
    args = parse_cli_args(
        extra={
            "--left-rigid-id": {
                "type": str,
                "default": "left_wrist",
                "help": "左臂 Motive 刚体：数字 id 或刚体名（默认 left_wrist）",
            },
            "--right-rigid-id": {
                "type": str,
                "default": "right_arm",
                "help": "右臂 Motive 刚体：数字 id 或刚体名（默认 right_arm）",
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
            "--connect-endpoint": {
                "type": str,
                "default": "",
                "help": "zenohd Router 端点（默认空=本机 scouting；"
                        "Motive 数据经 router ACL 放行的 mocap/** 转发"
                        "到 scouting 网络即可收到；仅当 scouting 不可达"
                        "时才需显式连 router，但 router ACL 不放行 "
                        "tj/live/**，会破坏真机链路 liveliness 检查）",
            },
        }
    )
    overrides = {}
    for spec in args.param:
        k, v = parse_param_override(spec)
        overrides[k] = v
    params = load_node_config(
        args.config,
        "mocap_live",
        DEFAULT_PARAMETERS,
        overrides,
    )
    if args.connect_endpoint:
        import json as _json

        import zenoh

        config = zenoh.Config.from_json5(
            _json.dumps(
                {
                    "mode": "client",
                    "connect": {
                        "endpoints": [args.connect_endpoint]
                    },
                }
            )
        )
        session = zenoh.open(config)
    else:
        session = open_session()
    _assert_replay_graph_is_safe(session)
    def _parse_rigid_spec(spec: str):
        try:
            return int(spec)
        except ValueError:
            return spec

    node = MocapLiveNode(
        session,
        params,
        left_rigid_id=_parse_rigid_spec(args.left_rigid_id),
        right_rigid_id=_parse_rigid_spec(args.right_rigid_id),
        rate=args.rate,
        side=args.side,
    )
    try:
        _LOG.warning(
            "等待动捕帧与键盘 's'；按 s 记录当前动捕位姿为参考并开始，"
            "再按 's' 结束回 Home，按 'q' 退出；"
            "该身份可配合真机桥做验收"
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
