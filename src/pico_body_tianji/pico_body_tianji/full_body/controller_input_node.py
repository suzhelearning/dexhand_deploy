from __future__ import annotations

import json
import logging
import time

from tianji_world_output.config_loader import TianjiConfig

from .controller_mapper import ControllerTargets, ControllerTeleopMapper
from .controller_source import XRoboControllerSource
from ..freshness import FreshnessGate
from ..teleop_state import TeleopStateMachine
from ..zenoh_util import (
    LatchedKey,
    ZenohPub,
    key,
    load_node_config,
    open_session,
    parse_cli_args,
    parse_param_override,
    stamp_now,
)

_LOG = logging.getLogger("pico_controller_input")

DEFAULT_PARAMETERS = {
    "rate": 90.0,
    "stale_timeout": 0.5,
    "require_reliable_timestamp": True,
    "allow_unstamped_preview": False,
    "min_cutoff": 1.0,
    "beta": 0.7,
    "elbow_min_cutoff": 0.3,
}


class PicoControllerInputNode:
    """PICO 双手柄相对末端遥操作输入，仅发布隔离的预览目标。"""

    def __init__(self, session, params):
        self._session = session
        self._log = _LOG

        rate = float(params["rate"])
        if rate <= 0.0:
            raise ValueError("rate must be positive")
        self._rate = rate
        self._require_reliable_timestamp = bool(
            params["require_reliable_timestamp"]
        )
        allow_unstamped = bool(params["allow_unstamped_preview"])

        config = TianjiConfig.load()
        self._mapper = ControllerTeleopMapper(
            config,
            rate=rate,
            min_cutoff=float(params["min_cutoff"]),
            beta=float(params["beta"]),
            elbow_min_cutoff=float(params["elbow_min_cutoff"]),
        )
        self._source = XRoboControllerSource()
        self._source.open()
        self._freshness = FreshnessGate(
            timeout_seconds=float(params["stale_timeout"]),
            allow_unstamped=allow_unstamped,
        )
        self._body_freshness = FreshnessGate(
            timeout_seconds=float(params["stale_timeout"]),
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

        self._left_pose_pub = ZenohPub(
            session, key("/pico_body/left_arm_target_pose")
        )
        self._right_pose_pub = ZenohPub(
            session, key("/pico_body/right_arm_target_pose")
        )
        self._left_elbow_pub = ZenohPub(
            session, key("/pico_body/left_arm_elbow_direction")
        )
        self._right_elbow_pub = ZenohPub(
            session, key("/pico_body/right_arm_elbow_direction")
        )
        self._state_pub = ZenohPub(session, key("/pico_body/teleop_state"))
        self._status_pub = ZenohPub(session, key("/pico_body/status"))

        # 事件 + 初始值（原 transient_local latched）：启动时主动取一次。
        self._at_home_latch = LatchedKey(
            session, key("/pico_body_sim/at_home"), initial=b"false"
        )
        self._return_latch = LatchedKey(
            session, key("/pico_body_sim/return_complete")
        )
        self._session.get(
            key("/pico_body_sim/at_home"),
            self._on_at_home_query,
            timeout=1.0,
        )

        self._publish_state("idle")
        self._log.info(
            "PICO 双手柄相对末端预览已启动；"
            "等待仿真回报安全初始位，然后按右手柄 A 开始。"
        )

    def _on_at_home_query(self, reply) -> None:
        if reply.ok and reply.result.payload:
            self._on_at_home_text(bytes(reply.result.payload))

    def _on_at_home_text(self, payload: bytes) -> None:
        self._at_home = payload.decode("utf-8").strip() == "true"

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
                self._log.error(
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
                self._log.error(
                    f"双手柄参考姿态初始化不完整：{sorted(initialized)}"
                )
                return
            self._log.info(
                "右手柄 A：在实时 SMPL 胸廓系记录双手柄参考位姿，"
                "启用相对末端预览"
            )
        elif transition.action == "start_return":
            self._log.warning(
                f"开始缓慢回安全初始位：{transition.reason}"
            )
        elif transition.action == "reject_start":
            self._log.warning(
                f"拒绝启动遥操作：{transition.reason}"
            )

        if transition.state != self._last_state:
            self._publish_state(transition.state)

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
                self._log.error(f"手柄相对末端映射失败：{exc}")

    def _publish_state(self, state: str) -> None:
        self._last_state = state
        self._state_pub.put_text(state)

    def _publish_targets(self, targets: ControllerTargets) -> None:
        stamp = stamp_now()
        self._left_pose_pub.put_json(
            self._pose_message(targets.left_pose, "left_chest", stamp)
        )
        self._right_pose_pub.put_json(
            self._pose_message(targets.right_pose, "right_chest", stamp)
        )
        self._left_elbow_pub.put_json(
            self._vector_message(
                targets.left_elbow_direction, "left_chest", stamp
            )
        )
        self._right_elbow_pub.put_json(
            self._vector_message(
                targets.right_elbow_direction, "right_chest", stamp
            )
        )
        for side, result in (
            ("left", targets.left_arm_angle),
            ("right", targets.right_arm_angle),
        ):
            self._last_arm_angle_deg[side] = float(
                result.constrained_angle_deg
            )
            self._last_raw_arm_angle_deg[side] = (
                None
                if result.measured_angle_deg is None
                else float(result.measured_angle_deg)
            )
            self._last_arm_angle_source[side] = result.source

    @staticmethod
    def _pose_message(values, frame_id: str, stamp) -> dict:
        return {
            "stamp": stamp,
            "frame_id": frame_id,
            "position": {
                "x": float(values[0]),
                "y": float(values[1]),
                "z": float(values[2]),
            },
            "orientation": {
                "x": float(values[3]),
                "y": float(values[4]),
                "z": float(values[5]),
                "w": float(values[6]),
            },
        }

    @staticmethod
    def _vector_message(values, frame_id: str, stamp) -> dict:
        return {
            "stamp": stamp,
            "frame_id": frame_id,
            "vector": {
                "x": float(values[0]),
                "y": float(values[1]),
                "z": float(values[2]),
            },
        }

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
        self._status_pub.put_text(json.dumps(status, ensure_ascii=False))

    def run(self) -> None:
        """主循环：rate Hz 解算 + 0.5 s 状态。"""
        tick_interval = 1.0 / self._rate
        status_interval = 0.5
        next_tick = time.monotonic() + tick_interval
        next_status = next_tick + status_interval
        while True:
            now = time.monotonic()
            if now >= next_tick:
                self._tick()
                next_tick += tick_interval
            if now >= next_status:
                self._publish_status()
                next_status += status_interval
            time.sleep(
                max(0.001, min(next_tick, next_status) - time.monotonic())
            )

    def close(self) -> None:
        try:
            self._source.close()
        finally:
            try:
                self._at_home_latch.close()
                self._return_latch.close()
            finally:
                self._session.close()


def main(argv=None) -> None:
    args = parse_cli_args()
    overrides = {}
    for spec in args.param:
        k, v = parse_param_override(spec)
        overrides[k] = v
    params = load_node_config(
        args.config,
        "pico_controller_input",
        DEFAULT_PARAMETERS,
        overrides,
    )
    session = open_session()
    node = PicoControllerInputNode(session, params)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    main()
