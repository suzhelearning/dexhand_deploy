from __future__ import annotations

import json
import logging
import time

from tianji_world_output.config_loader import TianjiConfig

from .controller_only_mapper import (
    ControllerOnlyTargets,
    ControllerOnlyTeleopMapper,
)
from .controller_only_source import XRoboControllerOnlySource
from .target_conditioner import TargetConditioningSettings
from ..freshness import FreshnessGate
from ..teleop_state import TeleopStateMachine
from ..zenoh_util import (
    LatchedKey,
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

_LOG = logging.getLogger("pico_controller_only_input")

DEFAULT_PARAMETERS = {
    "rate": 90.0,
    "stale_timeout": 0.5,
    "require_reliable_timestamp": True,
    "allow_unstamped_input": False,
    "min_cutoff": 1.0,
    "beta": 0.7,
    "translation_gain": [0.75, 0.75, 0.75],
    "rotation_gain": 0.85,
    "workspace_relative_radii_m": [0.32, 0.28, 0.28],
    "workspace_soft_zone_ratio": 0.80,
    "maximum_linear_speed_m_s": 0.18,
    "maximum_angular_speed_rad_s": 0.80,
    "maximum_linear_acceleration_m_s2": 1.20,
    "maximum_angular_acceleration_rad_s2": 4.0,
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


class PicoControllerOnlyInputNode:
    """只使用 PICO 左右手柄生成双臂 IK 目标，不读取 Body。"""

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

        config = load_tianji_config()
        self._mapper = ControllerOnlyTeleopMapper(
            config,
            rate=rate,
            min_cutoff=float(params["min_cutoff"]),
            beta=float(params["beta"]),
            conditioning_settings=TargetConditioningSettings(
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
            ),
            default_zsp_directions={
                side: params[f"{side}_default_zsp_direction"]
                for side in ("left", "right")
            },
        )
        # 该模式从 SDK 层就不访问 Body，避免其状态影响手柄链路。
        self._source = XRoboControllerOnlySource()
        self._source.open()
        self._freshness = FreshnessGate(
            timeout_seconds=float(params["stale_timeout"]),
            allow_unstamped=bool(params["allow_unstamped_input"]),
        )
        self._state_machine = TeleopStateMachine()

        self._at_home = False
        self._return_complete = False
        self._last_state = None
        self._last_source_state = "unavailable"
        self._last_timestamp_ns = 0
        self._last_error = None
        self._last_conditioning = {"left": None, "right": None}
        self._right_a_pressed = False

        self._left_pose_pub = ZenohPub(
            session, key("/pico_body/left_arm_target_pose")
        )
        self._right_pose_pub = ZenohPub(
            session, key("/pico_body/right_arm_target_pose")
        )
        # 始终发布安全初始位对应的固定 ZSP。Pinocchio 可忽略它，官方 IK
        # 通过 official_use_zsp 显式决定是否用它稳定第七自由度。
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
        self._live = LiveToken(session, "pico_controller_only_input")

        self._publish_state("idle")
        self._log.info(
            "PICO 纯手柄 IK 输入已启动；不读取 Body/Motion Tracker，"
            "等待 IK 安全初始位后按右手柄 A 开始。"
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
                self._log.error(self._last_error)
                return
            self._log.info(
                "右手柄 A：已记录左右手柄参考位姿，"
                "开始纯手柄 IK 解算"
            )
        elif transition.action == "start_return":
            self._log.warning(
                f"开始缓慢回安全初始位：{transition.reason}"
            )
        elif transition.action == "reject_start":
            self._log.warning(
                f"拒绝启动纯手柄 IK：{transition.reason}"
            )

        if transition.state != self._last_state:
            self._publish_state(transition.state)

        if transition.state == "teleop" and signal_live and sample is not None:
            try:
                self._publish_targets(self._mapper.map_frame(sample.frame))
            except Exception as exc:
                self._last_error = str(exc)
                self._log.error(f"纯手柄末端映射失败：{exc}")

    def _publish_state(self, state: str) -> None:
        self._last_state = state
        self._state_pub.put_text(state)

    def _publish_targets(self, targets: ControllerOnlyTargets) -> None:
        self._last_conditioning = {
            "left": targets.left_conditioning.as_dict(),
            "right": targets.right_conditioning.as_dict(),
        }
        stamp = stamp_now()
        self._left_pose_pub.put_json(
            self._pose_message(targets.left_pose, "left_chest", stamp)
        )
        self._right_pose_pub.put_json(
            self._pose_message(targets.right_pose, "right_chest", stamp)
        )
        self._left_elbow_pub.put_json(
            self._vector_message(
                targets.left_default_elbow_direction,
                "left_chest",
                stamp,
            )
        )
        self._right_elbow_pub.put_json(
            self._vector_message(
                targets.right_default_elbow_direction,
                "right_chest",
                stamp,
            )
        )

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
            "right_a_pressed": self._right_a_pressed,
            "at_safe_home": self._at_home,
            "error": self._last_error,
            "input": "pico_controllers_only",
            "mapping": "controller_relative_end_pose_conditioned_v1",
            "body_tracking": "disabled",
            "motion_trackers_required": False,
            "elbow_constraint": "published_default_zsp_backend_selected",
            "smpl_used": False,
            "scope": "controller_only_ik",
            "target_conditioning": self._last_conditioning,
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
                self._live.close()
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
        "pico_controller_only_input",
        DEFAULT_PARAMETERS,
        overrides,
    )
    session = open_session()
    node = PicoControllerOnlyInputNode(session, params)
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
