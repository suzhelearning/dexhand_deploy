#!/usr/bin/env python3
"""Motive 刚体定零 + 键盘步进/正面圆轨迹节点（Zenoh，无 ROS）。

订阅 ``mocap/hands/frame`` 中的 Motive 刚体位姿（x-forward / z-up
右手系、米制）。默认使用机器人右臂末端的 ``tianji_wrist``：按 ``s`` 时
冻结当前位姿作为控制零点；开始后刚体实测运动不再反馈进目标，避免
正反馈。

键盘命令在冻结参考上生成虚拟目标，四元数保持不变：

    上/下          动捕 +z/-z
    左/右          动捕 +y/-y
    1/0            动捕 +x/-x
    c              装载 x-y 正面顺时针圆轨迹
    Enter          按住推进轨迹 / 松开立即暂停
    s              记录参考开始 / 回 Home 后重新待命
    q / Ctrl+C     回 Home 后安全退出

``c`` 只装载轨迹，不会自行运动。X11 物理 ``Return``/``KP_Enter``
按住期间轨迹时钟才推进；松开后下一个控制周期内暂停并保持目标，再按
从同一点继续。键位状态不可用时拒绝自动运动。圆心在参考点上方
100mm：最高点 +200mm，顺时针半圈经过零点，整圈结束于 +200mm。
"""

from __future__ import annotations

import json
import logging
import threading
import time

import numpy as np

from ..sources.common.target_mapper import ArmTargetBatch, EndEffectorTargetMapper
from ..controller_only.mocap_keyboard_step import (
    AXIS_STEPS,
    ArrowKeyParser,
    CircleTrajectorySample,
    HoldToRunClock,
    MotiveFrontCircleTrajectory,
    StepAccumulator,
)
from ..controller_only.raw_keyboard import X11KeyState, raw_keyboard
from ..sources.common.target_conditioner import TargetConditioningSettings
from ..sources.common.session_client import SessionClient
from ..sources.pico_controller.controller_frame import ControllerFrame
from ..sources.mocap.motive import MotiveFrame, MotiveFrameSource
from ..protocol import topics
from ..zenoh_util import (
    ZenohJsonSub,
    load_node_config,
    load_tianji_config,
    open_session,
    parse_cli_args,
    parse_param_override,
    require_single_router,
)

_LOG = logging.getLogger("mocap_live")

FRAME_KEY = topics.MOCAP_HANDS_FRAME
RIGID_BODY_NAMES_KEY = topics.MOCAP_RIGID_BODY_NAMES

# 跟随中单侧失效容忍：超过该秒数未收到有效动捕帧即整体停止映射
# （真机桥侧另有命令超时软停保护）。
_FRAME_STALE_S = 0.5

# raw 模式终端 key repeat 仍生效：按住 s 稍久会收到多个 s，第二个 s
# 会把刚开始的步进立即切换为回 Home（用户感知为“自动回 Home”）。
# stepping 开始后该窗口内的重复 s 忽略；正常“开始→回 Home”间隔远大于此。
_S_DEBOUNCE_S = 0.5


_AXIS_LABELS = {
    "up": "动捕 +z",
    "down": "动捕 -z",
    "left": "动捕 +y",
    "right": "动捕 -y",
    "1": "动捕 +x",
    "0": "动捕 -x",
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
    "circle_radius_mm": 100.0,
    "circle_maximum_speed_mm_s": 50.0,
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
    """Motive 刚体定零后，以键盘累计虚拟目标并发布机器人末端目标。"""

    def __init__(
        self,
        session,
        params: dict,
        *,
        left_rigid_id: int | str = "left_back",
        right_rigid_id: int | str = "tianji_wrist",
        rate: float = 60.0,
        side: str = "right",
        step_mm: float = 10.0,
        publisher_instance_id: str | None = None,
        router_zid: str | None = None,
        coordinator_instance_id: str | None = None,
    ) -> None:
        if rate <= 0.0:
            raise ValueError("rate must be positive")
        if not np.isfinite(step_mm) or step_mm <= 0.0:
            raise ValueError("step_mm must be positive and finite")
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
        identity_values = (publisher_instance_id, router_zid, coordinator_instance_id)
        if any(value is not None for value in identity_values):
            if not all(value for value in identity_values):
                raise ValueError("diagnostic requires component/router/coordinator identities")
            self._session_client = SessionClient(
                session,
                source="diagnostic_mocap_calibration",
                publisher_instance_id=publisher_instance_id,
                router_zid=router_zid,
                expected_coordinator_instance_id=coordinator_instance_id,
            )
            self._session_client.start()
        else:
            self._session_client = None
        self._session = session
        self._rate = rate
        self._side = side
        self._step_mm = float(step_mm)
        self._circle_plan = MotiveFrontCircleTrajectory(
            radius_mm=float(params["circle_radius_mm"]),
            maximum_speed_mm_s=float(
                params["circle_maximum_speed_mm_s"]
            ),
        )
        try:
            self._circle_deadman: X11KeyState | None = X11KeyState(
                ("Return", "KP_Enter")
            )
            self._circle_deadman_error: str | None = None
        except RuntimeError as exc:
            self._circle_deadman = None
            self._circle_deadman_error = str(exc)
            _LOG.error(
                "正面圆轨迹已禁用：无法可靠读取 Enter 按下/松开：%s",
                exc,
            )
        self._active_sides = (
            ("right",) if side == "right" else ("left", "right")
        )
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
        tianji_config = load_tianji_config()
        self._mapper = EndEffectorTargetMapper(
            tianji_config,
            rate=rate,
            min_cutoff=float(params["min_cutoff"]),
            beta=float(params["beta"]),
            conditioning_settings=conditioning_settings,
            default_zsp_directions={
                side: params[f"{side}_default_zsp_direction"]
                for side in ("left", "right")
            },
            # Motive 系(+X 左, +Z 前)与 PICO 系(+X 右, +Z 后)水平轴
            # 相差 180°，必须用独立的动捕同向映射，不能复用 pico_to_robot。
            input_to_robot=tianji_config.mocap_to_robot,
        )

        # Diagnostics never publishes a session/state/target authority.  It
        # only observes the canonical coordinator state; the optional command
        # preview remains an in-process value used by the diagnostics UI.
        self._phase_lock = threading.RLock()
        self._at_home = False
        self._return_complete = False
        self._exit_after_return = False
        self._state_sub = ZenohJsonSub(
            session, topics.SESSION_STATE, self._on_authoritative_state
        )

        # 最新动捕帧（订阅回调写入，tick 读取）。
        self._motive_source = MotiveFrameSource()
        self._frame_lock = threading.Lock()
        self._latest_frame: MotiveFrame | None = None
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
        # 按 s 后由 Motive 参考位姿构造；随后只由键盘更新。
        self._command_lock = threading.Lock()
        self._accumulators: dict[str, StepAccumulator] | None = None
        self._circle_clock: HoldToRunClock | None = None
        self._circle_sample: CircleTrajectorySample | None = None
        self._parser = ArrowKeyParser()
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
            "Motive 刚体定零键盘控制已就绪：%s（刚体 %s/%s），"
            "每键 %g mm（动捕系），控制 %s；按 s 记录零点，"
            "方向键/1/0 连续步进；按 c 装载正面圆轨迹后，必须"
            "按住 Enter 才推进，松开即暂停"
            "（r=%gmm，最高点=%gmm，峰值=%gmm/s，需按住 %.1fs）；"
            "再按 s 回 Home，q 安全退出；终端每 0.5s 显示状态",
            FRAME_KEY,
            self._rigid_ids["left"],
            self._rigid_ids["right"],
            self._step_mm,
            side_label,
            self._circle_plan.radius_mm,
            2.0 * self._circle_plan.radius_mm,
            self._circle_plan.maximum_speed_mm_s,
            self._circle_plan.total_duration_s,
        )

    # -- 动捕帧 -------------------------------------------------------------

    def _on_authoritative_state(self, payload: dict) -> None:
        try:
            state = SessionState.from_dict(payload)
        except (TypeError, ValueError):
            return
        with self._phase_lock:
            self._authoritative_state = state
            self._at_home = state.state == "idle"

    def _on_mocap_frame(self, frame: dict) -> None:
        try:
            typed = self._motive_source.parse(frame)
        except (TypeError, ValueError) as exc:
            _LOG.warning("忽略无效 Motive frame: %s", exc)
            return
        with self._frame_lock:
            self._latest_frame = typed
            self._latest_received_monotonic = time.monotonic()

    def _on_rigid_body_names(self, mapping: dict) -> None:
        try:
            names = self._motive_source.parse_names(mapping)
        except (TypeError, ValueError) as exc:
            _LOG.warning("忽略无效 Motive names: %s", exc)
            return
        with self._frame_lock:
            changed = names != self._rigid_body_names
            self._rigid_body_names = names
        if changed:
            _LOG.info("刚体名映射已更新：%s", names)

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

    def _side_pose(self, frame: MotiveFrame | dict, side: str) -> np.ndarray | None:
        """取 typed MotiveFrame 的单侧位姿（字典仅供旧单元夹具使用）。"""
        rigid_id = self._resolve_rigid_id(side)
        if isinstance(frame, MotiveFrame):
            return None if rigid_id is None else frame.rigid_pose(rigid_id)
        if not isinstance(frame, dict) or rigid_id is None:
            return None
        for body in frame.get("rigid_bodies", []):
            if not isinstance(body, dict) or body.get("id") != rigid_id:
                continue
            if body.get("tracking_valid") is not True:
                return None
            values = np.asarray(
                list(body.get("position", ())) + list(body.get("quaternion_xyzw", ())),
                dtype=np.float64,
            )
            if values.shape != (7,) or not np.isfinite(values).all():
                return None
            norm = float(np.linalg.norm(values[3:]))
            if not 0.999 <= norm <= 1.001:
                return None
            values[3:] /= norm
            return values
        return None

    # raw 模式终端无 echo；按键事件实时回显到 stdout。
    _ECHO_SYMBOLS = {
        "up": "↑",
        "down": "↓",
        "left": "←",
        "right": "→",
        "1": "1",
        "0": "0",
        "c": "c",
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
        event = self._parser.feed(byte)
        if event is None:
            return
        if event in ("\x03", "q"):
            self._echo("q")
            self._handle_interrupt()
            return
        if event == "s":
            self._echo("s")
            with self._phase_lock:
                phase = self._phase
            if phase == "armed":
                with self._frame_lock:
                    frame = self._latest_frame
                    received_at = self._latest_received_monotonic
                if frame is None:
                    _LOG.warning(
                        "键盘 's'：尚未收到动捕帧，无法记录参考"
                    )
                    return
                age_s = time.monotonic() - received_at
                if age_s > _FRAME_STALE_S:
                    _LOG.warning(
                        "键盘 's'：最新动捕帧已超时 %.2fs，拒绝记录参考",
                        age_s,
                    )
                    return
                poses = {
                    side: self._side_pose(frame, side)
                    for side in ("left", "right")
                }
                missing = [
                    side
                    for side in self._active_sides
                    if poses[side] is None
                ]
                if missing:
                    labels = ", ".join(
                        f"{side}={self._rigid_ids[side]}"
                        for side in missing
                    )
                    _LOG.warning(
                        "键盘 's'：所选 Motive 刚体无效或缺失（%s），"
                        "拒绝开始",
                        labels,
                    )
                    return
                reference_poses = {
                    side: (
                        poses[side]
                        if side in self._active_sides
                        else self._reference_pose()
                    )
                    for side in ("left", "right")
                }
                try:
                    accumulators = {
                        side: StepAccumulator(
                            reference_poses[side], self._step_mm
                        )
                        for side in ("left", "right")
                    }
                    command_frame = ControllerFrame.from_poses(
                        accumulators["left"].pose(),
                        accumulators["right"].pose(),
                    )
                    self._mapper.initialize(command_frame)
                except ValueError as exc:
                    _LOG.warning("Motive 参考位姿无效，拒绝开始：%s", exc)
                    return
                # 与 _tick 的 phase 读取+状态发布互斥。无论谁先取得锁，
                # Zenoh 上都只能是 idle→teleop，不能 teleop→迟到 idle。
                with self._phase_lock:
                    if self._phase != "armed":
                        return
                    with self._command_lock:
                        self._accumulators = accumulators
                        self._circle_clock = None
                        self._circle_sample = None
                    self._return_complete = False
                    self._at_home = False
                    self._exit_after_return = False
                    session_client = getattr(self, "_session_client", None)
                    if session_client is not None:
                        if not session_client.startup_ready:
                            _LOG.warning("diagnostic coordinator snapshot 未就绪")
                            return
                        try:
                            session_client.request_start("diagnostic_s")
                        except (RuntimeError, ValueError) as exc:
                            _LOG.warning("diagnostic start 被拒绝: %s", exc)
                            return
                        self._phase = "start_pending"
                    else:
                        self._phase = "stepping"
                        self._publish_state("teleop")
                    self._phase_started = time.monotonic()
                _LOG.info(
                    "键盘 's'：已冻结 %s 参考位姿；后续 Motive 随动"
                    "不进入目标。方向键/1/0 手动步进；保持零位时按 c "
                    "执行正面圆轨迹",
                    "/".join(self._active_sides),
                )
            elif phase == "stepping":
                with self._phase_lock:
                    if self._phase != "stepping":
                        return
                    now = time.monotonic()
                    if now - self._phase_started < _S_DEBOUNCE_S:
                        _LOG.info(
                            "键盘 's'：开始后 %.0fms 内的重复 s 已忽略"
                            "（终端 key repeat 连击）",
                            (now - self._phase_started) * 1000.0,
                        )
                        return
                    self._begin_return(exit_after_return=False)
                _LOG.info("键盘 's'：请求结束并回 Home")
            elif phase == "returning":
                _LOG.info("正在回 Home；完成后可再次按 s 开始")
            return
        if event == "c":
            self._echo("c")
            with self._phase_lock:
                if self._phase != "stepping":
                    _LOG.info("按键 'c'：请先按 s 冻结 Motive 参考零点")
                    return
                if self._side != "right":
                    _LOG.warning(
                        "按键 'c'：正面圆轨迹只允许 --side right，"
                        "拒绝双臂联动"
                    )
                    return
                if self._circle_deadman is None:
                    _LOG.error(
                        "按键 'c'：拒绝启动；Enter 松开检测不可用：%s",
                        self._circle_deadman_error,
                    )
                    return
                with self._command_lock:
                    if self._accumulators is None:
                        return
                    if self._circle_clock is not None:
                        _LOG.info("按键 'c'：正面圆轨迹已经装载")
                        return
                    current_delta = self._accumulators["right"].delta_m()
                    if np.linalg.norm(current_delta) > 1.0e-9:
                        _LOG.warning(
                            "按键 'c'：当前已偏离 Motive 零点 "
                            "(%+.1f,%+.1f,%+.1f)mm；请先回 Home、"
                            "重新按 s 定零后再执行",
                            *(current_delta * 1000.0),
                        )
                        return
                    self._circle_clock = HoldToRunClock(
                        maximum_step_s=1.0 / self._rate
                    )
                    self._circle_sample = self._circle_plan.sample(0.0)
                    self._accumulators["right"].set_delta_m(
                        self._circle_sample.delta_m
                    )
            _LOG.info(
                "按键 'c'：右臂正面圆轨迹已装载；必须一直按住 Enter "
                "才推进，松开即暂停，再按从暂停处继续。有效运动 %.1fs："
                "先上移 %.0fmm，再从 Motive +z 侧看顺时针画 "
                "r=%.0fmm 整圆，半圈回零点，结束保持在上方 %.0fmm",
                self._circle_plan.total_duration_s,
                2.0 * self._circle_plan.radius_mm,
                self._circle_plan.radius_mm,
                2.0 * self._circle_plan.radius_mm,
            )
            return
        if event not in AXIS_STEPS:
            return
        with self._phase_lock:
            if self._phase != "stepping":
                return
            self._echo(event)
            with self._command_lock:
                if self._accumulators is None:
                    return
                if self._circle_clock is not None:
                    _LOG.info(
                        "正面圆轨迹已装载；方向键已忽略，"
                        "按住 Enter 推进，按 s 可回 Home"
                    )
                    return
                for side in self._active_sides:
                    self._accumulators[side].step(event)
                delta_mm = self._accumulators[
                    self._active_sides[-1]
                ].delta_m() * 1000.0
        _LOG.info(
            "按键 %s（%s）：%g mm → 累积 (%+.1f, %+.1f, %+.1f) mm",
            event,
            _AXIS_LABELS[event],
            self._step_mm,
            delta_mm[0],
            delta_mm[1],
            delta_mm[2],
        )

    def _reference_pose(self) -> np.ndarray:
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    def _command_frame(self) -> ControllerFrame | None:
        """返回冻结参考 + 键盘增量构成的虚拟帧。"""
        with self._command_lock:
            if self._accumulators is None:
                return None
            return ControllerFrame.from_poses(
                self._accumulators["left"].pose(),
                self._accumulators["right"].pose(),
            )

    def _accumulated_delta_mm(self) -> dict[str, list[float]]:
        with self._command_lock:
            if self._accumulators is None:
                return {
                    side: [0.0, 0.0, 0.0]
                    for side in ("left", "right")
                }
            return {
                side: [
                    float(value)
                    for value in (
                        self._accumulators[side].delta_m() * 1000.0
                    )
                ]
                for side in ("left", "right")
            }

    def _advance_circle(self, now: float) -> bool:
        """按 Enter 物理保压推进轨迹；完成瞬间返回 True。"""
        with self._command_lock:
            if self._circle_clock is None or self._accumulators is None:
                return False
            try:
                pressed = (
                    self._circle_deadman is not None
                    and self._circle_deadman.is_pressed()
                )
            except RuntimeError as exc:
                pressed = False
                message = str(exc)
                if message != self._circle_deadman_error:
                    self._circle_deadman_error = message
                    _LOG.error(
                        "Enter 松开检测失效，轨迹已安全暂停：%s", exc
                    )
            was_running = self._circle_clock.running
            elapsed_s = self._circle_clock.update(now, pressed)
            if self._circle_clock.running != was_running:
                if self._circle_clock.running:
                    _LOG.info(
                        "Enter 已按住：从 %.3fs 继续正面圆轨迹",
                        elapsed_s,
                    )
                else:
                    _LOG.warning(
                        "Enter 已松开：正面圆轨迹暂停在 %.3fs",
                        elapsed_s,
                    )
            sample = self._circle_plan.sample(elapsed_s)
            self._accumulators["right"].set_delta_m(sample.delta_m)
            self._circle_sample = sample
            if sample.complete:
                self._circle_clock = None
                return True
            return False

    def _circle_status(self) -> dict[str, object]:
        with self._command_lock:
            sample = self._circle_sample
            clock = self._circle_clock
            active = clock is not None
            deadman_pressed = False if clock is None else clock.running
            elapsed_s = (
                self._circle_plan.total_duration_s
                if sample is not None and sample.complete
                else 0.0 if clock is None else clock.elapsed_s
            )
        return {
            "active": active,
            "plane": "motive_xy",
            "clockwise_view": "motive_positive_z",
            "radius_mm": self._circle_plan.radius_mm,
            "top_offset_mm": 2.0 * self._circle_plan.radius_mm,
            "maximum_speed_mm_s":
                self._circle_plan.maximum_speed_mm_s,
            "required_hold_duration_s": self._circle_plan.total_duration_s,
            "elapsed_hold_s": elapsed_s,
            "deadman_key": "Return_or_KP_Enter",
            "deadman_available": self._circle_deadman is not None,
            "deadman_pressed": deadman_pressed,
            "deadman_error": self._circle_deadman_error,
            "segment": None if sample is None else sample.segment,
            "segment_progress": (
                None if sample is None else sample.segment_progress
            ),
            "complete": False if sample is None else sample.complete,
        }

    def _begin_return(self, *, exit_after_return: bool) -> None:
        with self._phase_lock:
            with self._command_lock:
                self._circle_clock = None
            self._return_complete = False
            self._exit_after_return = exit_after_return
            session_client = getattr(self, "_session_client", None)
            if session_client is not None:
                try:
                    session_client.request_return("diagnostic_return")
                except (RuntimeError, ValueError) as exc:
                    _LOG.warning("diagnostic return intent failed: %s", exc)
            self._phase = "returning"
            self._phase_started = time.monotonic()

    def _complete_return(self) -> None:
        with self._phase_lock:
            with self._command_lock:
                self._accumulators = None
                self._circle_clock = None
                self._circle_sample = None
            self._last_conditioning = {
                "left": None,
                "right": None,
            }
            self._phase = "armed"
            self._phase_started = time.monotonic()
            self._publish_state("idle")

    def _handle_interrupt(self) -> None:
        """q / Ctrl+C：跟随或回零中先等 Home，否则直接退出。"""
        with self._phase_lock:
            if self._phase == "stepping":
                self._begin_return(exit_after_return=True)
                _LOG.info("按键 q/Ctrl+C：请求回 Home 后退出")
                return
            if self._phase == "returning":
                self._exit_after_return = True
                _LOG.info("按键 q/Ctrl+C：等待回 Home 完成后退出")
                return
            _LOG.info("按键 q/Ctrl+C：退出")
            self._quit = True
            self._stop_event.set()

    # -- 发布 ---------------------------------------------------------------

    def _publish_state(self, state: str) -> None:
        # Diagnostic state is local display state, never a protocol authority.
        self._diagnostic_state = str(state)
    def _publish_targets(self, targets: ArmTargetBatch) -> None:
        # Keep the computed preview local.  Canonical target publication is
        # reserved for product sources and the coordinator.
        self._latest_target_preview = targets
    def _tick(self) -> bool:
        """rate Hz 映射虚拟目标；仅 q 回零完成后返回 False。"""
        session_client = getattr(self, "_session_client", None)
        if session_client is not None:
            session_client.poll()
        with self._phase_lock:
            if self._phase == "armed":
                self._publish_state("idle")
                return True
            if self._phase == "start_pending":
                if session_client is not None and session_client.start_authorized:
                    self._phase = "stepping"
                elif session_client is not None and session_client.pending_intent_sequence is None:
                    self._phase = "armed"
                return True
            if self._phase == "returning":
                complete = (
                    session_client is not None
                    and session_client.return_completion_fresh
                ) or (
                    session_client is None
                    and self._return_complete
                    and self._at_home
                )
                if not complete:
                    return True
                if self._exit_after_return:
                    return False
                self._complete_return()
                return True
            self._publish_state("teleop")
            if self._advance_circle(time.monotonic()):
                _LOG.info(
                    "右臂正面圆轨迹完成：已经过参考零点并保持在"
                    " Motive +y %.0fmm",
                    2.0 * self._circle_plan.radius_mm,
                )
            # 只消费按 s 时冻结的参考 + 键盘累计增量。这里禁止读取
            # _latest_frame；right_arm 随机器人运动不能形成控制反馈。
            command_frame = self._command_frame()
            if command_frame is None:
                return True
            try:
                targets = self._mapper.map_relative_controller_frame(command_frame)
            except Exception as exc:
                _LOG.error("键盘虚拟目标映射失败：%s", exc)
                return True
            self._publish_targets(targets)
            self._last_conditioning = {
                "left": targets.left_conditioning.as_dict(),
                "right": targets.right_conditioning.as_dict(),
            }
            return True

    def _publish_status(self) -> None:
        with self._phase_lock:
            phase = self._phase
            at_home = self._at_home
            target_conditioning = self._last_conditioning
        with self._frame_lock:
            frame = self._latest_frame
            received_at = self._latest_received_monotonic
        now = time.monotonic()
        age_s = None if frame is None else max(0.0, now - received_at)
        frame_fresh = age_s is not None and age_s <= _FRAME_STALE_S
        frame_number = (
            frame.get("frame_number") if isinstance(frame, dict) else None
        )
        tracking: dict[str, bool] = {}
        motive_pose: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            pose = self._side_pose(frame, side) if frame is not None else None
            tracking[side] = frame_fresh and pose is not None
            if side not in self._active_sides:
                continue
            motive_pose[side] = {
                "frame_number": frame_number,
                "age_ms": None if age_s is None else age_s * 1000.0,
                "tracking_valid": tracking[side],
                "position_m": (
                    None
                    if pose is None
                    else [float(value) for value in pose[:3]]
                ),
                "orientation_xyzw": (
                    None
                    if pose is None
                    else [float(value) for value in pose[3:]]
                ),
            }
        accumulated_delta_mm = self._accumulated_delta_mm()
        circle_trajectory = self._circle_status()
        state = {
            "armed": "idle",
            "stepping": "teleop",
            "returning": "returning",
        }[phase]
        status = {
            "phase": phase,
            "state": state,
            "source": "live",
            "input": "mocap_live",
            "scope": "mocap_live",
            "mapping": "controller_relative_end_pose_conditioned_v1",
            "elbow_constraint": "published_default_zsp_backend_selected",
            "smpl_used": False,
            "motion_trackers_required": True,
            "at_safe_home": state == "idle" and at_home,
            "left_rigid_id": self._rigid_ids["left"],
            "right_rigid_id": self._rigid_ids["right"],
            "side": self._side,
            "control_mode": "motive_reference_keyboard_step",
            "step_mm": self._step_mm,
            "accumulated_delta_mm": accumulated_delta_mm,
            "circle_trajectory": circle_trajectory,
            "tracking": tracking,
            "motive_pose": motive_pose,
            "target_conditioning": target_conditioning,
            "error": None,
        }
        self._latest_diagnostics = status
        capture = getattr(self, "_status_pub", None)
        if capture is not None:
            capture.put_json(status)
        for side, observed in motive_pose.items():
            position = observed["position_m"]
            orientation = observed["orientation_xyzw"]
            if position is None or orientation is None:
                _LOG.info(
                    "Motive %s frame=%s tracking=invalid age=%s phase=%s",
                    side,
                    observed["frame_number"],
                    "n/a"
                    if observed["age_ms"] is None
                    else f"{observed['age_ms']:.0f}ms",
                    phase,
                )
                continue
            delta = accumulated_delta_mm[side]
            _LOG.info(
                "Motive %s frame=%s tracking=%s age=%.0fms "
                "p[m]=(%+.4f,%+.4f,%+.4f) "
                "q[xyzw]=(%+.4f,%+.4f,%+.4f,%+.4f) "
                "key_delta[mm]=(%+.1f,%+.1f,%+.1f) phase=%s",
                side,
                observed["frame_number"],
                "ok" if observed["tracking_valid"] else "stale",
                observed["age_ms"],
                position[0],
                position[1],
                position[2],
                orientation[0],
                orientation[1],
                orientation[2],
                orientation[3],
                delta[0],
                delta[1],
                delta[2],
                phase,
            )

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
        self._stop()
        for resource in (self._frame_sub, self._names_sub, self._state_sub):
            try:
                resource.close()
            except Exception:
                pass
        if self._circle_deadman is not None:
            self._circle_deadman.close()
        if self._session_client is not None:
            self._session_client.close()
        self._session.close()

    def _stop(self) -> None:
        self._stop_event.set()


def main(argv=None) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    args = parse_cli_args(
        extra={
            "--left-rigid-id": {
                "type": str,
                "default": "left_back",
                "help": "左臂 Motive 刚体：数字 id 或刚体名（默认 left_back）",
            },
            "--right-rigid-id": {
                "type": str,
                "default": "tianji_wrist",
                "help": "右臂 Motive 刚体：数字 id 或刚体名（默认 tianji_wrist）",
            },
            "--rate": {
                "type": float,
                "default": 60.0,
                "help": "映射器采样率 Hz（默认 60）",
            },
            "--step-mm": {
                "type": float,
                "default": 10.0,
                "help": "每次按键的位置步长 mm（默认 10）",
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
    def _parse_rigid_spec(spec: str):
        try:
            return int(spec)
        except ValueError:
            return spec

    import os
    instance_id = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID")
    router_zid = os.environ.get("TIANJI_ROUTER_ZID")
    coordinator_id = os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID")
    if not instance_id or not router_zid or not coordinator_id:
        raise RuntimeError(
            "TIANJI_COMPONENT_INSTANCE_ID, TIANJI_ROUTER_ZID and "
            "TIANJI_COORDINATOR_INSTANCE_ID are required"
        )
    if args.connect_endpoint:
        import json as _json
        import zenoh
        config = zenoh.Config.from_json5(
            _json.dumps(
                {"mode": "client", "connect": {"endpoints": [args.connect_endpoint]}}
            )
        )
        session = zenoh.open(config)
    else:
        session = open_session()
    require_single_router(session, router_zid)

    node = MocapLiveNode(
        session,
        params,
        left_rigid_id=_parse_rigid_spec(args.left_rigid_id),
        right_rigid_id=_parse_rigid_spec(args.right_rigid_id),
        rate=args.rate,
        side=args.side,
        step_mm=args.step_mm,
        publisher_instance_id=instance_id,
        router_zid=router_zid,
        coordinator_instance_id=coordinator_id,
    )
    try:
        _LOG.warning(
            "等待 Motive 刚体帧与键盘 's'；按 s 冻结当前 tianji_wrist "
            "位姿为零点。方向键/1/0 连续累计（每键 %gmm）；零位"
            "按 c 只装载圆轨迹，必须持续按住 Enter 才运动，松开"
            "立即暂停，再按从暂停处继续。轨迹：先上移 %.0fmm，"
            "再在 Motive x-y 平面从 +z 侧看顺时针画 r=%.0fmm "
            "整圆，半圈经过零点，结束保持在上方 %.0fmm。按 s 回 "
            "Home，按 q 安全退出",
            args.step_mm,
            2.0 * node._circle_plan.radius_mm,
            node._circle_plan.radius_mm,
            2.0 * node._circle_plan.radius_mm,
        )
        return node.run()
    except KeyboardInterrupt:
        return 0
    finally:
        node.close()

if __name__ == "__main__":
    raise SystemExit(main())
