from __future__ import annotations

from dataclasses import dataclass

from .body_frame import BodyFrame
from .controller_frame import ControllerFrame


@dataclass(frozen=True)
class ControllerSample:
    frame: ControllerFrame
    source_timestamp_ns: int
    right_a_pressed: bool
    body_frame: BodyFrame | None
    body_timestamp_ns: int
    body_timestamp_fallback: bool


class XRoboControllerSource:
    """一次 SDK 会话读取双手柄与可选 SMPL Body。"""

    def __init__(self, sdk=None):
        if sdk is None:
            import xrobotoolkit_sdk as sdk

        self._sdk = sdk
        self._opened = False

    def open(self) -> None:
        if not self._opened:
            self._sdk.init()
            self._opened = True

    def read(self) -> ControllerSample | None:
        if not self._opened:
            raise RuntimeError(
                "XRoboControllerSource must be opened before reading"
            )
        try:
            frame = ControllerFrame.from_poses(
                self._sdk.get_left_controller_pose(),
                self._sdk.get_right_controller_pose(),
            )
        except ValueError:
            return None

        source_timestamp_ns = int(self._sdk.get_time_stamp_ns())
        body_frame = None
        body_timestamp_ns = 0
        body_timestamp_fallback = False
        if self._sdk.is_body_data_available():
            body_frame = BodyFrame.from_joints(
                self._sdk.get_body_joints_pose()
            )
            body_timestamp_ns = int(self._sdk.get_body_timestamp_ns())
            if body_timestamp_ns <= 0:
                body_timestamp_ns = 0
                body_timestamp_fallback = True

        return ControllerSample(
            frame=frame,
            source_timestamp_ns=source_timestamp_ns,
            right_a_pressed=bool(self._sdk.get_A_button()),
            body_frame=body_frame,
            body_timestamp_ns=body_timestamp_ns,
            body_timestamp_fallback=body_timestamp_fallback,
        )

    def close(self) -> None:
        if self._opened:
            self._sdk.close()
            self._opened = False
