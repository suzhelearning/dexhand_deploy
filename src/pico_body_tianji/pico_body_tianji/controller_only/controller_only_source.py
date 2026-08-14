from __future__ import annotations

from dataclasses import dataclass

from ..controller_frame import ControllerFrame


@dataclass(frozen=True)
class ControllerOnlySample:
    """一帧纯手柄 SDK 输入，不包含 Body/Motion Tracker 数据。"""

    frame: ControllerFrame
    source_timestamp_ns: int
    right_a_pressed: bool


class XRoboControllerOnlySource:
    """只读取 XRoboToolkit 左右手柄、时间戳和右手柄 A 键。"""

    def __init__(self, sdk=None):
        if sdk is None:
            import xrobotoolkit_sdk as sdk

        self._sdk = sdk
        self._opened = False

    def open(self) -> None:
        if not self._opened:
            self._sdk.init()
            self._opened = True

    def read(self) -> ControllerOnlySample | None:
        if not self._opened:
            raise RuntimeError(
                "XRoboControllerOnlySource must be opened before reading"
            )
        try:
            frame = ControllerFrame.from_poses(
                self._sdk.get_left_controller_pose(),
                self._sdk.get_right_controller_pose(),
            )
        except ValueError:
            return None

        return ControllerOnlySample(
            frame=frame,
            source_timestamp_ns=int(self._sdk.get_time_stamp_ns()),
            right_a_pressed=bool(self._sdk.get_A_button()),
        )

    def close(self) -> None:
        if self._opened:
            self._sdk.close()
            self._opened = False
