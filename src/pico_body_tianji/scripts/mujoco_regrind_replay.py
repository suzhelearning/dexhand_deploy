#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
from pathlib import Path
import time

import mujoco
import mujoco.viewer

from pico_body_tianji.regrind_h5 import (
    apply_regrind_frame,
    build_regrind_mujoco_model,
    load_regrind_h5,
)


_LOG = logging.getLogger("regrind_mujoco_replay")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "回放 Regrind wuji2 自由根、20 关节和物体 HDF5；"
            "不连接 Motive、Zenoh、IK 或实体机械臂"
        )
    )
    parser.add_argument("h5", type=Path)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="依次应用全部帧后退出，不打开 MuJoCo 窗口",
    )
    return parser


def _log_summary(recording, model) -> None:
    summary = recording.summary()
    _LOG.info(
        "Regrind 轨迹已加载：%s；%d 帧 @ %.3gHz = %.3fs；"
        "root=%s，quat=%s，关节=%d；模型 nq=%d/njnt=%d/nmesh=%d",
        summary["path"],
        summary["frames"],
        summary["fps"],
        summary["duration_s"],
        summary["root_link"],
        summary["quaternion_convention"],
        summary["joint_count"],
        model.nq,
        model.njnt,
        model.nmesh,
    )
    _LOG.info(
        "数据源固定为 regrind_retargeting_*；不会读取穿透率高的 "
        "wuji_retargeting_* 对照结果"
    )


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not args.speed > 0.0:
        raise ValueError("--speed 必须为正数")

    recording = load_regrind_h5(args.h5)
    model, layout = build_regrind_mujoco_model(recording)
    data = mujoco.MjData(model)
    apply_regrind_frame(model, data, recording, layout, 0)
    _log_summary(recording, model)

    if args.validate_only:
        apply_regrind_frame(
            model, data, recording, layout, recording.frame_count - 1
        )
        _LOG.info("Regrind HDF5、同目录手/锤子资产、首末帧均校验通过")
        return 0

    if args.headless:
        for frame_index in range(recording.frame_count):
            apply_regrind_frame(model, data, recording, layout, frame_index)
        _LOG.info("无窗口回放完成：%d 帧", recording.frame_count)
        return 0

    controls = {"paused": bool(args.paused), "restart": False}

    def on_key(keycode: int) -> None:
        if keycode == 32:  # GLFW_KEY_SPACE
            controls["paused"] = not controls["paused"]
            _LOG.info("%s", "暂停" if controls["paused"] else "继续")
        elif keycode == 82:  # GLFW_KEY_R
            controls["restart"] = True
            _LOG.info("从 frame0 重新开始")

    elapsed_s = 0.0
    last_wall = time.monotonic()
    last_frame = -1
    completion_logged = False
    _LOG.info(
        "开始 MuJoCo 回放：Space 暂停/继续，R 从 frame0 重播，关闭窗口退出；"
        "loop=%s，speed=%g",
        args.loop,
        args.speed,
    )
    try:
        with mujoco.viewer.launch_passive(
            model, data, key_callback=on_key
        ) as viewer:
            viewer.cam.lookat[:] = [-0.20, 0.0, 0.12]
            viewer.cam.distance = 1.25
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -35.0

            while viewer.is_running():
                now = time.monotonic()
                wall_step = max(0.0, now - last_wall)
                last_wall = now
                if controls["restart"]:
                    controls["restart"] = False
                    elapsed_s = 0.0
                    completion_logged = False
                if not controls["paused"]:
                    elapsed_s += wall_step * args.speed

                if args.loop:
                    elapsed_s %= recording.duration_s
                else:
                    elapsed_s = min(elapsed_s, recording.duration_s)
                frame_index = min(
                    int(elapsed_s * recording.fps),
                    recording.frame_count - 1,
                )
                if frame_index != last_frame:
                    with viewer.lock():
                        apply_regrind_frame(
                            model, data, recording, layout, frame_index
                        )
                    last_frame = frame_index
                if (
                    not args.loop
                    and elapsed_s >= recording.duration_s
                    and not completion_logged
                ):
                    completion_logged = True
                    controls["paused"] = True
                    _LOG.info(
                        "回放完成并保持末帧；按 R 重播或关闭窗口退出"
                    )
                viewer.sync()
                time.sleep(1.0 / 120.0)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    handler = logging.StreamHandler()
    handler.terminator = "\n\n"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=[handler],
    )
    raise SystemExit(main())
