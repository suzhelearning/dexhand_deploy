"""H5 v4 wrist diagnostic overlay preparation。

该工具只读外部 acquisition v4 文件；可选 MuJoCo passive viewer 用于现场目检
robot home、frame0 skeleton 数据和 wrist/TCP 标定摘要。它不会声明 Zenoh
publisher，不发布 SessionState、JointState 或 final command。
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from ..sources.mocap.h5 import load_mocap_h5


def _run_viewer(recording) -> None:
    import mujoco
    import mujoco.viewer
    from ..mujoco_urdf import portable_mujoco_urdf

    root = Path(__file__).resolve().parents[4]
    urdf = root / "src" / "pico_body_tianji" / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
    xml, assets = portable_mujoco_urdf(urdf)
    model = mujoco.MjModel.from_xml_string(xml, assets)
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Keep the model at configured Home while exposing frame0 diagnostics in
        # the terminal. Viewer remains passive: no command/state authority.
        while viewer.is_running():
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1.0 / max(float(recording.output_hz), 1.0))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="read-only H5 wrist MuJoCo diagnostic")
    parser.add_argument("h5", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--viewer", action="store_true", help="open passive MuJoCo overlay")
    args = parser.parse_args(argv)
    recording = load_mocap_h5(args.h5)
    summary = recording.summary()
    summary.update({"overlay": "frame0_hand_skeleton", "executor_authority": "MujocoExecutor", "viewer": bool(args.viewer)})
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.viewer:
        _run_viewer(recording)
    return 0


__all__ = ["main"]

if __name__ == "__main__":
    raise SystemExit(main())
