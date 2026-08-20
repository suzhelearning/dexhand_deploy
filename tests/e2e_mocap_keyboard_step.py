#!/usr/bin/env python3
"""mocap 键盘步进 Zenoh 端到端：按键注入 → 目标发布 → IK 解算。

验证（side=right 默认）：
- 按键流 s → up → up → 1 → up → 0 → s，每次 +10mm（动捕系）；
- 节点发布 /pico_body/right_arm_target_pose（settle 收敛）；
- IK 右臂 joint_commands 偏离 Home（solved 末端跟随目标）；
- 左臂 joint_commands 全程保持 Home（无目标臂不被解算）；
- teleop 结束后 IK at_home 恢复 true，节点退出码 0。

用法（在 bundle 根目录，先 activate_bundle_runtime）：
    python tests/e2e_mocap_keyboard_step.py
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time

import zenoh

BUNDLE_ROOT = os.environ.get(
    "PICO_BODY_TIANJI_BUNDLE_ROOT", os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
PROJECT_PREFIX = os.path.join(BUNDLE_ROOT, "runtime", "pico_body_tianji")

RIGHT_HOME = [-55.0, -65.0, 70.0, -60.0, -60.0, 0.0, 0.0]
LEFT_HOME = [55.0, -65.0, -70.0, -60.0, 60.0, 0.0, 0.0]
KEY_STREAM = "s\x1b[A\x1b[A1\x1b[A0s"  # s → up → up → 1 → up → 0 → s
KEY_INTERVAL_S = 0.4

buckets: dict[str, list] = {}
lock = threading.Lock()
at_home_samples: list[tuple[float, bytes]] = []


def collect(key: str):
    def handler(sample: zenoh.Sample) -> None:
        with lock:
            bucket = buckets.setdefault(key, [])
            if len(bucket) < 600:
                bucket.append(json.loads(bytes(sample.payload)))
    return handler


def on_latch(reply) -> None:
    if reply.ok:
        at_home_samples.append((time.monotonic(), bytes(reply.result.payload)))


def main() -> int:
    ik_bin = os.path.join(
        BUNDLE_ROOT, "staging", "ik", "lib", "pico_body_tianji",
        "tianji_kinematic_sim",
    )
    if not os.access(ik_bin, os.X_OK):
        ik_bin = os.path.join(
            PROJECT_PREFIX, "lib", "pico_body_tianji",
            "tianji_kinematic_sim",
        )
    urdf = os.path.join(
        PROJECT_PREFIX, "share", "pico_body_tianji", "assets",
        "marvin_m6_ccs", "urdf", "marvin_m6_s_ccs_696_v4.urdf",
    )
    node_module = (
        "pico_body_tianji.controller_only.mocap_keyboard_step_node"
    )

    ik = subprocess.Popen(
        [ik_bin, f"urdf_path:={urdf}", "ik_backend:=pinocchio_cpp"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    session = zenoh.open(zenoh.Config())
    try:
        time.sleep(1.5)
        for key in (
            "pico_body_sim/left_arm/joint_commands",
            "pico_body_sim/right_arm/joint_commands",
            "pico_body_sim/right_arm/solved_pose",
            "pico_body/right_arm_target_pose",
        ):
            session.declare_subscriber(key, collect(key))

        # 间隔注入按键流（瞬时写入会让结束 's' 与开始 's' 同帧）。
        keys = subprocess.Popen(
            [
                sys.executable, "-m", node_module,
                "--param", "step_mm:=10",
                "--param", "side:=right",
                "--param", "rate:=60",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        assert keys.stdin is not None
        for ch in KEY_STREAM:
            keys.stdin.write(ch)
            keys.stdin.flush()
            time.sleep(KEY_INTERVAL_S)
        keys.stdin.close()
        exit_code = keys.wait(timeout=15)
        stderr = keys.stderr.read() if keys.stderr else ""

        end = time.monotonic() + 8
        while time.monotonic() < end:
            session.get("pico_body_sim/at_home", on_latch, timeout=1.0)
            time.sleep(1.0)
    finally:
        session.close()
        ik.terminate()

    with lock:
        left = [r["position"] for r in buckets.get(
            "pico_body_sim/left_arm/joint_commands", [])]
        right = [r["position"] for r in buckets.get(
            "pico_body_sim/right_arm/joint_commands", [])]
        targets = buckets.get("pico_body/right_arm_target_pose", [])
        solved = buckets.get("pico_body_sim/right_arm/solved_pose", [])

    failures = []
    if exit_code != 0:
        failures.append(f"节点退出码 {exit_code}，stderr：{stderr[-500:]}")
    if not left:
        failures.append("未收到左臂 joint_commands")
    elif any(p != left[0] for p in left):
        failures.append("左臂 joint_commands 发生了变化（应为恒定 Home）")
    elif left[0] != LEFT_HOME:
        failures.append(f"左臂未保持 Home：{left[0]}")
    if not right:
        failures.append("未收到右臂 joint_commands")
    elif not any(p != RIGHT_HOME for p in right):
        failures.append("右臂 joint_commands 全程 Home（按键未生效）")
    if not targets:
        failures.append("未收到节点发布的右臂目标")
    if not solved:
        failures.append("未收到 IK solved_pose")
    if not any(v == b"true" for _, v in at_home_samples[-2:]):
        failures.append("步进结束后 at_home 未恢复 true")

    if targets and solved:
        t0 = targets[0]["position"]
        t1 = targets[-1]["position"]
        dx = round(t1["x"] - t0["x"], 3)
        s0 = solved[0]["position"]
        s1 = solved[-1]["position"]
        sx = round(s1["x"] - s0["x"], 3)
        print(f"目标位移 x：{dx} m（动捕 +z 30mm → 机器人 +x，mocap_to_robot）")
        print(f"solved 位移 x：{sx} m")
        # 动捕 +z 30mm → chest 系 +x 方向（同向映射）；容忍收敛误差
        if not (0.027 <= dx <= 0.033):
            failures.append(f"目标 x 位移异常：{dx}")

    print(f"左臂帧数：{len(left)}，右臂帧数：{len(right)}，"
          f"目标帧数：{len(targets)}，solved 帧数：{len(solved)}")
    print(f"at_home 采样：{[v.decode() for _, v in at_home_samples]}")

    if failures:
        for failure in failures:
            print("FAIL:", failure)
        return 1
    print("PASS：键盘步进 Zenoh 链路验证通过（右臂动、左臂 Home、回 Home 恢复）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
