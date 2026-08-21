"""zenoh 迁移端到端冒烟：Python 驱动 C++ IK 节点。"""
import json
import subprocess
import sys
import threading
import time

import zenoh

BIN = "staging/ik/lib/pico_body_tianji/tianji_kinematic_sim"
URDF = "staging/ik/share/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"

results = {}
done = threading.Event()
lock = threading.Lock()


def on_json(key):
    def handler(sample):
        data = json.loads(bytes(sample.payload))
        with lock:
            results.setdefault(key, []).append(data)
            if key == "model_joint_states":
                done.set()
    return handler


def main():
    proc = subprocess.Popen(
        [sys.executable and BIN, f"urdf_path:={URDF}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    session = zenoh.open(zenoh.Config())
    try:
        # 订阅输出
        for key in ("model_joint_states", "status",
                    "left_arm/joint_commands", "right_arm/joint_commands",
                    "left_arm/solved_pose"):
            k = f"pico_body_sim/{key}"
            session.declare_subscriber(k, on_json(k))

        # 1) at_home 初始值（queryable）
        got = {}
        latch_done = threading.Event()
        def on_latch(reply):
            if reply.ok:
                got["at_home"] = bytes(reply.result.payload)
            latch_done.set()
        session.get("pico_body_sim/at_home", on_latch, timeout=1.0)
        latch_done.wait(2)
        print("at_home initial:", got.get("at_home"))

        # 2) 等待 model_joint_states（30Hz）
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            with lock:
                if results.get("pico_body_sim/model_joint_states"):
                    break
            time.sleep(0.1)
        with lock:
            models = results.get("pico_body_sim/model_joint_states", [])
        if not models:
            print("FAIL: 未收到 model_joint_states")
            print("received keys:", {k: len(v) for k, v in results.items()})
            return 1
        print("model_joint_states:", json.dumps(models[0])[:180])

        # 3) 进入 teleop 并发左臂目标
        session.put("pico_body/teleop_state", b"teleop")
        target = {
            "stamp": {"sec": 0, "nanosec": 0},
            "frame_id": "left_chest",
            "position": {"x": 0.35, "y": 0.25, "z": 0.45},
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        }
        elbow = {
            "stamp": {"sec": 0, "nanosec": 0},
            "frame_id": "left_chest",
            "vector": {"x": 0.45638698, "y": -0.74604902, "z": -0.48489358},
        }
        end = time.monotonic() + 2.5
        while time.monotonic() < end:
            session.put(
                "pico_body/left_arm_target_pose",
                json.dumps(target).encode(),
            )
            session.put(
                "pico_body/left_arm_elbow_direction",
                json.dumps(elbow).encode(),
            )
            time.sleep(0.03)
        time.sleep(0.5)

        with lock:
            left_cmds = results.get("pico_body_sim/left_arm/joint_commands", [])
            solved = results.get("pico_body_sim/left_arm/solved_pose", [])
            statuses = results.get("pico_body_sim/status", [])
        print("left joint_commands count:", len(left_cmds))
        if left_cmds:
            print("last left cmd:", json.dumps(left_cmds[-1])[:200])
        print("left solved_pose count:", len(solved))
        print("status count:", len(statuses))
        if not left_cmds or not solved or len(statuses) < 3:
            print("FAIL: 输出不足")
            return 1

        # 4) returning → at_home true + return_complete
        session.put("pico_body/teleop_state", b"returning")
        time.sleep(4.0)
        got2 = {}
        def on_latch2(reply):
            if reply.ok:
                got2["value"] = bytes(reply.result.payload)
            latch_done.set()
        latch_done.clear()
        session.get("pico_body_sim/at_home", on_latch2, timeout=1.0)
        latch_done.wait(2)
        print("at_home after return:", got2.get("value"))
        if got2.get("value") != b"true":
            print("FAIL: 回位后 at_home 未变 true")
            return 1

        print("E2E-OK")
        return 0
    finally:
        session.close()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        out = proc.stdout.read() if proc.stdout else ""
        print("--- node log tail ---")
        print("\n".join(out.splitlines()[-8:]))


if __name__ == "__main__":
    sys.exit(main())
