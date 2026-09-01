#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

if [[ "$#" != 1 || "$1" != --confirm-real ]]; then
  printf '%s\n' '用法：pixi run bash scripts/return_home.sh --confirm-real' >&2
  exit 2
fi

activate_bundle_runtime
acquire_teleop_guard marvin_return_home
install_teleop_cleanup_traps

if ! existing_tokens="$(read_teleop_node_list)"; then
  printf '%s\n' '错误：无法确认当前没有运行中的 teleop 组件，拒绝连接真机。' >&2
  exit 1
fi
if [[ -n "${existing_tokens}" ]]; then
  printf '%s\n' '错误：仍有 teleop 组件运行；请先在原终端 Ctrl+C，等待清理完成。' >&2
  printf '%s\n' "${existing_tokens}" >&2
  exit 1
fi

device_config="$(canonical_config robot/devices.yaml)"
marvin_config="$(canonical_config executors/marvin.yaml)"

python - "${device_config}" "${marvin_config}" <<'PY'
from __future__ import annotations

import sys

import numpy as np
import yaml

from tianji_teleop.coordination.arm_command_coordinator import ArmRobotConfig
from tianji_teleop.executors.marvin.sdk_session import create_official_marvin_session


devices = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
settings = yaml.safe_load(open(sys.argv[2], encoding="utf-8"))
robot = ArmRobotConfig.load()
lower_deg = np.degrees(robot.lower_limits_rad)
upper_deg = np.degrees(robot.upper_limits_rad)
home_deg = np.degrees(robot.home_all)
hardware = create_official_marvin_session()

try:
    feedback = hardware.connect_and_prepare(
        devices["marvin"]["ip"],
        velocity_ratio=int(settings["velocity_ratio"]),
        acceleration_ratio=int(settings["acceleration_ratio"]),
        lower_limits_deg=lower_deg,
        upper_limits_deg=upper_deg,
        hard_limit_padding_deg=float(settings["feedback_hard_limit_padding_deg"]),
    )
    current_deg = np.concatenate((feedback.left_joints_deg, feedback.right_joints_deg))
    print(
        "Marvin 已连接；当前距 Home 最大关节误差 "
        f"{np.max(np.abs(current_deg - home_deg)):.2f} deg，开始平滑回零。",
        flush=True,
    )
    feedback = hardware.move_to_home(
        home_deg[:7],
        home_deg[7:],
        rate_hz=float(settings["rate_hz"]),
        minimum_duration_s=float(settings["return_minimum_duration_s"]),
        max_speed_deg_s=float(settings["return_max_speed_deg_s"]),
        maximum_tracking_error_deg=float(settings["maximum_tracking_error_deg"]),
        home_tolerance_deg=float(settings["home_tolerance_deg"]),
        lower_limits_deg=lower_deg,
        upper_limits_deg=upper_deg,
        hard_limit_padding_deg=float(settings["feedback_hard_limit_padding_deg"]),
        feedback_timeout_s=float(settings["feedback_timeout_s"]),
    )
    final_deg = np.concatenate((feedback.left_joints_deg, feedback.right_joints_deg))
    print(
        "完成：机械臂已平滑回到 Home；最大关节误差 "
        f"{np.max(np.abs(final_deg - home_deg)):.2f} deg。",
        flush=True,
    )
finally:
    hardware.shutdown()
PY
