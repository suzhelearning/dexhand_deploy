#!/usr/bin/env bash
set -euo pipefail

# H5 右腕绝对轨迹真机链路（Zenoh 通讯版，无 ROS）。
#
# 复用已运行的 H5 仿真主机（sim_mocap_h5：H5 回放节点 + IK），
# 再启动 Marvin 安全桥，把 IK 关节命令低速下发到真机。
#
# 前置：
#   - 主机必须用 --topics-only --speed <= 0.25 启动，且 yaw=0；
#   - Motive tianji_wrist 当前帧必须有效，Enter 必须处于松开状态；
#   - 必须显式 --confirm-real；
#   - 确认双臂 48V、电气急停、控制模式和运动空间均已检查，并
#     关闭 FxStation、官方天机控制节点及其他 Marvin SDK 会话。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

CONFIRMED=false
ROBOT_IP=""
# H5 首次真机验收默认采用桥的基准低速，而不是其他遥操作入口的
# 65/85 高动态参数。10% 在当前配置下将关节斜坡限制为约 11.1°/s。
VELOCITY_RATIO=10
ACCELERATION_RATIO=10

usage() {
  cat <<'EOF'
用法：
  pixi run real_mocap_h5 -- --confirm-real [选项]

危险操作确认：
  --confirm-real             明确允许 H5 轨迹连接并驱动实体 Marvin 双臂

选项：
  --robot-ip IP              覆盖真机 IP（默认取 controller_only_real.yaml）
  --velocity-ratio 1..100    速度比例，默认 10
  --acceleration-ratio 1..100
                             加速度比例，默认 10
  -h, --help

前置主机示例：
  pixi run sim_mocap_h5 -- TAKE.h5 --topics-only --speed 0.1 \
      --right-rigid-id ID

真机桥只接受 speed <= 0.25、yaw=0、phase=armed、IK/Home 安全、
Motive tianji_wrist marker 有效、Enter 松开的绝对 wrist 回放主机。
EOF
}

require_value() {
  if (($# < 2)); then
    printf '错误：%s 缺少参数值\n' "$1" >&2
    exit 2
  fi
}

validate_ratio() {
  local label="$1"
  local value="$2"
  if [[ ! "${value}" =~ ^[0-9]+$ ]] ||
     ((value < 1 || value > 100)); then
    printf '错误：%s 必须是 1..100 的整数\n' "${label}" >&2
    exit 2
  fi
}

while (($#)); do
  case "$1" in
    --confirm-real)
      CONFIRMED=true
      ;;
    --robot-ip)
      require_value "$@"
      ROBOT_IP="$2"
      shift
      ;;
    --velocity-ratio)
      require_value "$@"
      VELOCITY_RATIO="$2"
      shift
      ;;
    --acceleration-ratio)
      require_value "$@"
      ACCELERATION_RATIO="$2"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '错误：未知参数 %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "${CONFIRMED}" != true ]]; then
  printf '%s\n' \
    '拒绝启动：必须显式提供 --confirm-real。' >&2
  exit 2
fi
validate_ratio "--velocity-ratio" "${VELOCITY_RATIO}"
validate_ratio "--acceleration-ratio" "${ACCELERATION_RATIO}"

acquire_teleop_guard real
install_teleop_cleanup_traps

"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes real
assert_single_controller_only_simulation_host_chain

printf '%s\n' \
  '即将启动 H5 Manus wrist 绝对轨迹真机链路：' \
  '  复用 sim_mocap_h5 的 marker→wrist 接近 + IK 主机链路' \
  '  安全桥 → Marvin 低层关节控制' \
  "  速度比例=${VELOCITY_RATIO}% 加速度比例=${ACCELERATION_RATIO}%" \
  '请确认双臂 48V、电气急停、控制模式、轨迹空间及 tianji_wrist ID。' \
  '关闭 FxStation、官方天机控制节点及其他 Marvin SDK 会话。' \
  '等待提示“真机链路已就绪”（phase=armed_idle）；此前不得按 s 或 Enter。' \
  '就绪后按 s 读取 marker；Enter 保压到 frame0，松开后按 r，再保压回放。'

BRIDGE_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/marvin_hardware_bridge"
REAL_CONFIG="${PROJECT_PREFIX}/share/pico_body_tianji/config/mode/controller_only/controller_only_real.yaml"

bridge_arguments=(
  "--confirm-real"
  "--config"
  "${REAL_CONFIG}"
  "--param"
  "velocity_ratio:=${VELOCITY_RATIO}"
  "--param"
  "acceleration_ratio:=${ACCELERATION_RATIO}"
)
if [[ -n "${ROBOT_IP}" ]]; then
  bridge_arguments+=("--param" "robot_ip:=${ROBOT_IP}")
fi
setsid python "${BRIDGE_NODE}" "${bridge_arguments[@]}" &
bridge_pid=$!
register_teleop_process_group \
  "${bridge_pid}" marvin-mocap-h5-hardware-bridge 30

set +e
wait "${bridge_pid}"
bridge_exit=$?
set -e
printf '%s%s%s\n' \
  'Marvin H5 真机桥已退出，状态码：' \
  "${bridge_exit}" \
  '；H5 主机链路继续运行。' \
  >&2
exit "${bridge_exit}"
