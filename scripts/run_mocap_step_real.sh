#!/usr/bin/env bash
set -euo pipefail

# mocap 键盘步进真机链路（Zenoh 通讯版，无 ROS）。
#
# 复用已运行的键盘步进仿真主机（pixi run sim_mocap_step：键盘步进
# 节点 + IK），再启动 Marvin 安全桥，把 IK 解算的关节命令下发到
# 真机。按键 → 动捕系目标增量 → IK → 真机（默认仅右臂，10mm/键）。
#
# 前置：
#   - 先运行 pixi run sim_mocap_step（键盘步进 + IK 主机链路）；
#   - 必须显式 --confirm-real；
#   - 请确认双臂 48V、电气急停、控制模式和运动空间均已检查，并
#     关闭 FxStation、官方天机控制节点及其他 Marvin SDK 会话。
#
# 用法：
#   pixi run real_mocap_step -- --confirm-real
#   pixi run real_mocap_step -- --confirm-real --robot-ip 192.168.1.190

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

CONFIRMED=false
ROBOT_IP=""
VELOCITY_RATIO=65
ACCELERATION_RATIO=85

usage() {
  cat <<'EOF'
用法：
  pixi run real_mocap_step -- [--confirm-real] [--robot-ip IP]
      [--velocity-ratio N] [--acceleration-ratio N]

选项：
  --confirm-real          显式确认连接真机（必须提供）
  --robot-ip IP           覆盖真机 IP（默认取 controller_only_real.yaml）
  --velocity-ratio N      速度比例 1..100（默认 65）
  --acceleration-ratio N  加速度比例 1..100（默认 85）
  -h, --help
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
  '即将启动 mocap 键盘步进真机链路：' \
  '  复用 pixi run sim_mocap_step 的键盘步进 + IK 链路' \
  '  安全桥 → Marvin 低层关节控制' \
  '请确认双臂 48V、电气急停、控制模式和运动空间均已检查，' \
  '并关闭 FxStation、官方天机控制节点及其他 Marvin SDK 会话。' \
  '机械臂进入 armed_idle 前不要按 s / 方向键。'

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
  "${bridge_pid}" marvin-mocap-step-hardware-bridge 30

set +e
wait "${bridge_pid}"
bridge_exit=$?
set -e
printf '%s%s%s\n' \
  'Marvin 键盘步进真机桥已退出，状态码：' \
  "${bridge_exit}" \
  '；仿真主机链路继续运行。' \
  >&2
exit "${bridge_exit}"
