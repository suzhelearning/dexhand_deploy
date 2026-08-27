#!/usr/bin/env bash
set -euo pipefail

# 单 PC Wuji Hand 2 真机链：本机 H5 主机发布 Manus 键点和 teleop_state，
# 本机 wuji-sdk 经 enp129s0 控制 192.168.1.110/111。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

CONFIRMED=false
READINESS_ONLY=false
HAND_ONLY_REPLAY=false
DEFAULT_RIGHT_SERIAL="WH2KA01260814006"
SELECTED_SIDE="right"
HAS_DEVICE_SELECTOR=false
BRIDGE_ARGS=()

usage() {
  cat <<'EOF'
用法：
  pixi run wuji_hand2_real -- --confirm-real [选项]

选项：
  --confirm-real           确认真机（必须）
  --readiness-only         只检查发布端，不连接/使能右手
  --hand-only-replay       允许无 IK 的 wuji_hand_replay 发布端（仅右手）
  --side right|left        手侧（当前 H5 使用 right）
  --serial SN              显式序列号；覆盖默认右手 WH2KA01260814006
  --address HOST:PORT      直接连接，如 192.168.1.111:50001
  --kp / --kd / --effort-limit
  --rate N                 命令频率（默认 100 Hz）
  --keypoint-timeout S     键点超时回零（默认 0.5s）
  --command-slew-rate R    跟踪/回零最大速度（默认 1rad/s）
  --rotation-x/y/z DEG     mediapipe_rotation
EOF
}

while (($#)); do
  case "$1" in
    --confirm-real)
      CONFIRMED=true
      shift
      ;;
    --readiness-only)
      READINESS_ONLY=true
      shift
      ;;
    --hand-only-replay)
      HAND_ONLY_REPLAY=true
      shift
      ;;
    --side)
      if (($# < 2)); then
        printf '%s\n' '错误：--side 缺少值。' >&2
        exit 2
      fi
      SELECTED_SIDE="$2"
      BRIDGE_ARGS+=("$1" "$2")
      shift 2
      ;;
    --serial|--address)
      if (($# < 2)); then
        printf '错误：%s 缺少值。\n' "$1" >&2
        exit 2
      fi
      HAS_DEVICE_SELECTOR=true
      BRIDGE_ARGS+=("$1" "$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      BRIDGE_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "${CONFIRMED}" != true && "${READINESS_ONLY}" != true ]]; then
  printf '%s\n' '错误：连接真机必须显式传入 --confirm-real。' >&2
  usage >&2
  exit 2
fi
if [[ "${SELECTED_SIDE}" != "left" && "${SELECTED_SIDE}" != "right" ]]; then
  printf '错误：--side 必须为 left 或 right，实际 %s。\n'     "${SELECTED_SIDE}" >&2
  exit 2
fi
if [[ "${HAS_DEVICE_SELECTOR}" != true ]]; then
  if [[ "${SELECTED_SIDE}" == "right" ]]; then
    BRIDGE_ARGS+=(--serial "${DEFAULT_RIGHT_SERIAL}")
  else
    printf '%s\n' '错误：左手尚未固化序列号，必须显式传入 --serial。' >&2
    exit 2
  fi
fi
if [[ "${HAND_ONLY_REPLAY}" == true && "${SELECTED_SIDE}" != "right" ]]; then
  printf '%s\n' '错误：当前 hand-only H5 只发布右手。' >&2
  exit 2
fi

acquire_teleop_guard wuji-hand2-real
install_teleop_cleanup_traps

"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
if [[ "${HAND_ONLY_REPLAY}" == true ]]; then
  node_list="$(read_teleop_node_list)"
  if [[ $'\n'"${node_list}"$'\n' != *$'\n/wuji_hand_replay\n'* ]]; then
    printf '%s\n' \
      '拒绝连接真机：未检测到 tj/live/wuji_hand_replay。' \
      '请先运行 sim_mocap_h5_replay -- TAKE.h5 --hand-commands --paused。' >&2
    exit 1
  fi
else
  assert_single_controller_only_simulation_host_chain
fi
if [[ "${READINESS_ONLY}" == true ]]; then
  printf '%s\n' 'Wuji2 hand-only readiness 通过；未连接、未使能硬件。'
  exit 0
fi

BRIDGE="${PROJECT_PREFIX}/lib/pico_body_tianji/wuji_hand2_bridge"
if [[ ! -x "${BRIDGE}" ]]; then
  BRIDGE="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/wuji_hand2_bridge"
fi
if [[ ! -x "${BRIDGE}" ]]; then
  printf '错误：wuji_hand2_bridge 未部署（先 build/deploy）。\n' >&2
  exit 1
fi

printf '%s\n' \
  '即将控制本机控制网上的 Wuji Hand 2：' \
  '  enp129s0=192.168.1.165/24' \
  '  Left=192.168.1.110 Right=192.168.1.111' \
  "  side=${SELECTED_SIDE} hand_only_replay=${HAND_ONLY_REPLAY} default_right_sn=${DEFAULT_RIGHT_SERIAL}" \
  '  idle/returning/键点超时会按斜坡回零。' \
\
  '  确认手部供电、网线、清场、护罩与急停。'

exec stdbuf -oL -eL "${BRIDGE}" "${BRIDGE_ARGS[@]}"
