#!/usr/bin/env bash
set -euo pipefail

# PICO (XRoboToolkit-PICO) 通过 adb reverse 访问本机 XRoboToolkit PC Service。
#
# 原理: PICO 应用内把服务器地址设为 127.0.0.1（video_source.yml 的 ADB 模式），
#       adb reverse 把设备侧 tcp:PORT 的连接到本机同端口服务。
# 本机 RoboticsService 监听 127.0.0.1:60061（控制/数据）与 *:63901（视频流/辅助），
# 只绑定回环的 60061 无线不可达，必须走 adb reverse。
#
# 用法:
#   bash scripts/adb_reverse.sh          # 建立并验证 reverse 规则（默认）
#   bash scripts/adb_reverse.sh list     # 查看当前规则
#   bash scripts/adb_reverse.sh down     # 移除所有规则
#
# 环境变量:
#   ADB                          adb 路径（默认自动探测 PATH 或 ~/.local/opt/platform-tools）
#   PICO_XRTOOLKIT_DATA_PORT     控制/数据端口，默认 60061
#   PICO_XRTOOLKIT_STREAM_PORT   视频流/辅助端口，默认 63901；置空跳过

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

DATA_PORT="${PICO_XRTOOLKIT_DATA_PORT:-60061}"
STREAM_PORT="${PICO_XRTOOLKIT_STREAM_PORT:-63901}"

ADB="${ADB:-}"
if [[ -z "${ADB}" ]]; then
  if command -v adb >/dev/null 2>&1; then
    ADB="$(command -v adb)"
  elif [[ -x "${HOME}/.local/opt/platform-tools/adb" ]]; then
    ADB="${HOME}/.local/opt/platform-tools/adb"
  else
    echo "未找到 adb：请安装 Android platform-tools，或设置 ADB=/path/to/adb" >&2
    exit 1
  fi
fi

find_device() {
  local line serial
  while read -r line; do
    [[ "${line}" == *$'\tdevice'* ]] || continue
    serial="${line%%$'\t'*}"
    if [[ -n "${serial}" ]]; then
      echo "${serial}"
      return 0
    fi
  done < <("${ADB}" devices | tail -n +2)
  return 1
}

cmd_list() {
  echo "当前 reverse 规则:"
  "${ADB}" reverse --list || true
}

cmd_down() {
  "${ADB}" reverse --remove-all
  echo "已移除全部 reverse 规则"
}

cmd_up() {
  local serial port
  serial="$(find_device)" || {
    echo "没有处于 device 状态的 adb 设备。请检查 USB 连接，并在头显中授权 USB 调试。" >&2
    exit 1
  }
  echo "设备: ${serial}"

  for port in "${DATA_PORT}" "${STREAM_PORT}"; do
    [[ -n "${port}" ]] || continue
    "${ADB}" reverse "tcp:${port}" "tcp:${port}"
    echo "  reverse tcp:${port} (PICO) → 本机 tcp:${port}"
  done

  echo "---"
  cmd_list

  # 连通性验证: 从设备侧连接 127.0.0.1:端口，能建立 TCP 即 reverse 生效。
  if "${ADB}" shell 'command -v nc >/dev/null 2>&1'; then
    for port in "${DATA_PORT}" "${STREAM_PORT}"; do
      [[ -n "${port}" ]] || continue
      if "${ADB}" shell "timeout 2 nc -w 1 127.0.0.1 ${port} </dev/null >/dev/null 2>&1"; then
        echo "  设备 → 127.0.0.1:${port} 连通 OK"
      else
        echo "  设备 → 127.0.0.1:${port} 连接失败" >&2
        exit 1
      fi
    done
  else
    echo "（设备无 nc，跳过连通性验证；以 reverse --list 结果为准）"
  fi

  echo "完成。请在 PICO 的 XRoboToolkit 中选择 ADB 模式（服务器 127.0.0.1）。"
  echo "注意: 拔掉 USB 或重启设备后规则失效，需要重新执行本脚本。"
}

case "${1:-up}" in
  up)   cmd_up ;;
  list) cmd_list ;;
  down) cmd_down ;;
  *)
    echo "未知参数: ${1}" >&2
    echo "用法: $0 [up|list|down]" >&2
    exit 1
    ;;
esac
