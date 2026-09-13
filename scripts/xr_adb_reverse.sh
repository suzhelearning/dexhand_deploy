#!/usr/bin/env bash
set -euo pipefail

data_port="${TIANJI_XR_DATA_PORT:-60061}"
stream_port="${TIANJI_XR_STREAM_PORT:-63901}"
command_name="${1:-up}"

usage() {
  printf '%s\n' \
    '用法: xr_adb_reverse.sh [up|list|down]' \
    '' \
    '为 PICO 头显 + VR 手柄的 XRoboToolkit PC-Service 设置 ADB reverse。' \
    '默认映射数据端口 60061 和串流端口 63901；不会操作 PICO2 的 tcp:10002。' \
    '可用 TIANJI_XR_DATA_PORT、TIANJI_XR_STREAM_PORT 或 ADB_SERIAL 覆盖。'
}

case "${command_name}" in
  --help|-h)
    usage
    exit 0
    ;;
  up|list|down)
    ;;
  *)
    printf '错误：未知命令: %s\n' "${command_name}" >&2
    usage >&2
    exit 2
    ;;
esac

valid_port() {
  [[ "$1" =~ ^[0-9]+$ ]] && ((10#$1 >= 1 && 10#$1 <= 65535))
}
valid_port "${data_port}" || { printf '错误：非法 XR data port: %s\n' "${data_port}" >&2; exit 2; }
valid_port "${stream_port}" || { printf '错误：非法 XR stream port: %s\n' "${stream_port}" >&2; exit 2; }

if [[ -n "${ADB:-}" ]]; then
  adb_bin="${ADB}"
else
  adb_bin="$(command -v adb || true)"
fi
[[ -n "${adb_bin}" ]] || {
  printf '%s\n' '错误：找不到 adb；请安装 Android platform-tools 或设置 ADB。' >&2
  exit 1
}

serial="${ADB_SERIAL:-}"
if [[ -z "${serial}" ]]; then
  serial="$(
    "${adb_bin}" devices |
      awk '$2 == "device" { print $1; exit }'
  )"
fi
[[ -n "${serial}" ]] || {
  printf '%s\n' '错误：没有状态为 device 的 PICO ADB 设备。请先检查 adb devices。' >&2
  exit 1
}

# Always bind reverse operations to the selected device.  Without this, an
# automatically discovered serial was only used for the discovery check and
# adb could reject the reverse command (or apply it to an unintended device)
# when more than one headset was connected.
adb_prefix=("${adb_bin}" -s "${serial}")

if [[ "${command_name}" == list ]]; then
  "${adb_prefix[@]}" reverse --list
  exit 0
fi

if [[ "${command_name}" == up ]]; then
  "${adb_prefix[@]}" reverse "tcp:${data_port}" "tcp:${data_port}"
  "${adb_prefix[@]}" reverse "tcp:${stream_port}" "tcp:${stream_port}"
  printf 'XR ADB reverse 已设置: tcp:%s -> tcp:%s\n' "${data_port}" "${data_port}"
  printf 'XR ADB reverse 已设置: tcp:%s -> tcp:%s\n' "${stream_port}" "${stream_port}"
else
  "${adb_prefix[@]}" reverse --remove "tcp:${data_port}"
  "${adb_prefix[@]}" reverse --remove "tcp:${stream_port}"
  printf 'XR ADB reverse 已移除: tcp:%s, tcp:%s\n' "${data_port}" "${stream_port}"
fi
