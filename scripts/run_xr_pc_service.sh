#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

service_root_override=""
check_only=false

usage() {
  printf '%s\n' \
    '用法: run_xr_pc_service.sh [选项]' \
    '' \
    '在前台启动 XRoboToolkit PC-Service，并复现旧 runService.sh 的运行时环境。' \
    '默认使用 vendor/xr_pc_service/roboticsservice；不会由 run_session 自动启动。' \
    '' \
    '选项:' \
    '  --root PATH  指定 PC-Service 根目录；优先级高于 XR_PC_SERVICE_ROOT' \
    '  --check      只校验文件和动态库入口，不启动服务' \
    '  --help       显示帮助'
}

fail_config() {
  printf '错误：PC-Service %s\n' "$1" >&2
  exit 2
}

while (($#)); do
  case "$1" in
    --root)
      [[ $# -ge 2 ]] || fail_config '--root 缺少路径'
      service_root_override="$2"
      shift 2
      ;;
    --check)
      check_only=true
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      fail_config "未知参数: $1"
      ;;
  esac
done

service_root="${service_root_override:-${XR_PC_SERVICE_ROOT:-${BUNDLE_ROOT}/vendor/xr_pc_service/roboticsservice}}"
if ! service_root="$(realpath -e -- "${service_root}")"; then
  fail_config "根目录不存在: ${service_root}"
fi

service_binary="${service_root}/RoboticsServiceProcess"
sdk_library="${service_root}/SDK/x64/libPXREARobotSDK.so"
[[ -x "${service_binary}" ]] || fail_config "缺少可执行文件: ${service_binary}"
[[ -f "${sdk_library}" ]] || fail_config "缺少 SDK companion library: ${sdk_library}"

runtime_library_path="${service_root}:${service_root}/lib:${service_root}/SDK/x64"
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  runtime_library_path="${LD_LIBRARY_PATH}:${runtime_library_path}"
fi

if [[ "${check_only}" == true ]]; then
  printf 'PC-Service root: %s\n' "${service_root}"
  printf 'PC-Service binary: %s\n' "${service_binary}"
  printf 'PC-Service SDK library: %s\n' "${sdk_library}"
  printf 'PC-Service LD_LIBRARY_PATH: %s\n' "${runtime_library_path}"
  if [[ -x "${service_root}/runService.sh" ]]; then
    printf '%s\n' 'PC-Service legacy launcher: present'
  else
    printf '%s\n' 'PC-Service legacy launcher: absent (foreground binary is usable)'
  fi
  exit 0
fi

export LD_LIBRARY_PATH="${runtime_library_path}"
export QT_PLUGIN_PATH="${service_root}/plugins${QT_PLUGIN_PATH:+:${QT_PLUGIN_PATH}}"
export QT_QML_PATH="${service_root}/qml${QT_QML_PATH:+:${QT_QML_PATH}}"
cd -- "${service_root}"
printf '启动 XRoboToolkit PC-Service（前台）: %s\n' "${service_binary}" >&2
exec "${service_binary}"
