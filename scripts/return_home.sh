#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

confirm_real=false
side=both
recover_outside_limits=false
while (($#)); do
  case "$1" in
    --confirm-real)
      confirm_real=true
      shift
      ;;
    --side)
      [[ "$#" -ge 2 ]] || break
      side="$2"
      shift 2
      ;;
    --recover-outside-limits)
      recover_outside_limits=true
      shift
      ;;
    *)
      break
      ;;
  esac
done
if [[ "${confirm_real}" != true || "$#" != 0 || "${side}" != @(left|right|both) ]]; then
  printf '%s\n' '用法：pixi run bash scripts/return_home.sh --confirm-real [--side left|right|both] [--recover-outside-limits]' >&2
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

recovery_args=()
if [[ "${recover_outside_limits}" == true ]]; then
  recovery_args+=(--recover-outside-limits)
fi
python -m tianji_teleop.executors.marvin.return_home \
  --device-config "${device_config}" \
  --marvin-config "${marvin_config}" \
  --side "${side}" \
  "${recovery_args[@]}"
