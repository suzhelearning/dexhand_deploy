#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
activate_bundle_runtime

DIAGNOSTIC="${PROJECT_PREFIX}/lib/pico_body_tianji/controller_only_real_diagnostic"
if [[ ! -x "${DIAGNOSTIC}" ]]; then
  printf '错误：真机只读诊断器尚未部署：%s\n' "${DIAGNOSTIC}" >&2
  printf '%s\n' '请先执行 pixi run -e ik-build build-ik 和 deploy-ik。' >&2
  exit 1
fi
exec "${DIAGNOSTIC}" "$@"
