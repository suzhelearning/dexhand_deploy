#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

if (($# < 1)) || [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  printf '%s\n' \
    '用法：pixi run mocap-step-h5 -- --output OUT.h5 [--axis x|y|z]' \
    '      [--dir pos|neg] [--mm 50] [--ramp-s 1.0] [--hold-s 1.5]' \
    '      [--return-s 1.0] [--rate 60]' \
    '生成 mocap v4.0 合成台阶轨迹 HDF5，用于轨迹跟踪 1:1 验收。' >&2
  exit 0
fi

activate_bundle_runtime
exec "${PROJECT_PREFIX}/lib/pico_body_tianji/mocap_step_h5" "$@"
