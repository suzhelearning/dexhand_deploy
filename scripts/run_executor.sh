#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

executor_id=""
mode=""
side="right"
config_override=""
display_mode="config"
confirm_real=false
real_capability_provider=""
required_capability="${TIANJI_REQUIRED_CAPABILITY:-simulation}"
while (($#)); do
  case "$1" in
    --executor|--profile) executor_id="${2:-}"; shift 2 ;;
    --mode) mode="${2:-}"; shift 2 ;;
    --side) side="${2:-}"; shift 2 ;;
    --config) config_override="${2:-}"; shift 2 ;;
    --viewer)
      [[ "${display_mode}" != headless ]] || {
        printf '%s\n' '错误：--viewer 与 --headless 互斥。' >&2
        exit 2
      }
      display_mode=viewer
      shift
      ;;
    --headless)
      [[ "${display_mode}" != viewer ]] || {
        printf '%s\n' '错误：--viewer 与 --headless 互斥。' >&2
        exit 2
      }
      display_mode=headless
      shift
      ;;
    --confirm-real) confirm_real=true; shift ;;
    --real-capability-provider) real_capability_provider="${2:-}"; shift 2 ;;
    --help|-h)
      printf '%s\n' \
        '用法: run_executor.sh --executor {mujoco|marvin|wuji_hand2} [--side left|right] [--viewer|--headless] [--confirm-real]' \
        'MuJoCo: --viewer 覆盖 config 的 headless: true；--headless 显式启用无窗口模式。'
      exit 0 ;;
    --) shift; break ;;
    *) break ;;
  esac
done
if [[ -z "${executor_id}" ]]; then
  printf '%s\n' '错误：必须指定 --executor。' >&2
  exit 2
fi
case "${executor_id}" in
  mujoco)
    entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/mujoco_executor"
    default_config="executors/mujoco.yaml"
    ;;
  marvin)
    entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/marvin_executor"
    default_config="executors/marvin.yaml"
    [[ "${confirm_real}" == true ]] || { printf '%s\n' 'Marvin executor requires --confirm-real' >&2; exit 2; }
    ;;
  wuji_hand2)
    entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/wuji_hand2_executor"
    default_config="executors/wuji_hand2.yaml"
    ;;
  *) printf '错误：未知 executor: %s\n' "${executor_id}" >&2; exit 2 ;;
esac
if [[ "${executor_id}" != mujoco && "${display_mode}" != config ]]; then
  printf '%s\n' '错误：--viewer/--headless 仅适用于 MuJoCo executor。' >&2
  exit 2
fi
if [[ ! -x "${entry}" ]]; then
  printf '错误：executor entry 不存在或不可执行: %s\n' "${entry}" >&2
  exit 1
fi
config="${config_override}"
if [[ -z "${config}" ]]; then config="$(canonical_config "${default_config}")"; fi
headless=false
if [[ "${executor_id}" == mujoco && "${display_mode}" == headless ]]; then
  headless=true
elif [[ "${executor_id}" == mujoco && "${display_mode}" == config ]]; then
  # Headless is a component-configured execution mode.  The wrapper must
  # forward it explicitly so no-DISPLAY launches never fall through to the
  # passive viewer by accident.
  if pixi run python - "${config}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    value = yaml.safe_load(stream) or {}
if not isinstance(value, dict):
    raise SystemExit(1)
raise SystemExit(0 if value.get("headless") is True else 1)
PY
  then
    headless=true
  fi
fi
export TIANJI_ROUTER_ENDPOINT="${TIANJI_ROUTER_ENDPOINT:-tcp/127.0.0.1:7447}"
export TIANJI_ROUTER_ZID="${TIANJI_ROUTER_ZID:-$(require_router)}"
export TIANJI_COMPONENT_INSTANCE_ID="${TIANJI_COMPONENT_INSTANCE_ID:-$(new_instance_id)}"
export TIANJI_COORDINATOR_INSTANCE_ID="${TIANJI_COORDINATOR_INSTANCE_ID:?必须由run_session注入 TIANJI_COORDINATOR_INSTANCE_ID}"
export TIANJI_ARM_CONFIG="${TIANJI_ARM_CONFIG:-$(canonical_config robot/arm.yaml)}"
export TIANJI_HAND_CONFIG="${TIANJI_HAND_CONFIG:-$(canonical_config robot/wuji_hand2.yaml)}"
export TIANJI_DEVICE_CONFIG="${TIANJI_DEVICE_CONFIG:-$(canonical_config robot/devices.yaml)}"
device_config_value() {
  pixi run python - "${TIANJI_DEVICE_CONFIG}" "$@" <<'PY'
import sys
import yaml

value = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
for key in sys.argv[2:]:
    value = value[key]
if not isinstance(value, str) or not value.strip():
    raise SystemExit("device config value must be a non-empty string")
print(value.strip())
PY
}
case "${executor_id}" in
  mujoco)
    args=(--coordinator-instance-id "${TIANJI_COORDINATOR_INSTANCE_ID}" --publisher-instance-id "${TIANJI_COMPONENT_INSTANCE_ID}" --config "${config}")
    [[ "${headless}" == true ]] && args+=(--headless)
    [[ -n "${TIANJI_RUN_ID:-}" ]] && args+=(--run-id "${TIANJI_RUN_ID}")
    exec python "${entry}" "${args[@]}" "$@"
    ;;
  marvin)
    args=(--confirm-real --config "${config}")
    marvin_robot_ip="${MARVIN_ROBOT_IP:-$(device_config_value marvin ip)}"
    args+=(--robot-ip "${marvin_robot_ip}")
    export TIANJI_REAL_CAPABILITY_PROVIDER="${real_capability_provider:-tianji_teleop.executors.marvin.preflight:trusted_real_capability}"
    exec python "${entry}" "${args[@]}" "$@"
    ;;
  wuji_hand2)
    [[ "${mode}" == direct || "${mode}" == retarget ]] || { printf '%s\n' 'Wuji executor requires --mode direct|retarget' >&2; exit 2; }
    export TIANJI_WUJI_CONFIG="${TIANJI_HAND_CONFIG}"
    rate="$(
      pixi run python - "${config}" <<'PY'
import sys
import yaml
value = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
print(value.get("rate_hz", 100))
PY
    )"
    args=(--mode "${mode}" --side "${side}" --rate "${rate}")
    if [[ "${required_capability}" == real && "${side}" == right ]]; then
      wuji_serial="${TIANJI_WUJI_SERIAL:-$(device_config_value wuji_hand2 right serial)}"
      args+=(--serial "${wuji_serial}")
    fi
    native=""
    for candidate in \
      "${BUNDLE_ROOT}/staging/ik/lib/tianji_teleop/wuji_hand2_bridge" \
      "${BUNDLE_ROOT}/runtime/tianji_teleop/lib/tianji_teleop/wuji_hand2_bridge.bin" \
      "${PROJECT_PREFIX}/lib/tianji_teleop/wuji_hand2_bridge"; do
      if [[ -x "${candidate}" ]]; then native="${candidate}"; break; fi
    done
    if [[ "${required_capability}" == simulation ]]; then
      if [[ -n "${native}" ]]; then
        exec "${native}" "${args[@]}" --dry-run "$@"
      fi
      exec python "${entry}" "${args[@]}" --dry-run "$@"
    fi
    if [[ -z "${native}" ]]; then
      printf '%s\n' '错误：real Wuji executor requires the native wuji_hand2_bridge with SDK support; refusing Python no-op fallback.' >&2
      exit 1
    fi
    exec "${native}" "${args[@]}" "$@"
    ;;
esac
