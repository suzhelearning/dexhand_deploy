#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

acquire_teleop_guard test
preview_log=""
input_log=""
cleanup_preview() {
  local cleanup_failed=0
  trap - EXIT INT TERM
  teleop_cleanup_and_release || cleanup_failed=1
  if [[ -n "${preview_log}" ]]; then
    rm -f -- "${preview_log}"
  fi
  if [[ -n "${input_log}" ]]; then
    rm -f -- "${input_log}"
  fi
  return "${cleanup_failed}"
}
stop_on_signal() {
  cleanup_preview
  exit 130
}
trap cleanup_preview EXIT
trap stop_on_signal INT TERM

"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

python -m unittest \
  tests.test_target_mapper \
  tests.test_canonical_sources \
  tests.test_task3_round4 \
  tests.test_mocap_keyboard_step \
  tests.test_controller_only_host_readiness \
  tests.test_controller_only_real_profile \
  tests.test_mocap_h5 \
  tests.test_h5_replay \
  tests.test_mocap_h5_replay \
  tests.e2e_wuji_hand2_dry

# 优先用 staging 调试版；未构建时退回 runtime 部署的 .bin。
IK_NODE="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/tianji_kinematic_sim"
if [[ ! -x "${IK_NODE}" ]]; then
  IK_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim.bin"
fi
if [[ ! -x "${IK_NODE}" ]]; then
  printf '错误：可配置 IK 节点未生成：%s\n' "${IK_NODE}" >&2
  exit 1
fi
RUNTIME_SHARE="${PROJECT_PREFIX}/share/pico_body_tianji"
PREVIEW_URDF="${RUNTIME_SHARE}/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
CONTROLLER_ONLY_CONFIG="${RUNTIME_SHARE}/config/mode/controller_only/controller_only_ik.yaml"
PREVIEW_CONFIG="${RUNTIME_SHARE}/config/mode/full_body/preview.yaml"
HANGING_WORKER="${BUNDLE_ROOT}/tests/fake_hanging_official_ik_worker.sh"

# 官方 IK 后端配假挂死 worker：deadline/重启保护必须在 2 秒内触发。
worker_timeout_log="$(mktemp)"
mapfile -t ik_official_params < <(
  yaml_params_for tianji_kinematic_sim \
    "${CONTROLLER_ONLY_CONFIG}" \
    "urdf_path:=${PREVIEW_URDF}" \
    ik_backend:=tianji_official
)
if [[ "${#ik_official_params[@]}" -eq 0 ]]; then
  printf '%s\n' '错误：无法从 controller_only_ik.yaml 生成 IK 参数。' >&2
  exit 1
fi
set +e
TIANJI_OFFICIAL_IK_WORKER="${HANGING_WORKER}" \
TIANJI_OFFICIAL_IK_LIBRARY="${BUNDLE_ROOT}/runtime/tianji_official/kinematicsSDK/libKine.so" \
TIANJI_OFFICIAL_IK_CONFIG="${BUNDLE_ROOT}/runtime/tianji_official/CommonConfig/ccs_m6_40.MvKDCfg" \
  timeout 2 "${IK_NODE}" "${ik_official_params[@]/#--param /}" \
  >"${worker_timeout_log}" 2>&1
worker_timeout_exit=$?
set -e
if [[ "${worker_timeout_exit}" -eq 124 ]] ||
   ! grep -Fq '官方 IK worker 恢复失败' "${worker_timeout_log}"
then
  cat "${worker_timeout_log}" >&2
  rm -f -- "${worker_timeout_log}"
  printf '%s\n' '错误：官方 IK worker deadline/重启保护未按时生效。' >&2
  exit 1
fi
rm -f -- "${worker_timeout_log}"

# runtime 安装目录存在 + preview.yaml 的 ik_backend 可读且合法。
if [[ ! -d "${RUNTIME_SHARE}" ]]; then
  printf '错误：缺少 runtime 安装目录：%s\n' "${RUNTIME_SHARE}" >&2
  exit 1
fi
python - "${PREVIEW_CONFIG}" <<'PY'
import sys

import yaml

path = sys.argv[1]
with open(path, encoding="utf-8") as fh:
    data = yaml.safe_load(fh) or {}
section = data.get("tianji_kinematic_sim", data)
if isinstance(section, dict) and "ros__parameters" in section:
    section = section["ros__parameters"]
backend = section.get("ik_backend", "")
if backend not in ("pinocchio_cpp", "pinocchio_qp", "tianji_official"):
    raise SystemExit(f"错误：{path} 中的 ik_backend={backend!r} 无效")
print("Runtime 安装目录检查通过：", path, "ik_backend=", backend)
PY

# IK 纯运动学节点直接启动（--param key:=value），3 秒超时。
mapfile -t ik_preview_params < <(
  yaml_params_for tianji_kinematic_sim \
    "${PREVIEW_CONFIG}" \
    "urdf_path:=${PREVIEW_URDF}"
)
if [[ "${#ik_preview_params[@]}" -eq 0 ]]; then
  printf '%s\n' '错误：无法从 preview.yaml 生成 IK 参数。' >&2
  exit 1
fi
setsid timeout 3 "${IK_NODE}" "${ik_preview_params[@]/#--param /}" &
ik_probe_pid=$!
register_teleop_process_group \
  "${ik_probe_pid}" portable-test-ik-probe 5
set +e
wait "${ik_probe_pid}"
ik_exit=$?
set -e
if [[ "${ik_exit}" -ne 124 && "${ik_exit}" -ne 0 ]]; then
  printf '错误：可配置 IK 纯运动学节点启动失败：%s\n' \
    "${ik_exit}" >&2
  exit "${ik_exit}"
fi

# 仿真链路：IK（预览参数 + urdf）与 PICO 输入节点，无 RViz / robot_state_publisher。
preview_log="$(mktemp)"
input_log="$(mktemp)"
ik_preview_pid=""
input_preview_pid=""

setsid "${IK_NODE}" "${ik_preview_params[@]/#--param /}" \
  >"${preview_log}" 2>&1 &
ik_preview_pid=$!
register_teleop_process_group "${ik_preview_pid}" portable-test-ik-preview 5

mapfile -t input_preview_params < <(
  yaml_params_for pico_controller_input "${PREVIEW_CONFIG}"
)
if [[ "${#input_preview_params[@]}" -eq 0 ]]; then
  printf '%s\n' '错误：无法从 preview.yaml 生成输入节点参数。' >&2
  exit 1
fi
input_preview_arguments=()
for input_param in "${input_preview_params[@]}"; do
  input_preview_arguments+=("--param" "${input_param}")
done
setsid python "${BUNDLE_ROOT}/src/pico_body_tianji/scripts/pico_controller_input" \
  "${input_preview_arguments[@]}" >"${input_log}" 2>&1 &
input_preview_pid=$!
register_teleop_process_group "${input_preview_pid}" portable-test-input-preview 5

sleep 3

if ! kill -0 "${ik_preview_pid}" 2>/dev/null; then
  cat "${preview_log}" >&2
  printf '%s\n' \
    '错误：IK 纯运动学仿真链路提前退出。' >&2
  exit 1
fi
if ! kill -0 "${input_preview_pid}" 2>/dev/null; then
  cat "${input_log}" >&2
  printf '%s\n' \
    '错误：PICO 输入节点仿真链路提前退出。' >&2
  exit 1
fi
if grep -Eq \
  'process has died|Failed to load entry point|Traceback \(most recent call last\)' \
  "${preview_log}" "${input_log}"
then
  cat "${preview_log}" "${input_log}" >&2
  printf '%s\n' \
    '错误：仿真链路中的节点异常退出。' >&2
  exit 1
fi

# 断言 IK 持续发布 model_joint_states（zenoh JSON）。
python - <<'PY'
import json
import sys
import threading

import zenoh

received = []
done = threading.Event()


def handler(sample):
    received.append(json.loads(bytes(sample.payload)))
    done.set()


session = zenoh.open(zenoh.Config())
try:
    session.declare_subscriber("pico_body_sim/model_joint_states", handler)
    done.wait(3.0)
finally:
    session.close()
if not received:
    print("错误：未收到 model_joint_states", file=sys.stderr)
    raise SystemExit(1)
message = received[0]
names = message.get("name", [])
positions = message.get("position", [])
if len(names) != 14 or len(positions) != 14:
    print(f"错误：model_joint_states 字段异常：{names=} {positions=}", file=sys.stderr)
    raise SystemExit(1)
print("IK model_joint_states 输出正常：", names[0], positions[0])
PY

# zenoh liveliness 断言：IK 与输入节点必须在线。
node_list="$(read_teleop_node_list)"
for required_node in \
  /tianji_kinematic_sim \
  /pico_controller_input
do
  if ! grep -Fxq "${required_node}" <<<"${node_list}"; then
    cat "${preview_log}" "${input_log}" >&2
    printf '错误：纯仿真节点未启动：%s\n' "${required_node}" >&2
    exit 1
  fi
done

cleanup_preview
trap - EXIT INT TERM

printf '%s\n' \
  '便携包 Zenoh + 可配置 IK + 仿真链路验证通过；未连接实体机械臂。'
