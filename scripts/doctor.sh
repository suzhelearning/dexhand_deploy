#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

if [[ "$(uname -m)" != "x86_64" ]]; then
  printf '错误：厂商 SDK 仅支持 x86_64。\n' >&2
  exit 1
fi
activate_bundle_runtime

CONFIG_ROOT="${BUNDLE_ROOT}/src/pico_body_tianji/config"
required_configs=(
  robot/arm.yaml robot/wuji_hand2.yaml
  sources/pico_controller.yaml sources/mocap_live.yaml sources/h5_replay.yaml
  producers/ik.yaml producers/policy_hold.yaml
  coordinator/arm.yaml
  executors/mujoco.yaml executors/marvin.yaml executors/wuji_hand2.yaml
  recording/session.yaml replay/target.yaml replay/joint.yaml
  diagnostics/mocap_calibration.yaml
  sessions/pico_sim.yaml sessions/pico_real.yaml
  sessions/mocap_live_sim.yaml sessions/mocap_live_real.yaml
  sessions/h5_sim.yaml sessions/h5_real.yaml
  sessions/target_replay_sim.yaml sessions/joint_replay_sim.yaml
  sessions/diagnostic_mocap_calibration_sim.yaml
)
for relative in "${required_configs[@]}"; do
  if [[ ! -f "${CONFIG_ROOT}/${relative}" ]]; then
    printf '错误：缺少 canonical config: %s\n' "${relative}" >&2
    exit 1
  fi
done

# Validate shape, strict session authority split, and robot config through the
# same loaders used by production nodes.
PYTHONPATH="${BUNDLE_ROOT}/src/pico_body_tianji:${PYTHONPATH:-}" python - "${CONFIG_ROOT}" <<'PY'
from pathlib import Path
import sys
import yaml
from pico_body_tianji.config_loader import load_component_config
from pico_body_tianji.coordination.arm_command_coordinator import ArmRobotConfig
from pico_body_tianji.executors.wuji_hand2.config import WujiHandConfig

root = Path(sys.argv[1])
arm = ArmRobotConfig.load(root / "robot/arm.yaml")
hand = WujiHandConfig.load(root / "robot/wuji_hand2.yaml")
assert len(arm.left_joint_names) == len(arm.right_joint_names) == 7
assert len(hand.joint_names) == 20
for path in root.glob("sessions/*.yaml"):
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if "router_endpoint" in value or "ik_backend" in value:
        raise ValueError(f"session contains forbidden authority field: {path}")
ik = load_component_config(root / "producers/ik.yaml", required_keys={"ik_backend", "arm_config"})
if ik["ik_backend"] not in {"pinocchio_cpp", "pinocchio_qp", "tianji_official"}:
    raise ValueError("unsupported IK backend")
print("canonical config schema passed")
PY

# A doctor never starts a router. It connects to the operator-managed endpoint
# and rejects zero/multiple router identities before checking liveness.
router_zid=""
if ! router_zid="$(require_router)"; then
  exit 1
fi
printf '%s\n' "router endpoint=${TIANJI_ROUTER_ENDPOINT:-tcp/127.0.0.1:7447} router_zid=${router_zid}"

required_files=(
  "${BUNDLE_ROOT}/vendor/python/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so"
  "${BUNDLE_ROOT}/vendor/lib/libPXREARobotSDK.so"
  "${BUNDLE_ROOT}/vendor/python/marvin_sdk/libMarvinSDK.so"
  "${ZENOH_LIBRARY_ROOT}/libzenohc.so"
  "${ZENOH_CPP_INCLUDE_ROOT}/zenoh.hxx"
  "${BUNDLE_ROOT}/vendor/wuji-sdk/include/wuji_sdk.h"
  "${BUNDLE_ROOT}/vendor/wuji-sdk/lib/libwuji_sdk_c.so"
  "${BUNDLE_ROOT}/src/pico_body_tianji/assets/tianji_wuji2/tianji_wuji2.urdf"
)
for file in "${required_files[@]}"; do
  if [[ ! -e "${file}" ]]; then
    printf '错误：缺少运行时文件：%s\n' "${file}" >&2
    exit 1
  fi
done
runtime_bin="${PROJECT_PREFIX}/lib/pico_body_tianji"
required_programs=(arm_ik_producer tianji_official_ik_probe tianji_official_ik_worker wuji_hand2_bridge pico_controller_source mocap_live mocap_h5_replay target_replay joint_replay session_recorder arm_command_coordinator policy_hold_producer mujoco_executor marvin_executor wuji_hand2_executor trace_metrics real_diagnostic h5_wrist_diagnostic joint_watcher)
for program in "${required_programs[@]}"; do
  if [[ ! -x "${runtime_bin}/${program}" && ! -x "${runtime_bin}/${program}.bin" ]]; then
    printf '错误：runtime 缺少 canonical entry: %s\n' "${program}" >&2
    exit 1
  fi
done
for stale in \
  "${runtime_bin}"/marvin_hardware_* \
  "${runtime_bin}"/controller_only* \
  "${runtime_bin}"/pico_controller_input* \
  "${runtime_bin}"/pico_link_probe* \
  "${runtime_bin}"/mocap_keyboard_step* \
  "${runtime_bin}"/mujoco_joint_viewer* \
  "${runtime_bin}"/mujoco_h5_wrist_replay* \
  "${runtime_bin}"/tianji_kinematic_sim*; do
  [[ ! -e "${stale}" ]] || { printf '错误：runtime 存在过时入口：%s\n' "${stale}" >&2; exit 1; }
done
runtime_config="${PROJECT_PREFIX}/share/pico_body_tianji/config"
for stale_config in "${runtime_config}/mode"; do
  [[ ! -e "${stale_config}" ]] || { printf '错误：runtime 存在过时配置目录：%s\n' "${stale_config}" >&2; exit 1; }
done

if [[ -f "${BUNDLE_ROOT}/RUNTIME_TREE_SHA256" ]]; then
  runtime_hash="$({ cd "${BUNDLE_ROOT}"; find runtime -type f ! -name '*.pyc' ! -path '*/__pycache__/*' -print0 | LC_ALL=C sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1; })"
  expected_hash="$(cut -d' ' -f1 "${BUNDLE_ROOT}/RUNTIME_TREE_SHA256")"
  if [[ -z "${expected_hash}" || "${runtime_hash}" != "${expected_hash}" ]]; then
    printf '%s\n' '错误：runtime tree hash 不匹配，请重新 deploy-ik。' >&2
    exit 1
  fi
fi
printf '%s\n' 'doctor passed；未启动第二 router，未连接实体设备。'
