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

CONFIG_ROOT="${BUNDLE_ROOT}/src/tianji_teleop/config"
required_configs=(
  robot/arm.yaml robot/wuji_hand2.yaml robot/devices.yaml
  sources/mocap_live.yaml sources/h5_replay.yaml
  producers/ik.yaml producers/ik_regrind.yaml producers/policy_hold.yaml
  coordinator/arm.yaml coordinator/arm_regrind.yaml
  executors/mujoco.yaml executors/marvin.yaml executors/marvin_impedance.yaml
  executors/wuji_hand2.yaml executors/wuji_hand2_regrind.yaml
  recording/session.yaml replay/target.yaml replay/joint.yaml
  diagnostics/mocap_calibration.yaml
  sessions/mocap_live_sim.yaml sessions/mocap_live_real.yaml
  sessions/h5_sim.yaml sessions/h5_real.yaml
  sessions/regrind_real.yaml
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
PYTHONPATH="${BUNDLE_ROOT}/src/tianji_teleop:${PYTHONPATH:-}" python - "${CONFIG_ROOT}" <<'PY'
from pathlib import Path
import sys
import yaml
from tianji_teleop.config_loader import load_component_config
from tianji_teleop.coordination.arm_command_coordinator import ArmRobotConfig
from tianji_teleop.executors.wuji_hand2.config import WujiHandConfig

root = Path(sys.argv[1])
arm = ArmRobotConfig.load(root / "robot/arm.yaml")
hand = WujiHandConfig.load(root / "robot/wuji_hand2.yaml")
shared_executor = yaml.safe_load((root / "executors/wuji_hand2.yaml").read_text(encoding="utf-8")) or {}
regrind_executor = yaml.safe_load((root / "executors/wuji_hand2_regrind.yaml").read_text(encoding="utf-8")) or {}
assert float(shared_executor["rate_hz"]) == 60.0
assert shared_executor.get("linear_interpolation") is False
assert float(regrind_executor["rate_hz"]) == 100.0
assert regrind_executor.get("linear_interpolation") is True
shared_coordinator = yaml.safe_load((root / "coordinator/arm.yaml").read_text(encoding="utf-8")) or {}
regrind_coordinator = yaml.safe_load((root / "coordinator/arm_regrind.yaml").read_text(encoding="utf-8")) or {}
regrind_profile = yaml.safe_load((root / "sessions/regrind_real.yaml").read_text(encoding="utf-8")) or {}
assert float(shared_coordinator["maximum_command_step_rad"]) == 0.00596902599
assert float(regrind_coordinator["maximum_command_step_rad"]) == 1000.0
assert regrind_profile["coordinator_config"] == "coordinator/arm_regrind.yaml"
assert len(arm.left_joint_names) == len(arm.right_joint_names) == 7
assert len(hand.joint_names) == 20
for path in root.glob("sessions/*.yaml"):
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if "router_endpoint" in value or "ik_backend" in value:
        raise ValueError(f"session contains forbidden authority field: {path}")
for name in ("ik.yaml", "ik_regrind.yaml"):
    ik = load_component_config(root / "producers" / name, required_keys={"ik_backend", "arm_config"})
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
  "${BUNDLE_ROOT}/vendor/python/marvin_sdk/libMarvinSDK.so"
  "${ZENOH_LIBRARY_ROOT}/libzenohc.so"
  "${ZENOH_CPP_INCLUDE_ROOT}/zenoh.hxx"
  "${BUNDLE_ROOT}/vendor/wuji-sdk/include/wuji_sdk.h"
  "${BUNDLE_ROOT}/vendor/wuji-sdk/lib/libwuji_sdk_c.so"
  "${BUNDLE_ROOT}/src/tianji_teleop/assets/tianji_wuji2/tianji_wuji2.urdf"
)
for file in "${required_files[@]}"; do
  if [[ ! -e "${file}" ]]; then
    printf '错误：缺少运行时文件：%s\n' "${file}" >&2
    exit 1
  fi
done
runtime_bin="${PROJECT_PREFIX}/lib/tianji_teleop"
allowed_programs=(
  arm_ik_producer tianji_official_ik_probe tianji_official_ik_worker
  wuji_hand2_bridge mocap_live mocap_h5_replay
  target_replay joint_replay session_recorder arm_command_coordinator
  policy_hold_producer mujoco_executor marvin_executor wuji_hand2_executor
  trace_metrics real_diagnostic h5_wrist_diagnostic joint_watcher
  mocap_calibration
)
for artifact in "${runtime_bin}"/*; do
  [[ -e "${artifact}" ]] || continue
  program="$(basename -- "${artifact}")"
  program="${program%.bin}"
  case " ${allowed_programs[*]} " in
    *" ${program} "*) ;;
    *) printf '错误：runtime 存在未授权的非 canonical entry: %s\n' "${artifact}" >&2; exit 1 ;;
  esac
done
required_programs=("${allowed_programs[@]}")
for program in "${required_programs[@]}"; do
  if [[ ! -x "${runtime_bin}/${program}" && ! -x "${runtime_bin}/${program}.bin" ]]; then
    printf '错误：runtime 缺少 canonical entry: %s\n' "${program}" >&2
    exit 1
  fi
done
runtime_config="${PROJECT_PREFIX}/share/tianji_teleop/config"
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
