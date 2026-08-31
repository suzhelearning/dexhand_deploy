#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

producer_id=""
backend=""
config_override=""
while (($#)); do
  case "$1" in
    --producer|--profile) producer_id="${2:-}"; shift 2 ;;
    --backend) backend="${2:-}"; shift 2 ;;
    --config) config_override="${2:-}"; shift 2 ;;
    --help|-h)
      printf '%s\n' '用法: run_producer.sh --producer {ik|policy_hold} [--backend BACKEND] [--config PATH]'
      exit 0 ;;
    --) shift; break ;;
    *) break ;;
  esac
done
if [[ -z "${producer_id}" ]]; then
  printf '%s\n' '错误：必须指定 --producer。' >&2
  exit 2
fi
case "${producer_id}" in
  ik)
    entry="${BUNDLE_ROOT}/staging/ik/lib/tianji_teleop/arm_ik_producer"
    [[ -x "${entry}" ]] || entry="${PROJECT_PREFIX}/lib/tianji_teleop/arm_ik_producer.bin"
    default_config="producers/ik.yaml"
    ;;
  policy_hold)
    entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/policy_hold_producer"
    default_config="producers/policy_hold.yaml"
    ;;
  *) printf '错误：未知 producer: %s\n' "${producer_id}" >&2; exit 2 ;;
esac
if [[ ! -x "${entry}" ]]; then
  printf '错误：producer entry 不存在或不可执行: %s\n' "${entry}" >&2
  exit 1
fi
config="${config_override}"
if [[ -z "${config}" ]]; then config="$(canonical_config "${default_config}")"; fi
if [[ "${producer_id}" == ik ]]; then
  requested_backend="${TIANJI_VALIDATION_IK_BACKEND:-${backend:-}}"
  eval "$(
    pixi run python - "${config}" <<'PY'
import sys
import yaml
value = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
if not isinstance(value, dict):
    raise SystemExit("IK config must be a mapping")
required = {
    "ik_backend", "arm_config", "rate_hz", "freshness_timeout_s",
    "solver_reject_grace_s", "ruckig_max_velocity_rad_s",
    "ruckig_max_acceleration_rad_s2", "ruckig_max_jerk_rad_s3",
    "ruckig_validation_tolerance",
    "max_iterations", "position_tolerance_m", "orientation_tolerance_rad",
    "minimum_damping", "maximum_damping", "singular_value_threshold",
    "maximum_iteration_step_rad", "maximum_joint_step_rad",
    "joint_limit_margin_rad", "arm_angle_gain", "arm_angle_tolerance_rad",
    "arm_angle_finite_difference_rad", "arm_angle_merit_weight",
    "nullspace_damping", "joint_center_gain", "joint_center_activation_margin_rad",
    "joint_center_merit_weight", "singularity_avoidance_gain",
    "singularity_finite_difference_rad", "singularity_merit_weight",
    "control_period_s", "qp_position_time_constant_s",
    "qp_orientation_time_constant_s", "qp_max_linear_speed_m_s",
    "qp_max_angular_speed_rad_s", "qp_joint_velocity_limits_rad_s",
    "qp_position_weight", "qp_orientation_weight",
    "qp_velocity_regularization_weight", "qp_continuity_weight",
    "qp_posture_weight", "qp_posture_time_constant_s", "qp_left_nominal_rad",
    "qp_right_nominal_rad", "qp_joint_limit_activation_margin_rad",
    "qp_joint_limit_velocity_damper_gain", "qp_singularity_critical_threshold",
    "qp_singularity_orientation_scale", "qp_singularity_posture_multiplier",
    "qp_singularity_velocity_multiplier", "qp_singularity_escape_weight",
    "qp_singularity_escape_speed_rad_s", "qp_max_active_set_iterations",
    "qp_active_set_tolerance", "official_use_zsp", "official_dgr1",
    "official_dgr2", "official_dgr3", "official_joint_limit_soft_margin_rad",
    "official_candidate_continuity_weight", "official_candidate_limit_weight",
    "official_candidate_posture_weight", "official_left_nominal_rad",
    "official_right_nominal_rad", "official_orientation_relaxation_steps",
    "official_workspace_backoff_iterations", "worker_timeout_ms",
    "worker_restart_attempts", "capabilities",
}
missing = required - set(value)
if missing:
    raise SystemExit(f"IK config missing canonical fields: {sorted(missing)}")
for key, item in value.items():
    name = "BACKEND" if key == "ik_backend" else key.upper()
    import shlex
    print(f"export TIANJI_IK_{name}={shlex.quote(str(item))}")
PY
  )"
  [[ -z "${requested_backend}" ]] || export TIANJI_IK_BACKEND="${requested_backend}"
fi
if [[ "${producer_id}" == ik ]]; then
  arm_config="$(canonical_config robot/arm.yaml)"
  urdf_path="${TIANJI_ARM_URDF:-${BUNDLE_ROOT}/src/tianji_teleop/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf}"
  export TIANJI_ARM_CONFIG="${TIANJI_ARM_CONFIG:-${arm_config}}"
  export TIANJI_ARM_URDF="${urdf_path}"
  export TIANJI_IK_BACKEND="${TIANJI_IK_BACKEND:-${backend:-pinocchio_qp}}"
  exec "${entry}" "$@"
fi
exec python "${entry}" "$@"
