#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)"
ROS_ROOT="${BUNDLE_ROOT}/runtime/ros/humble"
ABI_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/abi/lib"
PIN_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/pin/lib"
IK_BINARY="${SCRIPT_DIR}/tianji_kinematic_sim.bin"

ros_library_path=""
if [[ -d "${ROS_ROOT}/lib/x86_64-linux-gnu" ]]; then
  ros_library_path="${ROS_ROOT}/lib/x86_64-linux-gnu"
fi
while IFS= read -r -d '' library_dir; do
  ros_library_path+="${ros_library_path:+:}${library_dir}"
done < <(find "${ROS_ROOT}" -type d -name lib -print0)

export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export TIANJI_OFFICIAL_IK_WORKER="${SCRIPT_DIR}/tianji_official_ik_worker"
export TIANJI_OFFICIAL_IK_LIBRARY="${BUNDLE_ROOT}/runtime/tianji_official/kinematicsSDK/libKine.so"
export TIANJI_OFFICIAL_IK_CONFIG="${BUNDLE_ROOT}/runtime/tianji_official/CommonConfig/ccs_m6_40.MvKDCfg"
exec "${ABI_LIBRARY_ROOT}/ld-linux-x86-64.so.2" \
  --library-path \
  "${PIN_LIBRARY_ROOT}:${SCRIPT_DIR}:${ros_library_path}:${ABI_LIBRARY_ROOT}" \
  "${IK_BINARY}" "$@"
