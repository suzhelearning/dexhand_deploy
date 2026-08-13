#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

activate_bundle_runtime
exec python "${ROS_ROOT}/bin/ros2" topic echo \
  /pico_body_sim/model_joint_states sensor_msgs/msg/JointState
