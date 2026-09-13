#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

if [[ "${PIXI_PROJECT_ROOT:-}" != "$repo_root" ]]; then
  echo "请先执行: cd $repo_root && pixi shell" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export EXO_REQUESTED_ROS_DOMAIN_ID="${EXO_REQUESTED_ROS_DOMAIN_ID:-$ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"
config_dir="${PICO_TRACKER_CONFIG_DIR:-$HOME/.config/pico_tracker}"
config_dir="$(cd "$config_dir" 2>/dev/null && pwd -P || true)"
if [[ -z "$config_dir" ]]; then
  echo "PICO_TRACKER_CONFIG_DIR does not exist or is not accessible" >&2
  exit 2
fi
export PICO_TRACKER_CONFIG_DIR="$config_dir"

set +u
source "$repo_root/install/local_setup.bash"
set -u

left_geometry="$config_dir/pico_left_arm_geometry.yaml"
right_geometry="$config_dir/pico_right_arm_geometry.yaml"
left_tcp="$config_dir/pico_left_palm_tcp.yaml"
right_tcp="$config_dir/pico_right_palm_tcp.yaml"
left_wrist="$config_dir/pico_left_wrist_pivot.yaml"
right_wrist="$config_dir/pico_right_wrist_pivot.yaml"

runtime_arguments=(
  --domain "$ROS_DOMAIN_ID"
  --left-tcp "$left_tcp"
  --right-tcp "$right_tcp"
  --left-wrist "$left_wrist"
  --right-wrist "$right_wrist"
)
valid_geometry_count=0
for side in left right; do
  if [[ "$side" == "left" ]]; then
    geometry="$left_geometry"
    tcp="$left_tcp"
    wrist="$left_wrist"
  else
    geometry="$right_geometry"
    tcp="$right_tcp"
    wrist="$right_wrist"
  fi
  if python "$repo_root/src/pico_bridge/scripts/pico_calibration_artifact.py" \
      validate --path "$geometry" --kind geometry --side "$side" \
      --tcp-path "$tcp" --wrist-path "$wrist" >/dev/null 2>&1; then
    runtime_arguments+=("--${side}-geometry" "$geometry" "--require-${side}-geometry")
    valid_geometry_count=$((valid_geometry_count + 1))
    echo "✓ $side 个体骨长已通过严格验证，将启用掌心约束修正"
  else
    echo "⚠ $side 个体骨长或其上游 artifact 未通过严格验证；该侧不加载严格个体骨长，使用运行时 SMPL baseline" >&2
    echo "  如需修正该侧，请运行: ./scripts/calibrate_pico_arm.sh $side all" >&2
  fi
done

if ((valid_geometry_count == 0)); then
  echo "至少需要一侧完成有效标定后才能启动 PICO M0 可视化。" >&2
  exit 2
fi

exec "$repo_root/src/pico_bridge/scripts/start_pico_m0_pixi.sh" \
  "${runtime_arguments[@]}" \
  "$@"
