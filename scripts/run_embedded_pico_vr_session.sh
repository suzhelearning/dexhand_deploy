#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
source "${SCRIPT_DIR}/common.sh"
source "${SCRIPT_DIR}/graceful_process_stop.sh"
BUNDLE_ROOT="${ROOT}/vendor/pico_tracker"

profile=""
resolve_only=false
disable_hands=false
display_mode=""
record_path=""
spark_overlay=false
tjvr_bind="127.0.0.1"
tjvr_port="15000"
duration_s=""
manus_rawviz=""
manus_user=""
manus_library_dir=""
right_glove=""
left_glove=""
ik_backend=""
arm_pose_mapper=""
arm_target_processor=""
joint_trajectory=""
command_step_clipping=""
joint_limit_source=""
operator_input=""
arm_input=""
pico_calibration_dir="${PICO_TRACKER_CONFIG_DIR:-${HOME}/.config/pico_tracker}"
pico_app_package="com.PICO.wholebody_stream.unity"
pico_ros_domain="${PICO_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-120}}"
pico_startup_timeout_s="20"
adb_serial="${ADB_SERIAL:-}"
pico_calibration_dir_explicit=false
pico_app_package_explicit=false
pico_ros_domain_explicit=false
pico_startup_timeout_explicit=false
adb_serial_explicit=false

usage() {
  cat <<'EOF'
用法: run_embedded_pico_vr_session.sh --profile pico_vr_manus_sim [选项]

启动内嵌的原版 PICO wholebody driver、M0 修正骨架、TJVR UDP bridge，
再连接当前工程已有的 vr_manus_sim 下游。该入口只支持仿真，不驱动真机。

输入选项：
  --disable-hands
  --manus-rawviz PATH --manus-user USER
  --manus-library-dir PATH --right-glove ID --left-glove ID
  --tjvr-bind HOST --tjvr-port PORT --duration-s SECONDS
  --record PATH --spark-overlay --viewer|--headless

PICO 上游选项：
  --pico-calibration-dir PATH
  --pico-app-package PACKAGE
  --pico-ros-domain ID
  --pico-startup-timeout-s SECONDS
  --adb-serial SERIAL（多台 ADB 设备时可选）
EOF
}

fail() {
  printf '错误：%s\n' "$*" >&2
  exit 2
}

is_decimal() {
  [[ "${1:-}" =~ ^[0-9]+([.][0-9]+)?$ ]]
}

is_positive_decimal() {
  is_decimal "$1" && awk -v value="$1" 'BEGIN { exit !(value > 0) }'
}

normalize_existing_dir() {
  local value="$1"
  [[ -d "$value" ]] || return 1
  (cd -- "$value" && pwd -P)
}

while (($#)); do
  case "$1" in
    --profile) profile="${2:-}"; shift 2 ;;
    --resolve-only) resolve_only=true; shift ;;
    --disable-hands) disable_hands=true; shift ;;
    --viewer)
      [[ -z "$display_mode" || "$display_mode" == viewer ]] || fail '--viewer 与 --headless 互斥'
      display_mode=viewer; shift ;;
    --headless)
      [[ -z "$display_mode" || "$display_mode" == headless ]] || fail '--viewer 与 --headless 互斥'
      display_mode=headless; shift ;;
    --record) record_path="${2:-}"; shift 2 ;;
    --spark-overlay) spark_overlay=true; shift ;;
    --tjvr-bind) tjvr_bind="${2:-}"; shift 2 ;;
    --tjvr-port) tjvr_port="${2:-}"; shift 2 ;;
    --duration-s) duration_s="${2:-}"; shift 2 ;;
    --manus-rawviz) manus_rawviz="${2:-}"; shift 2 ;;
    --manus-user) manus_user="${2:-}"; shift 2 ;;
    --manus-library-dir) manus_library_dir="${2:-}"; shift 2 ;;
    --right-glove) right_glove="${2:-}"; shift 2 ;;
    --left-glove) left_glove="${2:-}"; shift 2 ;;
    --ik-backend) ik_backend="${2:-}"; shift 2 ;;
    --arm-pose-mapper) arm_pose_mapper="${2:-}"; shift 2 ;;
    --arm-target-processor) arm_target_processor="${2:-}"; shift 2 ;;
    --joint-trajectory) joint_trajectory="${2:-}"; shift 2 ;;
    --command-step-clipping) command_step_clipping="${2:-}"; shift 2 ;;
    --joint-limit-source) joint_limit_source="${2:-}"; shift 2 ;;
    --operator-input) operator_input="${2:-}"; shift 2 ;;
    --arm-input) arm_input="${2:-}"; shift 2 ;;
    --pico-calibration-dir) pico_calibration_dir="${2:-}"; pico_calibration_dir_explicit=true; shift 2 ;;
    --pico-app-package) pico_app_package="${2:-}"; pico_app_package_explicit=true; shift 2 ;;
    --pico-ros-domain) pico_ros_domain="${2:-}"; pico_ros_domain_explicit=true; shift 2 ;;
    --pico-startup-timeout-s) pico_startup_timeout_s="${2:-}"; pico_startup_timeout_explicit=true; shift 2 ;;
    --adb-serial) adb_serial="${2:-}"; adb_serial_explicit=true; shift 2 ;;
    --pico-overlay|--ik-target-overlay|--xr-overlay)
      fail "$1 不属于 pico_vr_manus_sim；PICO 原始可视化请使用内嵌 M0 的 --viewer"
      ;;
    --h5|--input|--speed|--observation-config|--confirm-real)
      fail "$1 不属于 pico_vr_manus_sim"
      ;;
    --help|-h) usage; exit 0 ;;
    *) fail "不支持的参数: $1" ;;
  esac
done

[[ "$profile" == pico_vr_manus_sim ]] || fail '必须使用 --profile pico_vr_manus_sim'
[[ -f "$BUNDLE_ROOT/pixi.toml" && -f "$BUNDLE_ROOT/pixi.lock" ]] ||
  fail "缺少内嵌 PICO Pixi manifest: $BUNDLE_ROOT"

if [[ -n "$pico_ros_domain" && "$pico_ros_domain" =~ ^[0-9]+$ ]]; then
  ((pico_ros_domain >= 0 && pico_ros_domain <= 232)) || fail 'PICO ROS domain 必须在 0..232'
else
  fail 'PICO ROS domain 必须是 0..232 的整数'
fi
is_positive_decimal "$pico_startup_timeout_s" || fail 'PICO startup timeout 必须是正数'
if [[ -n "$duration_s" ]]; then
  is_positive_decimal "$duration_s" || fail '--duration-s 必须是正数'
fi
[[ -n "${tjvr_bind//[[:space:]]/}" ]] || fail 'TJVR bind host 不能为空'
if [[ "$tjvr_port" =~ ^[0-9]+$ ]]; then
  ((tjvr_port >= 1 && tjvr_port <= 65535)) || fail 'TJVR port 必须在 1..65535'
else
  fail 'TJVR port 必须是整数'
fi
[[ "$pico_app_package" =~ ^[A-Za-z0-9_.]+$ ]] || fail 'PICO APK package 名称非法'
[[ "$adb_serial_explicit" != true || -n "$adb_serial" ]] || fail '--adb-serial 不能为空'
[[ -z "$adb_serial" || ! "$adb_serial" =~ [[:space:]] ]] || fail 'ADB serial 不能包含空白字符'

for value_name in ik_backend arm_pose_mapper arm_target_processor joint_trajectory \
  command_step_clipping joint_limit_source operator_input arm_input; do
  value="${!value_name}"
  case "$value_name" in
    ik_backend) [[ -z "$value" || "$value" == spark_upper_qpoases_headroom_feedforward_velocity_qp ]] || fail 'pico_vr_manus_sim 必须使用原版 Spark IK backend' ;;
    arm_pose_mapper) [[ -z "$value" || "$value" == none ]] || fail 'TJVR corrected palm 必须使用 arm-pose-mapper none' ;;
    arm_target_processor) [[ -z "$value" || "$value" == passthrough ]] || fail 'reference-direct TJVR 必须使用 passthrough target processor' ;;
    joint_trajectory) [[ -z "$value" || "$value" == passthrough ]] || fail 'reference-direct TJVR 必须使用 passthrough trajectory' ;;
    command_step_clipping) [[ -z "$value" || "$value" == false ]] || fail 'reference-direct TJVR 必须关闭 command-step-clipping' ;;
    joint_limit_source) [[ -z "$value" || "$value" == urdf ]] || fail 'reference-direct TJVR 必须使用 URDF limits' ;;
    operator_input) [[ -z "$value" || "$value" == keyboard ]] || fail 'pico_vr_manus_sim 的 operator-input 必须为 keyboard' ;;
    arm_input) [[ -z "$value" || "$value" == tjvr_corrected_palm ]] || fail 'pico_vr_manus_sim 的 arm-input 必须为 tjvr_corrected_palm' ;;
  esac
done

if [[ "$resolve_only" == true ]]; then
  [[ -z "${record_path}${duration_s}${manus_rawviz}${manus_user}${manus_library_dir}${right_glove}${left_glove}" &&
     "$pico_calibration_dir_explicit" != true &&
     "$pico_app_package_explicit" != true &&
     "$pico_ros_domain_explicit" != true &&
     "$pico_startup_timeout_explicit" != true &&
     "$adb_serial_explicit" != true &&
     -z "$display_mode" && "$spark_overlay" != true ]] ||
    fail '--resolve-only 不接受设备、Manus、TJVR 或录制参数'
  resolve_args=(--profile pico_vr_manus_sim)
  [[ "$disable_hands" != true ]] || resolve_args+=(--disable-hands)
  [[ -z "$ik_backend" ]] || resolve_args+=(--ik-backend "$ik_backend")
  [[ -z "$arm_pose_mapper" ]] || resolve_args+=(--arm-pose-mapper "$arm_pose_mapper")
  [[ -z "$arm_target_processor" ]] || resolve_args+=(--arm-target-processor "$arm_target_processor")
  [[ -z "$joint_trajectory" ]] || resolve_args+=(--joint-trajectory "$joint_trajectory")
  [[ -z "$command_step_clipping" ]] || resolve_args+=(--command-step-clipping "$command_step_clipping")
  [[ -z "$joint_limit_source" ]] || resolve_args+=(--joint-limit-source "$joint_limit_source")
  [[ -z "$operator_input" ]] || resolve_args+=(--operator-input "$operator_input")
  [[ -z "$arm_input" ]] || resolve_args+=(--arm-input "$arm_input")
  exec python3 "$ROOT/scripts/resolve_dual_session.py" "${resolve_args[@]}"
fi

[[ -f "$BUNDLE_ROOT/install/local_setup.bash" ]] ||
  fail "内嵌 PICO 尚未构建：缺少 $BUNDLE_ROOT/install/local_setup.bash；请先运行 pixi run build-embedded-pico"

if [[ "$disable_hands" == true ]]; then
  [[ -z "${manus_rawviz}${manus_user}${manus_library_dir}${right_glove}${left_glove}" ]] ||
    fail '--disable-hands 时不能提供 Manus 设备参数'
else
  [[ -n "$manus_rawviz" && -n "$manus_user" ]] ||
    fail '启用 Manus 时必须提供 --manus-rawviz 和 --manus-user'
fi

calibration_dir="$(normalize_existing_dir "$pico_calibration_dir")" ||
  fail "PICO calibration directory 不存在: $pico_calibration_dir"
[[ -w "$calibration_dir" ]] || fail "PICO calibration directory 不可写: $calibration_dir"
for artifact in \
  pico_left_arm_geometry.yaml pico_right_arm_geometry.yaml \
  pico_left_palm_tcp.yaml pico_right_palm_tcp.yaml \
  pico_left_wrist_pivot.yaml pico_right_wrist_pivot.yaml; do
  [[ -r "$calibration_dir/$artifact" ]] || fail "缺少 PICO calibration artifact: $calibration_dir/$artifact"
done

if [[ -n "$record_path" ]]; then
  [[ -x "$ROOT/build/hdf5_recorder/tianji_hdf5_recorder" ]] || fail "缺少 C++ HDF5 recorder；请先运行 pixi run build-hdf5-recorder"
  [[ ! -e "$record_path" && ! -L "$record_path" ]] || fail "拒绝覆盖已有 recording: $record_path"
  record_parent="$(dirname -- "$record_path")"
  [[ -d "$record_parent" ]] || fail "recording parent directory 不存在: $record_parent"
  record_path="$(cd -- "$record_parent" && pwd -P)/$(basename -- "$record_path")"
fi

downstream_args=(--profile vr_manus_sim)
[[ "$disable_hands" != true ]] || downstream_args+=(--disable-hands)
[[ -z "$display_mode" ]] || downstream_args+=("--$display_mode")
[[ -z "$record_path" ]] || downstream_args+=(--record "$record_path")
[[ "$spark_overlay" != true ]] || downstream_args+=(--spark-overlay)
downstream_args+=(--tjvr-bind "$tjvr_bind" --tjvr-port "$tjvr_port")
[[ -z "$duration_s" ]] || downstream_args+=(--duration-s "$duration_s")
[[ -z "$manus_rawviz" ]] || downstream_args+=(--manus-rawviz "$manus_rawviz")
[[ -z "$manus_user" ]] || downstream_args+=(--manus-user "$manus_user")
[[ -z "$manus_library_dir" ]] || downstream_args+=(--manus-library-dir "$manus_library_dir")
[[ -z "$right_glove" ]] || downstream_args+=(--right-glove "$right_glove")
[[ -z "$left_glove" ]] || downstream_args+=(--left-glove "$left_glove")
[[ -z "$ik_backend" ]] || downstream_args+=(--ik-backend "$ik_backend")
[[ -z "$arm_target_processor" ]] || downstream_args+=(--arm-target-processor "$arm_target_processor")
[[ -z "$joint_trajectory" ]] || downstream_args+=(--joint-trajectory "$joint_trajectory")
[[ -z "$command_step_clipping" ]] || downstream_args+=(--command-step-clipping "$command_step_clipping")
[[ -z "$joint_limit_source" ]] || downstream_args+=(--joint-limit-source "$joint_limit_source")

# Reuse the existing downstream asset/manus validation before touching adb or ROS.
python3 "$ROOT/scripts/vr_manus_live.py" --check "${downstream_args[@]}" >/dev/null

pixi_bin="${PIXI_BIN:-}"
if [[ -n "$pixi_bin" ]]; then
  [[ -x "$pixi_bin" ]] || fail "PIXI_BIN 不可执行: $pixi_bin"
else
  pixi_bin="$(command -v pixi || true)"
  [[ -n "$pixi_bin" ]] || fail '找不到 pixi'
fi
command -v adb >/dev/null 2>&1 || fail '找不到 adb'
adb_cmd=(adb)
[[ -z "$adb_serial" ]] || adb_cmd+=(-s "$adb_serial")

runtime_base="${TELEOP_RUNTIME_DIR}"
log_dir="$runtime_base/embedded-pico"
mkdir -p -- "$log_dir"

runtime_env=(
  "ROS_DOMAIN_ID=$pico_ros_domain"
  "EXO_REQUESTED_ROS_DOMAIN_ID=$pico_ros_domain"
  "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}"
  "ROS2CLI_DISABLE_DAEMON=${ROS2CLI_DISABLE_DAEMON:-1}"
  "PICO_TRACKER_CONFIG_DIR=$calibration_dir"
  "PICO_TRACKING_EPOCH_STATE_FILE=$calibration_dir/tracking_epoch"
  "ADB_SERIAL=$adb_serial"
)

run_preflight() {
  local mode="$1"
  env "${runtime_env[@]}" "$pixi_bin" run --manifest-path "$BUNDLE_ROOT/pixi.toml" \
    python3 "$ROOT/scripts/embedded_pico_preflight.py" \
    --mode "$mode" --timeout-s "$pico_startup_timeout_s" --domain "$pico_ros_domain"
}

declare -A child_pid=()
declare -A child_pgid=()
declare -A child_start_ticks=()
declare -A child_process_token=()
cleanup_done=false
forward_owned=false
existing_forward=false

process_group_for() {
  local pid="$1"
  ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' '
}

process_groups_for_token() {
  local token="$1"
  local process_dir
  local pid
  local pgid

  [[ -n "$token" ]] || return 0
  for process_dir in /proc/[0-9]*; do
    [[ -r "$process_dir/environ" ]] || continue
    /usr/bin/grep -Fzxq -- \
      "TIANJI_EMBEDDED_PICO_PROCESS_TOKEN=$token" \
      "$process_dir/environ" 2>/dev/null || continue
    pid="${process_dir##*/}"
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
    [[ "$pgid" =~ ^[0-9]+$ && "$pgid" -gt 1 ]] || continue
    printf '%s\n' "$pgid"
  done | sort -nu
}

process_group_is_running() {
  local pgid="$1"
  [[ "$pgid" =~ ^[0-9]+$ && "$pgid" -gt 1 ]] &&
    kill -0 -- "-$pgid" 2>/dev/null
}

start_child() {
  local label="$1"
  shift
  local log_path="$log_dir/${label}.log"
  local process_token="$embedded_process_run_token:$label"
  register_teleop_process_token "$process_token" || fail "无法登记 $label 进程标识"
  child_process_token["$label"]="$process_token"
  setsid env "${runtime_env[@]}" \
    "TIANJI_EMBEDDED_PICO_PROCESS_TOKEN=$process_token" \
    "$@" >"$log_path" 2>&1 &
  local pid=$!
  local pgid=""
  for _ in {1..20}; do
    pgid="$(process_group_for "$pid" || true)"
    [[ "$pgid" == "$pid" ]] && break
    sleep 0.01
  done
  [[ "$pgid" == "$pid" && "$pgid" -gt 1 ]] || {
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    fail "无法建立 $label 独立进程组"
  }
  child_pid["$label"]="$pid"
  child_pgid["$label"]="$pgid"
  child_start_ticks["$label"]="$(_process_start_ticks "$pid" || true)"
  if ! register_teleop_process_group "$pgid" "$label" 30; then
    kill -TERM -- "-$pgid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    fail "无法登记 $label 受管进程组"
  fi
}

stop_child() {
  local label="$1"
  local pid="${child_pid[$label]:-}"
  local pgid="${child_pgid[$label]:-}"
  local process_token="${child_process_token[$label]:-}"
  local group
  local groups_alive
  local -A process_groups=()

  [[ -n "$pid" ]] || return 0
  if [[ "$label" == downstream ]]; then
    # The downstream shell in turn notifies the Python owner. Do not signal
    # its recorder/IK/Manus children while that owner is draining them.
    graceful_stop_session_owner "$pid" 50 "${child_start_ticks[$label]:-}" || true
  fi
  if [[ "$pgid" =~ ^[0-9]+$ && "$pgid" -gt 1 ]]; then
    process_groups["$pgid"]=1
  fi

  while read -r group; do
    [[ "$group" =~ ^[0-9]+$ && "$group" -gt 1 ]] || continue
    process_groups["$group"]=1
  done < <(process_groups_for_token "$process_token")

  for group in "${!process_groups[@]}"; do
    kill -TERM -- "-$group" 2>/dev/null || true
  done

  for _ in {1..100}; do
    groups_alive=false
    while read -r group; do
      [[ "$group" =~ ^[0-9]+$ && "$group" -gt 1 ]] || continue
      process_groups["$group"]=1
    done < <(process_groups_for_token "$process_token")
    for group in "${!process_groups[@]}"; do
      if process_group_is_running "$group"; then
        groups_alive=true
        break
      fi
    done
    [[ "$groups_alive" == true ]] || break
    sleep 0.05
  done

  if [[ "$groups_alive" == true ]]; then
    for group in "${!process_groups[@]}"; do
      kill -KILL -- "-$group" 2>/dev/null || true
    done
  fi

  wait "$pid" 2>/dev/null || true
  child_pid["$label"]=""
  child_pgid["$label"]=""
  child_start_ticks["$label"]=""
  child_process_token["$label"]=""
}

cleanup() {
  local result=$?
  local guard_result=0
  if [[ "$cleanup_done" == true ]]; then
    return "$result"
  fi
  cleanup_done=true
  trap - EXIT
  trap '' INT TERM
  stop_child downstream
  stop_child bridge
  stop_child m0
  stop_child driver
  if [[ "$forward_owned" == true ]]; then
    "${adb_cmd[@]}" forward --remove tcp:9999 >/dev/null 2>&1 || true
  fi
  teleop_cleanup_and_release || guard_result=$?
  if ((guard_result != 0)); then
    return "$guard_result"
  fi
  return "$result"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# The shared device-route guard is authoritative. Keep the historical path as
# a marker for operators and older tooling, but do not hold a second kernel
# flock whose descriptor could be inherited by an orphaned child.
embedded_lock_path="$runtime_base/embedded-pico.lock"
: >"$embedded_lock_path"
command -v flock >/dev/null 2>&1 || fail '系统缺少 flock，无法检查旧版内嵌 PICO 会话'
if ! flock -n "$embedded_lock_path" -c true; then
  fail "已有旧版内嵌 PICO 会话正在运行：$embedded_lock_path"
fi
if ! acquire_teleop_guard pico_vr_manus_sim \
  "${TELEOP_DEVICE_ROUTE_MODES[@]}"; then
  if [[ "${TELEOP_GUARD_DIR}" == "${TELEOP_GUARDS_DIR}/pico_vr_manus_sim" ]]; then
    printf '%s\n' '已有内嵌 PICO 会话正在运行；请先停止原会话后再启动。' >&2
  fi
  exit 2
fi
embedded_process_run_token="${BASHPID}_${RANDOM}_$(date +%s%N)"

adb_state="$("${adb_cmd[@]}" get-state 2>/dev/null || true)"
[[ "$adb_state" == device ]] || fail '未检测到已授权的 PICO adb 设备；请连接头显并确认 adb devices 为 device'
"${adb_cmd[@]}" shell monkey -p "$pico_app_package" 1 >/dev/null 2>&1 || fail "无法启动 PICO APK: $pico_app_package"
forward_listing="$("${adb_cmd[@]}" forward --list 2>/dev/null || true)"
while read -r _serial local_endpoint remote_endpoint _rest; do
  [[ -n "${local_endpoint:-}" ]] || continue
  [[ -z "$adb_serial" || "$_serial" == "$adb_serial" ]] || continue
  if [[ "$local_endpoint" == tcp:9999 ]]; then
    if [[ "$remote_endpoint" == tcp:9999 ]]; then
      existing_forward=true
    else
      fail "adb tcp:9999 已被映射到非 tcp:9999 目标，拒绝覆盖"
    fi
  fi
done <<< "$forward_listing"
"${adb_cmd[@]}" forward tcp:9999 tcp:9999 >/dev/null
[[ "$existing_forward" == true ]] || forward_owned=true

start_child driver "$pixi_bin" run --manifest-path "$BUNDLE_ROOT/pixi.toml" \
  bash "$BUNDLE_ROOT/scripts/start_pico_driver.sh"
if ! run_preflight raw; then
  fail 'PICO driver raw ROS stream readiness failed'
fi

m0_args=(bash "$BUNDLE_ROOT/scripts/start_pico_m0.sh")
[[ "$display_mode" != viewer ]] || m0_args+=(--viewer)
start_child m0 "$pixi_bin" run --manifest-path "$BUNDLE_ROOT/pixi.toml" "${m0_args[@]}"
if ! run_preflight m0; then
  fail 'PICO M0 corrected skeleton readiness failed'
fi

start_child bridge "$pixi_bin" run --manifest-path "$BUNDLE_ROOT/pixi.toml" \
  bash -c \
  'bundle="$1"; bind="$2"; port="$3"; source "$bundle/install/local_setup.bash"; exec ros2 launch pico_bridge start_tianji_mujoco_teleop.launch.py "destination_address:=$bind" "destination_port:=$port" position_retargeting_mode:=robot_arm_segments robot_arm_reach_scale:=0.95' \
  _ "$BUNDLE_ROOT" "$tjvr_bind" "$tjvr_port"
if ! run_preflight bridge; then
  fail 'TJVR bridge readiness failed'
fi

downstream_script="${TIANJI_EMBEDDED_PICO_DOWNSTREAM_SCRIPT:-$ROOT/scripts/run_vr_manus_session.sh}"
[[ -f "$downstream_script" ]] || fail "缺少下游启动脚本: $downstream_script"
start_child downstream env TIANJI_EMBEDDED_PICO_DOWNSTREAM=1 \
  bash "$downstream_script" "${downstream_args[@]}"

printf 'session pico_vr_manus_sim started; embedded_pico_bundle=%s; tjvr=%s:%s\n' \
  "$BUNDLE_ROOT" "$tjvr_bind" "$tjvr_port"
wait "${child_pid[downstream]}"
