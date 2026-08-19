#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  printf '%s\n' '错误：IK 只能在 Linux x86_64 构建。' >&2
  exit 1
fi
if [[ ! -r /etc/os-release ]] ||
   ! grep -Eq 'VERSION_ID="(22\.04|24\.04)"' /etc/os-release; then
  printf '%s\n' '错误：IK 构建环境必须是 Ubuntu 22.04 或 24.04。' >&2
  exit 1
fi

# 编译器：系统默认 GCC（22.04 为 GCC 11，24.04 为 GCC 13）。
# 新二进制经 runtime/abi 便携层部署，GLIBCXX 需求必须被
# runtime/abi/lib/libstdc++.so.6 覆盖（见下方 ABI 检查）。
for executable in /usr/bin/gcc /usr/bin/g++; do
  if [[ ! -x "${executable}" ]]; then
    printf '错误：缺少编译器：%s\n' "${executable}" >&2
    exit 1
  fi
done
cc_version="$(/usr/bin/gcc -dumpfullversion -dumpversion)"
cc_major="${cc_version%%.*}"
if (( cc_major < 11 )); then
  printf '错误：要求 GCC >= 11，当前为 %s。\n' "${cc_version}" >&2
  exit 1
fi
if (( cc_major > 13 )); then
  printf '错误：GCC %s 的 GLIBCXX 版本超过 runtime/abi 便携层（GCC 13）；\n' \
    "${cc_version}" >&2
  printf '  请改用 gcc-13（update-alternatives 或环境变量）。\n' >&2
  exit 1
fi

# ROS Humble 开发环境来自 pixi ik-build（robostack-staging），
# 必须通过 pixi run -e ik-build 执行本脚本。
CONDA_PREFIX="${CONDA_PREFIX:-}"
if [[ -z "${CONDA_PREFIX}" || ! -x "${CONDA_PREFIX}/bin/python" ]]; then
  printf '%s\n' '错误：请通过 pixi run -e ik-build 运行本脚本。' >&2
  exit 1
fi
ROS_SETUP="${CONDA_PREFIX}/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  printf '错误：缺少 pixi ROS Humble 环境：%s\n' "${ROS_SETUP}" >&2
  exit 1
fi
for executable in "${CONDA_PREFIX}/bin/colcon" "${CONDA_PREFIX}/bin/cmake"; do
  if [[ ! -x "${executable}" ]]; then
    printf '错误：缺少构建工具：%s\n' "${executable}" >&2
    exit 1
  fi
done

# robostack 的 setup.bash 不设置 PKG_CONFIG_PATH；
# ROS 的 libyaml_vendor 等通过 pkg-config 找依赖，必须显式补上。
export PKG_CONFIG_PATH="${CONDA_PREFIX}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"

# ROS 生成的 setup.bash 会读取若干可能未定义的跟踪变量。
set +u
# shellcheck disable=SC1091
source "${ROS_SETUP}"
set -u
for package in \
  ament_cmake \
  ament_cmake_python \
  ament_index_cpp \
  geometry_msgs \
  rclcpp \
  sensor_msgs \
  std_msgs \
  tf2_ros \
  visualization_msgs
do
  if [[ ! -f "${CONDA_PREFIX}/share/${package}/package.xml" ]]; then
    printf '错误：缺少 ROS Humble 开发包：%s\n' "${package}" >&2
    exit 1
  fi
done

rclcpp_version="$({
  sed -n 's/.*Found rclcpp: \([^ ]*\).*/\1/p' \
    "${CONDA_PREFIX}/share/rclcpp/cmake/rclcppConfig.cmake"
} | head -n 1)"
if [[ "${rclcpp_version}" != 16.0.19 ]]; then
  printf '错误：要求 rclcpp 16.0.19，当前为 %s。\n' \
    "${rclcpp_version:-unknown}" >&2
  exit 1
fi

python - <<'PY'
from importlib.metadata import distribution
from pathlib import Path

pin = distribution("pin")
if pin.version != "4.0.0":
    raise SystemExit(f"错误：要求 pin 4.0.0，当前为 {pin.version}")
prefix = pin.locate_file("cmeel.prefix").resolve()
required = (
    prefix / "include" / "pinocchio",
    prefix / "lib" / "cmake" / "pinocchio" / "pinocchioConfig.cmake",
    prefix / "lib" / "libpinocchio_default.so",
    prefix / "lib" / "libpinocchio_parsers.so",
)
missing = [str(path) for path in required if not path.exists()]
if missing:
    raise SystemExit("错误：pin wheel 缺少开发文件：" + ", ".join(missing))
print("Pinocchio 构建前缀：", prefix)
PY

pin_prefix="$(python - <<'PY'
from importlib.metadata import distribution
print(distribution("pin").locate_file("cmeel.prefix").resolve())
PY
)"
for library in libpinocchio_default.so libpinocchio_parsers.so; do
  if ! cmp -s \
    "${pin_prefix}/lib/${library}" \
    "${BUNDLE_ROOT}/runtime/pin/lib/${library}"
  then
    printf '错误：Pixi %s 与 runtime/pin 的 ABI 不一致。\n' \
      "${library}" >&2
    exit 1
  fi
done

runtime_loader="${BUNDLE_ROOT}/runtime/abi/lib/ld-linux-x86-64.so.2"
runtime_cpp="${BUNDLE_ROOT}/runtime/abi/lib/libstdc++.so.6"
if [[ ! -x "${runtime_loader}" || ! -f "${runtime_cpp}" ]]; then
  printf '%s\n' '错误：bundled ABI runtime 不完整。' >&2
  exit 1
fi

# GCC 编译的二进制经 runtime/abi 便携层部署；该层 libstdc++ 至少提供
# GLIBCXX_3.4.29（GCC 11）。精确的 GLIBC/GLIBCXX 需求约束由
# build_ik.sh 在编译后按产物实测检查。
runtime_cpp_max="$(
  objdump -T "${runtime_cpp}" |
    grep -v '\*UND\*' |
    grep -oE 'GLIBCXX_[0-9.]+' |
    sort -V |
    tail -n 1
)"
if [[ "$(printf '%s\n%s\n' "3.4.29" "${runtime_cpp_max}" | sort -V | tail -n 1)" != "${runtime_cpp_max}" ]]; then
  printf '错误：runtime/abi libstdc++（GLIBCXX %s）低于便携层最小需求 3.4.29。\n' \
    "${runtime_cpp_max}" >&2
  exit 1
fi

printf '%s\n' \
  'IK 构建环境检查通过：Ubuntu 24.04 + GCC 13 + pixi ROS Humble 16.0.19 + Pinocchio 4.0.0。'
