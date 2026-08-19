#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

CONDA_PREFIX="${CONDA_PREFIX:-}"
if [[ -z "${CONDA_PREFIX}" || ! -x "${CONDA_PREFIX}/bin/python" ]]; then
  printf '%s\n' '错误：请通过 pixi run -e ik-build 运行本脚本。' >&2
  exit 1
fi

# robostack 的 setup.bash 不设置 PKG_CONFIG_PATH；
# ROS 的 libyaml_vendor 等通过 pkg-config 找依赖，必须显式补上。
export PKG_CONFIG_PATH="${CONDA_PREFIX}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"

set +u
# shellcheck disable=SC1091
source "${CONDA_PREFIX}/setup.bash"
set -u

export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

cd "${BUNDLE_ROOT}"
"${CONDA_PREFIX}/bin/colcon" --log-base log/ik build \
  --base-paths src \
  --packages-select pico_body_tianji \
  --build-base build/ik \
  --install-base staging/ik \
  --merge-install \
  --cmake-force-configure \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_C_COMPILER=/usr/bin/gcc \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
    -DPython3_EXECUTABLE="$(command -v python)"

ik_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/tianji_kinematic_sim"
probe_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/tianji_official_ik_probe"
worker_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/tianji_official_ik_worker"
qp_probe_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/pinocchio_qp_ik_probe"
for binary in \
  "${ik_binary}" \
  "${probe_binary}" \
  "${worker_binary}" \
  "${qp_probe_binary}"
do
  if [[ ! -x "${binary}" ]]; then
    printf '错误：编译产物不存在：%s\n' "${binary}" >&2
    exit 1
  fi
  if [[ "$(file -b "${binary}")" != *"ELF 64-bit"* ]]; then
    printf '错误：编译产物不是 64 位 ELF：%s\n' "${binary}" >&2
    exit 1
  fi
done

# 便携层 ABI 检查：新二进制（系统 GCC/glibc 编译）必须能被
# runtime/abi 的 glibc 2.35 + libstdc++ 加载。
runtime_loader="${BUNDLE_ROOT}/runtime/abi/lib/ld-linux-x86-64.so.2"
runtime_libc="${BUNDLE_ROOT}/runtime/abi/lib/libc.so.6"
runtime_cpp="${BUNDLE_ROOT}/runtime/abi/lib/libstdc++.so.6"
runtime_glibc_max="$(
  objdump -T "${runtime_libc}" |
    grep -v '\*UND\*' |
    grep -oE 'GLIBC_[0-9.]+' |
    sort -V |
    tail -n 1
)"
runtime_glibcxx_max="$(
  objdump -T "${runtime_cpp}" |
    grep -v '\*UND\*' |
    grep -oE 'GLIBCXX_[0-9.]+' |
    sort -V |
    tail -n 1
)"
for binary in "${ik_binary}" "${probe_binary}" "${worker_binary}"; do
  needed_glibc="$(
    objdump -T "${binary}" |
      grep '\*UND\*' |
      grep -oE 'GLIBC_[0-9.]+' |
      sort -V |
      tail -n 1
  )"
  needed_glibcxx="$(
    objdump -T "${binary}" |
      grep '\*UND\*' |
      grep -oE 'GLIBCXX_[0-9.]+' |
      sort -V |
      tail -n 1
  )"
  if [[ -n "${needed_glibc}" ]] &&
     [[ "$(printf '%s\n%s\n' "${needed_glibc}" "${runtime_glibc_max}" | sort -V | tail -n 1)" != "${runtime_glibc_max}" ]]
  then
    printf '错误：%s 需要 GLIBC %s，超过 runtime/abi 提供的 %s。\n' \
      "$(basename "${binary}")" "${needed_glibc}" "${runtime_glibc_max}" >&2
    printf '  便携层无法在旧目标机加载；请检查链接参数。\n' >&2
    exit 1
  fi
  if [[ -n "${needed_glibcxx}" ]] &&
     [[ "$(printf '%s\n%s\n' "${needed_glibcxx}" "${runtime_glibcxx_max}" | sort -V | tail -n 1)" != "${runtime_glibcxx_max}" ]]
  then
    printf '错误：%s 需要 GLIBCXX %s，超过 runtime/abi 提供的 %s。\n' \
      "$(basename "${binary}")" "${needed_glibcxx}" "${runtime_glibcxx_max}" >&2
    exit 1
  fi
  printf 'ABI 检查通过：%s（GLIBC %s ≤ %s，GLIBCXX %s ≤ %s）\n' \
    "$(basename "${binary}")" \
    "${needed_glibc:-none}" "${runtime_glibc_max}" \
    "${needed_glibcxx:-none}" "${runtime_glibcxx_max}"
done

if ! nm -C "${ik_binary}" | grep -F 'TianjiOfficialArmIk' >/dev/null; then
  printf '%s\n' '错误：新 IK 二进制不包含天机官方 IK 实现。' >&2
  exit 1
fi
if ! nm -C "${ik_binary}" | grep -F 'create_arm_ik_solver' >/dev/null; then
  printf '%s\n' '错误：新 IK 二进制不包含 IK factory。' >&2
  exit 1
fi
if ! nm -C "${ik_binary}" | grep -F 'PinocchioQpArmIk' >/dev/null; then
  printf '%s\n' '错误：新 IK 二进制不包含 Pinocchio QP IK 实现。' >&2
  exit 1
fi

printf 'IK 编译完成：%s\n' "${ik_binary}"
printf '官方 IK probe：%s\n' "${probe_binary}"
printf '官方 IK worker：%s\n' "${worker_binary}"
printf 'Pinocchio QP probe：%s\n' "${qp_probe_binary}"
