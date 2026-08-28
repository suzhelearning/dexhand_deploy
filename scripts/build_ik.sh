#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

CONDA_PREFIX="${CONDA_PREFIX:-}"
if [[ -z "${CONDA_PREFIX}" || ! -x "${CONDA_PREFIX}/bin/python" ]]; then
  printf '%s\n' '错误：请在 pixi ik-build 环境中运行（pixi run -e ik-build build-ik）。' >&2
  exit 1
fi

export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

cd "${BUNDLE_ROOT}"
rm -rf build/ik
cmake -S src/pico_body_tianji -B build/ik \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DPython3_EXECUTABLE="$(command -v python)" \
  -DCMAKE_INSTALL_PREFIX="${BUNDLE_ROOT}/staging/ik"
cmake --build build/ik --parallel "$(nproc)"
cmake --install build/ik

ik_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/tianji_kinematic_sim"
probe_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/tianji_official_ik_probe"
worker_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/tianji_official_ik_worker"
qp_probe_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/pinocchio_qp_ik_probe"
wuji_bridge_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/wuji_hand2_bridge"
for binary in \
  "${ik_binary}" \
  "${probe_binary}" \
  "${worker_binary}" \
  "${qp_probe_binary}" \
  "${wuji_bridge_binary}"
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
for binary in \
  "${ik_binary}" \
  "${probe_binary}" \
  "${worker_binary}" \
  "${wuji_bridge_binary}"; do
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

printf 'wuji2 手桥：%s\n' "${wuji_bridge_binary}"
