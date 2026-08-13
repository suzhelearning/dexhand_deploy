#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

export CC=/usr/bin/gcc-11
export CXX=/usr/bin/g++-11

cd "${BUNDLE_ROOT}"
/usr/bin/colcon --log-base log/ik build \
  --base-paths src \
  --packages-select pico_body_tianji \
  --build-base build/ik \
  --install-base staging/ik \
  --merge-install \
  --cmake-force-configure \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_C_COMPILER=/usr/bin/gcc-11 \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 \
    -DPython3_EXECUTABLE="$(command -v python)"

ik_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/tianji_kinematic_sim"
probe_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/tianji_official_ik_probe"
worker_binary="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/tianji_official_ik_worker"
for binary in "${ik_binary}" "${probe_binary}" "${worker_binary}"; do
  if [[ ! -x "${binary}" ]]; then
    printf '错误：编译产物不存在：%s\n' "${binary}" >&2
    exit 1
  fi
  if [[ "$(file -b "${binary}")" != *"ELF 64-bit"* ]]; then
    printf '错误：编译产物不是 64 位 ELF：%s\n' "${binary}" >&2
    exit 1
  fi
done

if ! nm -C "${ik_binary}" | grep -F 'TianjiOfficialArmIk' >/dev/null; then
  printf '%s\n' '错误：新 IK 二进制不包含天机官方 IK 实现。' >&2
  exit 1
fi
if ! nm -C "${ik_binary}" | grep -F 'create_arm_ik_solver' >/dev/null; then
  printf '%s\n' '错误：新 IK 二进制不包含 IK factory。' >&2
  exit 1
fi

printf 'IK 编译完成：%s\n' "${ik_binary}"
printf '官方 IK probe：%s\n' "${probe_binary}"
printf '官方 IK worker：%s\n' "${worker_binary}"
