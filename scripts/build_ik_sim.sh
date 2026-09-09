#!/usr/bin/env bash
# Run in `pixi run -e ik-build`: incremental, host-native simulation build.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM_BUILD_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cmake -S "${SIM_BUILD_ROOT}/src/tianji_teleop" -B "${SIM_BUILD_ROOT}/build/ik-sim" \
  -DTIANJI_SIMULATION_ONLY=ON \
  -DTIANJI_ENABLE_V131=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DPython3_EXECUTABLE="$(command -v python)" \
  -DCMAKE_INSTALL_PREFIX="${SIM_BUILD_ROOT}/staging/ik"
cmake --build "${SIM_BUILD_ROOT}/build/ik-sim" --parallel "${TIANJI_BUILD_JOBS:-2}"
cmake --install "${SIM_BUILD_ROOT}/build/ik-sim"
"${SIM_BUILD_ROOT}/staging/ik/lib/tianji_teleop/v131_qp_probe"
"${SIM_BUILD_ROOT}/staging/ik/lib/tianji_teleop/dexhand_qp_core_probe"
"${SIM_BUILD_ROOT}/staging/ik/lib/tianji_teleop/dexhand_qp_arm_probe" \
  "${SIM_BUILD_ROOT}/src/tianji_teleop/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
"${SIM_BUILD_ROOT}/staging/ik/lib/tianji_teleop/pinocchio_qp_ik_probe" \
  "${SIM_BUILD_ROOT}/src/tianji_teleop/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
