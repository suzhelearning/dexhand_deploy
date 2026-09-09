cmake_minimum_required(VERSION 3.16)
project(tianji_teleop)
option(TIANJI_SIMULATION_ONLY "Build local Pinocchio IK without physical-device SDKs" OFF)
option(TIANJI_ENABLE_V131 "Enable the host-native v131 simulation backend" OFF)
if(TIANJI_ENABLE_V131 AND NOT TIANJI_SIMULATION_ONLY)
  message(FATAL_ERROR "v131 requires TIANJI_SIMULATION_ONLY; its host-native dependencies are not a portable runtime bundle")
endif()

# Zenoh（zenoh-c + zenoh-cpp）vendored 于仓库根 vendor/ 下：
#   vendor/zenoh/include        zenoh-c 头文件
#   vendor/zenoh/lib            libzenohc.so
#   vendor/zenoh-cpp/include    zenoh-cpp（header-only）
# wuji-sdk（C SDK v2026.8.17）vendored 于 vendor/wuji-sdk：
#   vendor/wuji-sdk/include/wuji_sdk.h
#   vendor/wuji-sdk/lib/libwuji_sdk_c.so
get_filename_component(
  ZENOH_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/zenoh"
  REALPATH
)
get_filename_component(
  ZENOH_CPP_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/zenoh-cpp"
  REALPATH
)
get_filename_component(
  WUJI_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/wuji-sdk"
  REALPATH
)
if(NOT EXISTS "${ZENOH_ROOT}/lib/libzenohc.so")
  message(FATAL_ERROR "缺少 vendor/zenoh/lib/libzenohc.so（zenoh-c 1.10.0 standalone 包）")
endif()
if(NOT EXISTS "${ZENOH_CPP_ROOT}/include/zenoh.hxx")
  message(FATAL_ERROR "缺少 vendor/zenoh-cpp/include（zenoh-cpp 1.10.0 头文件）")
endif()
if(NOT TIANJI_SIMULATION_ONLY AND NOT EXISTS "${WUJI_SDK_ROOT}/lib/libwuji_sdk_c.so")
  message(FATAL_ERROR "缺少 vendor/wuji-sdk/lib/libwuji_sdk_c.so（wuji-sdk C v2026.8.17）")
endif()

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# 与 TJ_arm_control 保持同一版本；静态链接，部署时无需额外共享库。
include(FetchContent)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  ruckig
  GIT_REPOSITORY https://github.com/pantor/ruckig.git
  GIT_TAG v0.19.4
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(ruckig)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(
  COMMAND
    "${Python3_EXECUTABLE}" -c
    "from importlib.metadata import distribution; print(distribution('pin').locate_file('cmeel.prefix').resolve())"
  RESULT_VARIABLE PINOCCHIO_PREFIX_RESULT
  OUTPUT_VARIABLE PINOCCHIO_CMEEL_PREFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT PINOCCHIO_PREFIX_RESULT EQUAL 0)
  message(FATAL_ERROR "无法从 Pixi 的 pin wheel 定位 C++ CMake 前缀")
endif()
list(PREPEND CMAKE_PREFIX_PATH "${PINOCCHIO_CMEEL_PREFIX}")
find_package(pinocchio 4.0 REQUIRED CONFIG)
if(TIANJI_ENABLE_V131)
  find_path(V131_QPOASES_INCLUDE qpOASES.hpp HINTS "$ENV{CONDA_PREFIX}/include" REQUIRED)
  find_library(V131_QPOASES_LIBRARY qpOASES HINTS "$ENV{CONDA_PREFIX}/lib" REQUIRED)
  find_path(V131_MUJOCO_INCLUDE mujoco/mujoco.h HINTS "$ENV{CONDA_PREFIX}/include" REQUIRED)
  find_library(V131_MUJOCO_LIBRARY mujoco HINTS "$ENV{CONDA_PREFIX}/lib" REQUIRED)
  # MuJoCo's conda build uses newer GLIBCXX symbols than the host GCC 11
  # driver. Link the matching conda runtime explicitly, not its system default.
  find_library(V131_CXX_RUNTIME stdc++ HINTS "$ENV{CONDA_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)
endif()

get_filename_component(
  PINOCCHIO_CMEEL_PREFIX_REAL
  "${PINOCCHIO_CMEEL_PREFIX}"
  REALPATH
)
set(CMAKE_BUILD_RPATH "${PINOCCHIO_CMEEL_PREFIX_REAL}/lib")
set(CMAKE_INSTALL_RPATH "${PINOCCHIO_CMEEL_PREFIX_REAL}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_library(
  arm_ik
  src/control/joint_trajectory_limiter.cpp
  src/ik/arm_ik_factory.cpp
  src/ik/pinocchio_cpp/pinocchio_arm_ik.cpp
  src/ik/pinocchio_qp/pinocchio_qp_arm_ik.cpp
  src/ik/tianji_official/tianji_official_arm_ik_proxy.cpp
)
if(TIANJI_ENABLE_V131)
  target_sources(arm_ik PRIVATE
  src/ik/dexhand_qp/arm_ik.cpp
  src/ik/dexhand_qp/dexhand_velocity_qp_ik.cpp
  src/ik/dexhand_qp/dexhand_velocity_ruckig.cpp
  src/ik/dexhand_qp/so3.cpp
  src/ik/v131_qp/hierarchical_qp.cpp
  src/ik/v131_qp/hierarchical_qpoases_solver.cpp
  src/ik/v131_qp/velocity_ik.cpp
  src/ik/v131_qp/full_joint_temporal_envelope.cpp
  src/ik/v131_qp/pico_ee_v131_singularity.cpp
  src/ik/v131_qp/pico_ee_v131_jacobian_svd.cpp
  src/ik/v131_qp/so3.cpp
  src/ik/v131_qp/velocity_qp.cpp
  src/ik/v131_qp/mujoco_robot.cpp
  src/ik/v131_qp/cartesian_otg.cpp
  src/ik/v131_qp/cartesian_servo.cpp
  src/ik/v131_qp/target_manager.cpp
  src/ik/v131_qp/dexhand_target_conditioner.cpp
)
  target_compile_definitions(arm_ik PRIVATE TIANJI_ENABLE_V131=1)
  target_include_directories(arm_ik PUBLIC ${V131_QPOASES_INCLUDE} ${V131_MUJOCO_INCLUDE})
  target_link_libraries(arm_ik PUBLIC ${V131_QPOASES_LIBRARY} ${V131_MUJOCO_LIBRARY} ${V131_CXX_RUNTIME})
endif()
target_compile_features(arm_ik PUBLIC cxx_std_17)
target_compile_options(
  arm_ik PRIVATE -Wall -Wextra -Wpedantic
)
target_include_directories(
  arm_ik
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  arm_ik PUBLIC Eigen3::Eigen pinocchio::pinocchio ruckig::ruckig ${CMAKE_DL_LIBS}
)

# Local simulation uses the same solver/factory and producer as deployment.
# No Wuji device SDK or portable runtime/ABI bundle is needed on this path.
if(TIANJI_SIMULATION_ONLY)
  if(TIANJI_ENABLE_V131)
    add_executable(dexhand_qp_arm_probe src/ik/dexhand_qp/arm_probe.cpp)
    target_link_libraries(dexhand_qp_arm_probe PRIVATE arm_ik)
    add_executable(dexhand_qp_core_probe src/ik/dexhand_qp/core_probe.cpp)
    add_executable(v131_qp_probe src/ik/v131_qp/probe.cpp)
    target_link_libraries(v131_qp_probe PRIVATE arm_ik)
    add_executable(v131_model_trace src/ik/v131_qp/model_trace.cpp)
    target_link_libraries(v131_model_trace PRIVATE arm_ik)
    target_link_libraries(dexhand_qp_core_probe PRIVATE arm_ik)
    install(TARGETS dexhand_qp_core_probe dexhand_qp_arm_probe v131_qp_probe
      RUNTIME DESTINATION lib/${PROJECT_NAME})
  endif()
  add_executable(arm_ik_producer src/producers/arm_ik_producer_node.cpp)
  target_compile_features(arm_ik_producer PRIVATE cxx_std_17)
  target_compile_definitions(arm_ik_producer PRIVATE ZENOHCXX_ZENOHC)
  target_include_directories(arm_ik_producer PRIVATE
    "${ZENOH_CPP_ROOT}/include" "${ZENOH_ROOT}/include")
  target_link_libraries(arm_ik_producer PRIVATE arm_ik "${ZENOH_ROOT}/lib/libzenohc.so")
  add_executable(pinocchio_qp_ik_probe src/pinocchio_qp/pinocchio_qp_ik_probe.cpp)
  target_link_libraries(pinocchio_qp_ik_probe PRIVATE arm_ik)
  add_executable(joint_trajectory_limiter_probe src/control/joint_trajectory_limiter_probe.cpp)
  target_link_libraries(joint_trajectory_limiter_probe PRIVATE arm_ik)
  install(TARGETS arm_ik_producer pinocchio_qp_ik_probe joint_trajectory_limiter_probe
    RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(
    DIRECTORY tianji_world_output
    DESTINATION lib/python3.10/site-packages
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
  )
  return()
endif()

# Marvin SDK I/O stays in one dedicated 200 Hz native loop.  The Python
# executor remains the protocol/safety boundary and calls this small C ABI.
add_library(marvin_native_driver SHARED src/marvin/native_driver.cpp)
target_compile_features(marvin_native_driver PUBLIC cxx_std_17)
target_compile_options(marvin_native_driver PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  marvin_native_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(marvin_native_driver PRIVATE Threads::Threads ${CMAKE_DL_LIBS})

add_library(marvin_native_fake_sdk SHARED src/marvin/native_driver_fake_sdk.cpp)
target_compile_features(marvin_native_fake_sdk PUBLIC cxx_std_17)
target_compile_options(marvin_native_fake_sdk PRIVATE -Wall -Wextra -Wpedantic)

add_executable(marvin_native_driver_probe src/marvin/native_driver_probe.cpp)
target_compile_features(marvin_native_driver_probe PUBLIC cxx_std_17)
target_compile_options(marvin_native_driver_probe PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(marvin_native_driver_probe PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(marvin_native_driver_probe PRIVATE marvin_native_driver Threads::Threads)

# 厂商 libKine 与 Pinocchio 的依赖闭包不能安全共存。native 库只链接
# Eigen/dl，供独立 worker 和 probe 使用；主节点内使用 IPC proxy。
add_library(
  tianji_official_native STATIC
  src/ik/tianji_official/tianji_official_arm_ik.cpp
  src/tianji_official/glibc_isoc23_strtol_shim.cpp
)
target_compile_features(tianji_official_native PUBLIC cxx_std_17)
target_compile_options(tianji_official_native PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  tianji_official_native PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  tianji_official_native PUBLIC Eigen3::Eigen ${CMAKE_DL_LIBS}
)

# 该节点只做所选后端的 IK/FK，不直接连接机械臂；通讯走 Zenoh。
add_executable(
  arm_ik_producer src/producers/arm_ik_producer_node.cpp
)

add_executable(
  protocol_cpp_fixture
  "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/cpp_protocol_fixture.cpp"
)
target_compile_features(protocol_cpp_fixture PUBLIC cxx_std_17)
target_compile_options(protocol_cpp_fixture PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  protocol_cpp_fixture PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
add_executable(
  tianji_official_ik_probe
  src/tianji_official/tianji_official_ik_probe.cpp
)
add_executable(
  tianji_official_ik_worker
  src/tianji_official/tianji_official_ik_worker.cpp
)
add_executable(
  pinocchio_qp_ik_probe
  src/pinocchio_qp/pinocchio_qp_ik_probe.cpp
)
add_executable(
  joint_trajectory_limiter_probe
  src/control/joint_trajectory_limiter_probe.cpp
)
add_executable(
  wuji_hand2_interpolation_probe
  src/wuji_hand2/wuji_hand2_interpolation_probe.cpp
)
add_executable(
  wuji_hand2_control_probe
  src/wuji_hand2/wuji_hand2_control_probe.cpp
)
target_compile_features(arm_ik_producer PUBLIC cxx_std_17)
target_compile_features(tianji_official_ik_probe PUBLIC cxx_std_17)
target_compile_features(tianji_official_ik_worker PUBLIC cxx_std_17)
target_compile_features(pinocchio_qp_ik_probe PUBLIC cxx_std_17)
target_compile_features(joint_trajectory_limiter_probe PUBLIC cxx_std_17)
target_compile_features(wuji_hand2_interpolation_probe PUBLIC cxx_std_17)
target_compile_features(wuji_hand2_control_probe PUBLIC cxx_std_17)
target_compile_options(
  arm_ik_producer PRIVATE -Wall -Wextra -Wpedantic
)
target_compile_options(
  tianji_official_ik_probe PRIVATE -Wall -Wextra -Wpedantic
)
target_compile_options(
  tianji_official_ik_worker PRIVATE -Wall -Wextra -Wpedantic
)
target_compile_options(
  pinocchio_qp_ik_probe PRIVATE -Wall -Wextra -Wpedantic
)
target_compile_options(
  joint_trajectory_limiter_probe PRIVATE -Wall -Wextra -Wpedantic
)
target_compile_options(
  wuji_hand2_interpolation_probe PRIVATE -Wall -Wextra -Wpedantic
)
target_compile_options(
  wuji_hand2_control_probe PRIVATE -Wall -Wextra -Wpedantic
)
target_include_directories(
  wuji_hand2_interpolation_probe PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_include_directories(
  wuji_hand2_control_probe PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${WUJI_SDK_ROOT}/include"
)
target_include_directories(
  arm_ik_producer PRIVATE
    "${ZENOH_CPP_ROOT}/include"
    "${ZENOH_ROOT}/include"
)
target_compile_definitions(arm_ik_producer PRIVATE ZENOHCXX_ZENOHC)
target_link_libraries(arm_ik_producer arm_ik "${ZENOH_ROOT}/lib/libzenohc.so")
target_link_libraries(tianji_official_ik_probe tianji_official_native)
target_link_libraries(tianji_official_ik_worker tianji_official_native)
target_link_libraries(pinocchio_qp_ik_probe arm_ik)
target_link_libraries(joint_trajectory_limiter_probe arm_ik)
target_link_libraries(
  wuji_hand2_control_probe PRIVATE
  "${WUJI_SDK_ROOT}/lib/libwuji_sdk_c.so"
  ${CMAKE_DL_LIBS}
  Threads::Threads
)
# 便携 runtime 层为 glibc 2.35：把 glibc 2.38 的 __isoc23_strtol
# 引用改接到 shim（见 glibc_isoc23_strtol_shim.cpp）。
target_sources(arm_ik_producer PRIVATE src/tianji_official/glibc_isoc23_strtol_shim.cpp)
target_link_libraries(arm_ik_producer arm_ik "${ZENOH_ROOT}/lib/libzenohc.so")
target_link_options(
  arm_ik_producer PRIVATE "-Wl,--wrap=__isoc23_strtol"
)
target_link_options(
  tianji_official_ik_probe PRIVATE "-Wl,--wrap=__isoc23_strtol"
)
target_link_options(
  tianji_official_ik_worker PRIVATE "-Wl,--wrap=__isoc23_strtol"
)
set_target_properties(
  tianji_official_ik_probe
  tianji_official_ik_worker
  PROPERTIES
    SKIP_BUILD_RPATH TRUE
    INSTALL_RPATH ""
)

# wuji-sdk 桥：只链接 zenoh-c 与 wuji-sdk C 库，不依赖 Pinocchio。
add_library(
  wuji_hand2
  src/wuji_hand2/wuji_hand2_control.cpp
)
target_compile_features(wuji_hand2 PUBLIC cxx_std_17)
target_compile_options(wuji_hand2 PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  wuji_hand2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    "${WUJI_SDK_ROOT}/include"
)

add_executable(
  wuji_hand2_bridge src/wuji_hand2/wuji_hand2_bridge_node.cpp
)
target_compile_features(wuji_hand2_bridge PUBLIC cxx_std_17)
target_compile_options(wuji_hand2_bridge PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
  wuji_hand2_bridge PRIVATE
    "${ZENOH_CPP_ROOT}/include"
    "${ZENOH_ROOT}/include"
    "${WUJI_SDK_ROOT}/include"
)
target_compile_definitions(wuji_hand2_bridge PRIVATE ZENOHCXX_ZENOHC)
target_sources(wuji_hand2_bridge PRIVATE src/tianji_official/glibc_isoc23_strtol_shim.cpp)
target_link_libraries(
  wuji_hand2_bridge
  wuji_hand2
  "${ZENOH_ROOT}/lib/libzenohc.so"
  "${WUJI_SDK_ROOT}/lib/libwuji_sdk_c.so"
)
target_link_options(
  wuji_hand2_bridge PRIVATE "-Wl,--wrap=__isoc23_strtol"
)

install(
  TARGETS
    arm_ik
    marvin_native_driver
    tianji_official_native
    arm_ik_producer
    tianji_official_ik_probe
    tianji_official_ik_worker
    pinocchio_qp_ik_probe
    joint_trajectory_limiter_probe
    wuji_hand2
    wuji_hand2_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS marvin_native_fake_sdk marvin_native_driver_probe
  LIBRARY DESTINATION lib/${PROJECT_NAME}/test
  RUNTIME DESTINATION lib/${PROJECT_NAME}/test
)
install(DIRECTORY include/ DESTINATION include)
install(
  PROGRAMS
    scripts/mocap_live
    scripts/mocap_h5_replay
    scripts/regrind_policy
    scripts/mocap_calibration
    scripts/target_replay
    scripts/joint_replay
    scripts/session_recorder
    scripts/marvin_executor
    scripts/mujoco_executor
    scripts/wuji_hand2_executor
    scripts/arm_command_coordinator
    scripts/policy_hold_producer
    scripts/trace_metrics
    scripts/real_diagnostic
    scripts/h5_wrist_diagnostic
    scripts/joint_watcher
    scripts/hand_tracking_observation
    scripts/hand_tracking_target
    scripts/pico_capture
    scripts/manus_capture
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY tianji_teleop
  DESTINATION lib/python3.10/site-packages
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  DIRECTORY tianji_world_output
  DESTINATION lib/python3.10/site-packages
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  DIRECTORY assets config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "mode" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
