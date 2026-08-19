cmake_minimum_required(VERSION 3.16)
project(pico_body_tianji)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

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
  src/arm_ik_factory.cpp
  src/pinocchio_cpp/pinocchio_arm_ik.cpp
  src/pinocchio_qp/pinocchio_qp_arm_ik.cpp
  src/tianji_official/tianji_official_arm_ik_proxy.cpp
)
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
  arm_ik PUBLIC Eigen3::Eigen pinocchio::pinocchio ${CMAKE_DL_LIBS}
)

# 厂商 libKine 与 Pinocchio 的依赖闭包不能安全共存。native 库只链接
# Eigen/dl，供独立 worker 和 probe 使用；ROS 主节点内使用 IPC proxy。
add_library(
  tianji_official_native STATIC
  src/tianji_official/tianji_official_arm_ik.cpp
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

# 该节点名称沿用已验证的话题兼容名；它只做所选后端的 IK/FK，
# 不直接连接机械臂。RViz/MuJoCo 与真机桥接仅订阅其输出。
add_executable(
  tianji_kinematic_sim src/tianji_kinematic_sim_node.cpp
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
target_compile_features(tianji_kinematic_sim PUBLIC cxx_std_17)
target_compile_features(tianji_official_ik_probe PUBLIC cxx_std_17)
target_compile_features(tianji_official_ik_worker PUBLIC cxx_std_17)
target_compile_features(pinocchio_qp_ik_probe PUBLIC cxx_std_17)
target_compile_options(
  tianji_kinematic_sim PRIVATE -Wall -Wextra -Wpedantic
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
target_link_libraries(tianji_kinematic_sim arm_ik)
target_link_libraries(tianji_official_ik_probe tianji_official_native)
target_link_libraries(tianji_official_ik_worker tianji_official_native)
target_link_libraries(pinocchio_qp_ik_probe arm_ik)
# 便携 runtime 层为 glibc 2.35：把 glibc 2.38 的 __isoc23_strtol
# 引用改接到 shim（见 glibc_isoc23_strtol_shim.cpp）。
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
ament_target_dependencies(
  tianji_kinematic_sim
  ament_index_cpp
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS
    arm_ik
    tianji_official_native
    tianji_kinematic_sim
    tianji_official_ik_probe
    tianji_official_ik_worker
    pinocchio_qp_ik_probe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  PROGRAMS
    scripts/pico_controller_input
    scripts/pico_controller_only_input
    scripts/pico_link_probe
    scripts/controller_only_trace
    scripts/controller_only_real_diagnostic
    scripts/marvin_hardware_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY assets config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
