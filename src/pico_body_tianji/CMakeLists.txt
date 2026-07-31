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

add_library(pinocchio_arm_ik src/pinocchio_arm_ik.cpp)
target_compile_features(pinocchio_arm_ik PUBLIC cxx_std_17)
target_compile_options(
  pinocchio_arm_ik PRIVATE -Wall -Wextra -Wpedantic
)
target_include_directories(
  pinocchio_arm_ik
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  pinocchio_arm_ik PUBLIC Eigen3::Eigen pinocchio::pinocchio
)

# 该节点名称沿用已验证的话题兼容名；它只做 Pinocchio IK/FK，
# 不加载 libKine，也不直接连接机械臂。RViz/MuJoCo 仅订阅其输出。
add_executable(
  tianji_kinematic_sim src/tianji_kinematic_sim_node.cpp
)
target_compile_features(tianji_kinematic_sim PUBLIC cxx_std_17)
target_compile_options(
  tianji_kinematic_sim PRIVATE -Wall -Wextra -Wpedantic
)
target_link_libraries(tianji_kinematic_sim pinocchio_arm_ik)
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
  TARGETS pinocchio_arm_ik tianji_kinematic_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  PROGRAMS
    scripts/pico_controller_input
    scripts/marvin_hardware_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY assets config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
