cmake_minimum_required(VERSION 3.24)
project(tianji_spark_native LANGUAGES CXX)
# Standalone process/library: NEVER link this Pinocchio 3.9 closure into v131.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 3.4 REQUIRED CONFIG)
find_package(mujoco 3.10.0 EXACT REQUIRED CONFIG)
find_package(pinocchio 3.9.0 EXACT REQUIRED CONFIG)
# conda libosqp 1.0.0 ships a 0.0.0 CMake version file. The unchanged
# reference Pixi lock pins the actual package/build; use its exported target.
find_package(osqp REQUIRED CONFIG)
find_package(yaml-cpp 0.8.0 EXACT REQUIRED CONFIG)
find_path(SPARK_QPOASES_INCLUDE qpOASES.hpp REQUIRED)
find_library(SPARK_QPOASES_LIBRARY qpOASES REQUIRED)
include(FetchContent)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_ONLINE_CLIENT OFF CACHE BOOL "" FORCE)
FetchContent_Declare(ruckig
  GIT_REPOSITORY https://github.com/pantor/ruckig.git
  GIT_TAG v0.19.4 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(ruckig)
get_filename_component(SPARK_PACKAGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
get_filename_component(SPARK_WORKSPACE_ROOT "${SPARK_PACKAGE_ROOT}/../.." ABSOLUTE)
add_library(tianji_spark_core STATIC
  bilateral_cycle.cpp
  arm_angle.cpp
  cartesian_otg.cpp
  cartesian_servo.cpp
  config.cpp
  controller.cpp
  equality_constrained_qpoases_solver.cpp
  hierarchical_qp.cpp
  hierarchical_qp_ik.cpp
  hierarchical_qpoases_solver.cpp
  iterative_pose_dls.cpp
  joint_trajectory_limiter.cpp
  mujoco_robot.cpp
  nullspace_dls.cpp
  osqp_solver.cpp
  pinocchio_arm_kinematics.cpp
  pico_teleop_protocol.cpp
  pico_teleop_session.cpp
  qp_builder.cpp
  qpoases_solver.cpp
  safety.cpp
  spark_upper_retarget.cpp
  spark_upper_qpoases_ik.cpp
  spark_posture_reference.cpp
  spark_feedforward_reference.cpp
  spark_constraint_headroom.cpp
  spark_palm_twist_estimator.cpp
  spark_guidance.cpp
  spark_qpoases_diagnostic.cpp
  so3.cpp
  target_manager.cpp
  upper_arm_outward.cpp
  velocity_ik.cpp
)
target_include_directories(tianji_spark_core PUBLIC
  "${SPARK_PACKAGE_ROOT}/include" "${SPARK_QPOASES_INCLUDE}")
target_link_libraries(tianji_spark_core PUBLIC Eigen3::Eigen mujoco::mujoco
  pinocchio::pinocchio osqp::osqp yaml-cpp::yaml-cpp
  "${SPARK_QPOASES_LIBRARY}" ruckig::ruckig)
target_compile_options(tianji_spark_core PRIVATE -Wall -Wextra -Wpedantic)
add_executable(spark_native_worker native_worker.cpp)
target_link_libraries(spark_native_worker PRIVATE tianji_spark_core)
# Optional read-only oracle build. No reference checkout is required by the
# deployed library/worker, and the two namespaces are never linked together.
set(SPARK_REFERENCE_ROOT "" CACHE PATH "Pinned reference source checkout for conformance only")
if(SPARK_REFERENCE_ROOT)
  get_target_property(SPARK_MIGRATED_SOURCES tianji_spark_core SOURCES)
  list(REMOVE_ITEM SPARK_MIGRATED_SOURCES bilateral_cycle.cpp)
  set(SPARK_ORIGINAL_SOURCES)
  foreach(source IN LISTS SPARK_MIGRATED_SOURCES)
    list(APPEND SPARK_ORIGINAL_SOURCES "${SPARK_REFERENCE_ROOT}/src/${source}")
  endforeach()
  foreach(source acceleration_bounds acceleration_controller acceleration_qp
      acceleration_qpoases_solver cartesian_acceleration_servo)
    list(APPEND SPARK_ORIGINAL_SOURCES "${SPARK_REFERENCE_ROOT}/src/${source}.cpp")
  endforeach()
  add_library(tianji_spark_reference_core STATIC ${SPARK_ORIGINAL_SOURCES})
  target_include_directories(tianji_spark_reference_core PUBLIC
    "${SPARK_REFERENCE_ROOT}/include" "${SPARK_QPOASES_INCLUDE}")
  target_link_libraries(tianji_spark_reference_core PUBLIC Eigen3::Eigen mujoco::mujoco
    pinocchio::pinocchio osqp::osqp yaml-cpp::yaml-cpp
    "${SPARK_QPOASES_LIBRARY}" ruckig::ruckig)
  add_executable(spark_reference_worker "${SPARK_WORKSPACE_ROOT}/tests/cpp/spark/reference_worker.cpp")
  target_link_libraries(spark_reference_worker PRIVATE tianji_spark_reference_core)
endif()
include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  foreach(name so3 pico_teleop_protocol pico_teleop_session spark_upper_retarget
      spark_posture_reference spark_feedforward_reference spark_constraint_headroom
      spark_palm_twist_estimator bilateral_cycle)
    add_executable(spark_test_${name}
      "${SPARK_WORKSPACE_ROOT}/tests/cpp/spark/test_${name}.cpp")
    target_link_libraries(spark_test_${name} PRIVATE tianji_spark_core GTest::gtest_main)
    target_compile_definitions(spark_test_${name} PRIVATE
      TIANJI_PROJECT_SOURCE_DIR="${SPARK_WORKSPACE_ROOT}")
    # These two original tests inspect the reference Viewer's UI source text.
    # This port does not ship that Viewer; preserve tests verbatim but exclude
    # those frontend checks explicitly, rather than claim them as port coverage.
    if(name STREQUAL "pico_teleop_session")
      add_test(NAME spark_${name} COMMAND spark_test_${name}
        "--gtest_filter=-PicoTeleopSession.ViewerSourceExposesPicoControlsAndCliContract:PicoTeleopSession.PicoPauseGatesManusHandUpdates")
    else()
      add_test(NAME spark_${name} COMMAND spark_test_${name})
    endif()
  endforeach()
endif()
