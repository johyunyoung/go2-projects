cmake_minimum_required(VERSION 3.16)
project(go2_mpc C CXX)

# go2_mpc links unitree_sdk2 only. It deliberately does not depend on ROS: the same
# binary drives unitree_mujoco on the development PC and the real Go2 from its Jetson.
# Debug data leaves the process as UDP packets (see debug_packet.hpp); the ROS viewer
# (go2_mpc_viz) runs as a separate process on the development PC.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(GO2_BUILD_TESTS "Build unit tests (needs GTest)" ON)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(unitree_sdk2 REQUIRED)

# ---------------------------------------------------------------------------
# Vendored QP stack: BLASFEO + HPIPM (see ../../thirdparty/VERSIONS.md)
# ---------------------------------------------------------------------------
set(GO2_THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty"
    CACHE PATH "Directory holding vendored blasfeo/ and hpipm/")

# Both libraries read the same cache variable TARGET but accept different names, so it
# is set right before each add_subdirectory. BLASFEO holds the architecture-specific
# kernels; Cortex-A57 kernels run on every ARMv8 Jetson (Nano: A57, Orin: A78AE).
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(_blasfeo_default ARMV8A_ARM_CORTEX_A57)
  set(_hpipm_default GENERIC)
else()
  set(_blasfeo_default X64_INTEL_HASWELL)
  set(_hpipm_default AVX)
endif()
set(GO2_BLASFEO_TARGET ${_blasfeo_default} CACHE STRING "BLASFEO TARGET architecture")
set(GO2_HPIPM_TARGET ${_hpipm_default} CACHE STRING "HPIPM TARGET (AVX or GENERIC)")

set(BLASFEO_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BLASFEO_TESTING OFF CACHE BOOL "" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(TARGET ${GO2_BLASFEO_TARGET} CACHE STRING "" FORCE)
add_subdirectory(${GO2_THIRDPARTY_DIR}/blasfeo ${CMAKE_BINARY_DIR}/blasfeo EXCLUDE_FROM_ALL)
set(TARGET ${GO2_HPIPM_TARGET} CACHE STRING "" FORCE)
add_subdirectory(${GO2_THIRDPARTY_DIR}/hpipm ${CMAKE_BINARY_DIR}/hpipm EXCLUDE_FROM_ALL)
set(CMAKE_VERBOSE_MAKEFILE OFF CACHE BOOL "" FORCE)  # BLASFEO turns this on
message(STATUS "go2_mpc: BLASFEO target ${GO2_BLASFEO_TARGET}, HPIPM target ${GO2_HPIPM_TARGET}")

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(go2_mpc_core STATIC
  src/config.cpp
  src/debug_link.cpp
  src/fsm.cpp
  src/kinematics.cpp
  src/remote.cpp
  src/robot_interface.cpp
  src/foothold_planner.cpp
  src/gait_scheduler.cpp
  src/leg_controller.cpp
  src/swing_test_controller.cpp
  src/swing_trajectory.cpp
  src/mpc_controller.cpp
  src/script.cpp
  src/sim_noise.cpp
  src/srb_mpc.cpp
  src/state_estimator.cpp
  src/stand_controller.cpp
)
target_include_directories(go2_mpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(go2_mpc_core PUBLIC
  Eigen3::Eigen
  yaml-cpp
  unitree_sdk2
  hpipm
  blasfeo
  Threads::Threads
)
target_compile_options(go2_mpc_core PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------
add_executable(go2_mpc apps/go2_mpc_main.cpp)
target_link_libraries(go2_mpc PRIVATE go2_mpc_core)

# Solves a 4-leg standing SRB problem of the real size (N=15, 12 states, 12 inputs)
# to check the HPIPM build and measure solve time on this machine.
add_executable(hpipm_smoke apps/hpipm_smoke.cpp)
target_link_libraries(hpipm_smoke PRIVATE go2_mpc_core)

# ---------------------------------------------------------------------------
# Tests (development PC). The FK test also compares against MuJoCo's own forward
# kinematics of the simulator model when MuJoCo is available.
# ---------------------------------------------------------------------------
if(GO2_BUILD_TESTS)
  find_package(GTest)
  if(GTest_FOUND)
    enable_testing()
    add_executable(go2_mpc_tests test/test_kinematics.cpp test/test_srb_mpc.cpp test/test_swing.cpp test/test_estimator.cpp)
    target_link_libraries(go2_mpc_tests PRIVATE go2_mpc_core GTest::gtest_main)

    set(GO2_MUJOCO_DIR "/opt/mujoco" CACHE PATH "MuJoCo install (include/, lib/)")
    set(GO2_MUJOCO_MODEL "/opt/unitree_mujoco/unitree_robots/go2/go2.xml"
        CACHE FILEPATH "go2.xml used by the simulator")
    find_library(GO2_MUJOCO_LIB mujoco PATHS ${GO2_MUJOCO_DIR}/lib NO_DEFAULT_PATH)
    if(GO2_MUJOCO_LIB AND EXISTS ${GO2_MUJOCO_MODEL})
      target_include_directories(go2_mpc_tests PRIVATE ${GO2_MUJOCO_DIR}/include)
      target_link_libraries(go2_mpc_tests PRIVATE ${GO2_MUJOCO_LIB})
      target_compile_definitions(go2_mpc_tests PRIVATE
        GO2_HAVE_MUJOCO GO2_MUJOCO_MODEL="${GO2_MUJOCO_MODEL}")
      set_target_properties(go2_mpc_tests PROPERTIES BUILD_RPATH ${GO2_MUJOCO_DIR}/lib)
      message(STATUS "go2_mpc: tests compare FK against MuJoCo (${GO2_MUJOCO_MODEL})")
    else()
      message(STATUS "go2_mpc: MuJoCo not found, FK-vs-MuJoCo test skipped")
    endif()

    include(GoogleTest)
    gtest_discover_tests(go2_mpc_tests)
  else()
    message(STATUS "go2_mpc: GTest not found, tests skipped")
  endif()
endif()

install(TARGETS go2_mpc hpipm_smoke RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config scripts DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
