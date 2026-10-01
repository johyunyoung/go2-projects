# READ-ONLY: no publishers
# go2_inspect : Unitree Go2 상태 확인 도구 (구독만 함, 어떤 토픽에도 발행하지 않음)
cmake_minimum_required(VERSION 3.10)
project(go2_inspect CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# unitree_sdk2 를 /opt/unitree_robotics 에 설치한 경우도 찾을 수 있도록 추가.
# (/usr/local 설치는 기본 탐색 경로에 포함됨)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics")

find_package(unitree_sdk2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_executable(go2_inspect
  src/main.cpp
  src/state_cache.cpp
  src/terminal.cpp
  src/monitor_mode.cpp
  src/guided_mode.cpp
  src/calib_writer.cpp
)

target_compile_options(go2_inspect PRIVATE -Wall -Wextra)

# yaml-cpp 버전에 따라 타깃 이름이 다름 (0.5.x / 0.6.x: yaml-cpp, 0.8+: yaml-cpp::yaml-cpp)
if(TARGET yaml-cpp::yaml-cpp)
  set(GO2_YAML_LIB yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(GO2_YAML_LIB yaml-cpp)
else()
  set(GO2_YAML_LIB ${YAML_CPP_LIBRARIES})
  target_include_directories(go2_inspect PRIVATE ${YAML_CPP_INCLUDE_DIR})
endif()

target_link_libraries(go2_inspect PRIVATE unitree_sdk2 ${GO2_YAML_LIB} Threads::Threads)
