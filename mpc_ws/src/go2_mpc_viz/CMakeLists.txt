cmake_minimum_required(VERSION 3.16)
project(go2_mpc_viz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(go2_mpc_msgs REQUIRED)

add_executable(go2_mpc_viz src/viz_node.cpp)
# Only the dependency-free wire format header is shared with the controller.
target_include_directories(go2_mpc_viz PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../go2_mpc/include
)
target_compile_options(go2_mpc_viz PRIVATE -Wall -Wextra)
ament_target_dependencies(go2_mpc_viz rclcpp sensor_msgs geometry_msgs tf2_ros visualization_msgs go2_mpc_msgs)

install(TARGETS go2_mpc_viz DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
