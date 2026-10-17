cmake_minimum_required(VERSION 3.16)
project(go2_description)

find_package(ament_cmake REQUIRED)

# The URDF refers to meshes as package://go2_description/dae/*.dae
install(DIRECTORY urdf dae DESTINATION share/${PROJECT_NAME})

ament_package()
