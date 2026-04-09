cmake_minimum_required(VERSION 3.8)
project(mechelangelo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()