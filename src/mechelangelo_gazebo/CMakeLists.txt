cmake_minimum_required(VERSION 3.8)
project(mechelangelo_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/debug_spawn_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
