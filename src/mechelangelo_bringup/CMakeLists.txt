cmake_minimum_required(VERSION 3.8)
project(mechelangelo_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds Models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()