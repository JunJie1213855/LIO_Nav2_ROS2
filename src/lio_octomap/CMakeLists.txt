cmake_minimum_required(VERSION 3.10)
project(lio_octomap)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
