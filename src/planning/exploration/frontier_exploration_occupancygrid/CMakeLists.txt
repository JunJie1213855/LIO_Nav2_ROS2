cmake_minimum_required(VERSION 3.8)
project(frontier_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-sign-compare -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointArray.msg"
  "srv/GetCentroids.srv"
  DEPENDENCIES geometry_msgs
)

add_library(frontierDetector
  src/frontier_detector.cpp
  src/actuator.cpp
)
target_include_directories(frontierDetector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(frontierDetector
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  nav2_msgs
)
target_link_libraries(frontierDetector ${PROJECT_NAME}__rosidl_typesupport_cpp)

add_executable(frontier_planner src/frontierMain.cpp)
target_include_directories(frontier_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(frontier_planner
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  nav2_msgs
)
target_link_libraries(frontier_planner frontierDetector ${PROJECT_NAME}__rosidl_typesupport_cpp)

install(TARGETS frontier_planner frontierDetector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
