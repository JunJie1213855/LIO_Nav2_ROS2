cmake_minimum_required(VERSION 3.8)
project(traj_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(traj_utils src/planning_visualization.cpp src/polynomial_traj.cpp)
target_include_directories(traj_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(traj_utils bspline_opt rclcpp std_msgs visualization_msgs)
target_link_libraries(traj_utils Eigen3::Eigen)

install(TARGETS traj_utils EXPORT export_traj_utils
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_traj_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(bspline_opt Eigen3 rclcpp std_msgs visualization_msgs)
ament_package()
