cmake_minimum_required(VERSION 3.8)
project(mockamap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(mockamap_sources src/mockamap.cpp src/maps.cpp src/perlinnoise.cpp)
add_executable(mockamap_node ${mockamap_sources})
target_include_directories(mockamap_node PRIVATE include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(mockamap_node
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)
target_link_libraries(mockamap_node Eigen3::Eigen ${PCL_LIBRARIES})

install(TARGETS mockamap_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
