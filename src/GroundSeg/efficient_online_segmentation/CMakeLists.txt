cmake_minimum_required(VERSION 3.8)
project(efficient_online_segmentation)

# Default to C++14; the algorithm core is C++11-compatible.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

set(CORE_SOURCES
  core/efficient_online_segmentation.cpp
  core/smart_sector.cpp
  core/segmentation_utility.cpp
  core/adaptive_ground_plane_estimator.cpp
)

set(AMENT_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  pcl_conversions
  tf2_ros
  tf2_geometry_msgs
)

# Helper: build a node executable on top of the shared segmentation core.
function(add_segmentation_node target_name source_file)
  add_executable(${target_name} ${source_file} ${CORE_SOURCES})

  target_include_directories(${target_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )

  target_link_libraries(${target_name}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )

  ament_target_dependencies(${target_name} ${AMENT_DEPS})

  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

add_segmentation_node(efficient_online_segmentation_node
  ros_interface/efficient_online_segmentation_node.cpp)
add_segmentation_node(ground_separation_node
  ros_interface/ground_separation_node.cpp)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
