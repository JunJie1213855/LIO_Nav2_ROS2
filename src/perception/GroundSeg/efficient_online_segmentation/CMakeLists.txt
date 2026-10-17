cmake_minimum_required(VERSION 3.8)
project(efficient_online_segmentation)

# Default to C++14; the algorithm core is C++11-compatible.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror=return-type)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED COMPONENTS core)

set(CORE_SOURCES
  core/efficient_online_segmentation.cpp
  core/smart_sector.cpp
  core/segmentation_utility.cpp
)

set(AMENT_DEPS
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
)

add_library(ground_seg_core STATIC ${CORE_SOURCES})
target_include_directories(ground_seg_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ground_seg_core PUBLIC ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_library(ground_seg_ros STATIC ros_interface/ground_separation_node.cpp)
target_link_libraries(ground_seg_ros ground_seg_core)
ament_target_dependencies(ground_seg_ros ${AMENT_DEPS})

foreach(target ground_separation_node cloud_only_segmentation_node efficient_online_segmentation_node)
  add_executable(${target} ros_interface/main.cpp)
  target_link_libraries(${target} ground_seg_ros)
  ament_target_dependencies(${target} rclcpp)
  if(NOT target STREQUAL "ground_separation_node")
    target_compile_definitions(${target} PRIVATE GROUND_SEG_CLOUD_ONLY)
  endif()
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_registered_cloud
    test/test_registered_cloud.py
    TIMEOUT 45
    ENV
      "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
      "GROUND_SEPARATION_EXECUTABLE=${CMAKE_CURRENT_BINARY_DIR}/ground_separation_node"
      "GROUND_SEG_CLOUD_ONLY_EXECUTABLE=${CMAKE_CURRENT_BINARY_DIR}/cloud_only_segmentation_node"
      "GROUND_SEPARATION_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/gazebo_mid360.yaml")
  ament_add_gtest(test_segmentation_startup
    test/test_segmentation_startup.cpp)
  target_include_directories(test_segmentation_startup PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_segmentation_startup ground_seg_core)
  ament_add_gtest(test_point_cloud_io test/test_point_cloud_io.cpp)
  target_link_libraries(test_point_cloud_io ground_seg_core)
  ament_target_dependencies(test_point_cloud_io sensor_msgs)
endif()

ament_package()
