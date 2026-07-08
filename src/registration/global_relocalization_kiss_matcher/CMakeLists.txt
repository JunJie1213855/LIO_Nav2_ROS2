cmake_minimum_required(VERSION 3.22)
project(global_relocalization_kiss_matcher)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set a default build type if none is specified
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    # Provide options for cmake-gui or ccmake
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
cmake_policy(SET CMP0074 NEW)

option(USE_ADDRESS_SANITIZER "Use address sanitizer" OFF)
if (USE_ADDRESS_SANITIZER)
    add_compile_options(-fsanitize=address -fsanitize-recover=address)
    add_link_options(-fsanitize=address -fsanitize-recover=address)
endif()

find_package(TBB REQUIRED)
find_package(robin REQUIRED)
find_package(kiss_matcher REQUIRED)

# Load small_gicp
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
include(FetchContent)
FetchContent_Declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp
  GIT_TAG master
)
FetchContent_MakeAvailable(small_gicp)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(
    include
    src
    ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)

################################################################################
add_executable(run_kiss_matcher src/run_kiss_matcher.cpp)
target_include_directories(run_kiss_matcher
    PUBLIC
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(run_kiss_matcher
    kiss_matcher::kiss_matcher_core
    ${PCL_LIBRARIES}
)

add_executable(registration_visualizer src/registration_visualizer.cpp)
ament_target_dependencies(registration_visualizer
  rclcpp
  tf2_ros
  sensor_msgs
  pcl_conversions
  PCL
)
target_link_libraries(registration_visualizer
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
)

add_executable(kiss_matcher_sam src/slam/pose_graph_manager.cpp
                                src/slam/loop_closure.cpp
                                src/slam/loop_detector.cpp)
ament_target_dependencies(kiss_matcher_sam
  rclcpp
  tf2_ros
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  PCL
)

target_link_libraries(kiss_matcher_sam
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
  small_gicp
  gtsam
)

add_executable(inter_frame_alignment src/inter_frame_alignment.cpp
                                     src/slam/loop_closure.cpp)
ament_target_dependencies(inter_frame_alignment
  rclcpp
  tf2_ros
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  PCL
)

target_link_libraries(inter_frame_alignment
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
  small_gicp
  gtsam
)

add_library(global_small_gicp_relocalization_node SHARED
  src/global_small_gicp_relocalization.cpp
  src/slam/loop_closure.cpp
)
target_include_directories(global_small_gicp_relocalization_node PUBLIC include)
target_link_libraries(global_small_gicp_relocalization_node
  ${PCL_LIBRARIES}
  small_gicp
  OpenMP::OpenMP_CXX
  kiss_matcher::kiss_matcher_core
  gtsam
)
ament_target_dependencies(global_small_gicp_relocalization_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)
rclcpp_components_register_node(global_small_gicp_relocalization_node
  PLUGIN "global_small_gicp_relocalization::GlobalSmallGicpRelocalizationNode"
  EXECUTABLE global_small_gicp_relocalization_exec
)

add_library(global_kiss_matcher_relocalization_node SHARED
  src/global_kiss_matcher_relocalization.cpp
  src/slam/loop_closure.cpp
)
target_include_directories(global_kiss_matcher_relocalization_node PUBLIC include)
target_link_libraries(global_kiss_matcher_relocalization_node
  ${PCL_LIBRARIES}
  small_gicp
  OpenMP::OpenMP_CXX
  kiss_matcher::kiss_matcher_core
  gtsam
)
ament_target_dependencies(global_kiss_matcher_relocalization_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)
rclcpp_components_register_node(global_kiss_matcher_relocalization_node
  PLUGIN "global_kiss_matcher_relocalization::GlobalKissMatcherRelocalizationNode"
  EXECUTABLE global_kiss_matcher_relocalization_exec
)

install(TARGETS
  registration_visualizer
  run_kiss_matcher
  kiss_matcher_sam
  inter_frame_alignment
  global_small_gicp_relocalization_node
  global_kiss_matcher_relocalization_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
