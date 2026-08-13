cmake_minimum_required(VERSION 3.8)
project(map_generator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(map_pub src/map_publisher.cpp)
target_include_directories(map_pub PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(map_pub pcl_conversions rclcpp sensor_msgs)
target_link_libraries(map_pub Eigen3::Eigen ${PCL_LIBRARIES})
install(TARGETS map_pub DESTINATION lib/${PROJECT_NAME})

ament_package()
