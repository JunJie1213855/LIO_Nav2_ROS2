cmake_minimum_required(VERSION 3.8)
project(bspline_opt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)

add_library(bspline_opt
  src/uniform_bspline.cpp
  src/bspline_optimizer.cpp
  src/gradient_descent_optimizer.cpp)
target_include_directories(bspline_opt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bspline_opt path_searching plan_env rclcpp)
target_link_libraries(bspline_opt Eigen3::Eigen)

install(TARGETS bspline_opt EXPORT export_bspline_opt
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_bspline_opt HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 path_searching plan_env rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_uniform_bspline test/test_uniform_bspline.cpp)
  if(TARGET test_uniform_bspline)
    target_link_libraries(test_uniform_bspline bspline_opt Eigen3::Eigen)
  endif()
endif()

ament_package()
