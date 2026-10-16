cmake_minimum_required(VERSION 3.11)
project(global_relocalization)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
# add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
ament_auto_find_build_dependencies()

# Prefer system-installed small_gicp (avoids GitHub access at configure time)
find_package(small_gicp QUIET)
if(small_gicp_FOUND)
  message(STATUS "Using system-installed small_gicp")
  add_library(small_gicp ALIAS small_gicp::small_gicp)
else()
  message(STATUS "Fetching small_gicp via FetchContent...")
  include(FetchContent)
  FetchContent_Declare(
    small_gicp
    GIT_REPOSITORY https://github.com/koide3/small_gicp.git
    GIT_TAG "master"
  )
  FetchContent_MakeAvailable(small_gicp)
endif()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  OpenMP::OpenMP_CXX
  small_gicp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::SmallGicpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
)
