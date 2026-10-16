option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.8.1
    BINARY_DIR ${CMAKE_BINARY_DIR}/googletest-src
    BINARY_DIR ${CMAKE_BINARY_DIR}/googletest-build
)
FetchContent_MakeAvailable(googletest)

FetchContent_Declare(
    pmc
    GIT_REPOSITORY https://github.com/jingnanshi/pmc.git
    GIT_TAG master
)
FetchContent_MakeAvailable(pmc)

FetchContent_Declare(
    tinyply
    GIT_REPOSITORY https://github.com/jingnanshi/tinyply.git
    GIT_TAG 0b9fff8e8bd4d37256554fe40cf76b2f3134377b
)
FetchContent_MakeAvailable(tinyply)

add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
add_subdirectory(${pmc_SOURCE_DIR} ${pmc_BINARY_DIR})
add_subdirectory(${tinyply_SOURCE_DIR} ${tinyply_BINARY_DIR})

FetchContent_Declare(teaserpp URL https://github.com/MIT-SPARK/TEASER-plusplus/archive/refs/tags/v2.0.tar.gz)
FetchContent_GetProperties(teaserpp)
if(NOT teaserpp_POPULATED)
  FetchContent_Populate(teaserpp)
  if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
    add_subdirectory(${teaserpp_SOURCE_DIR} ${teaserpp_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
  else()
    # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the compiler will
    # consider this 3rdparty headers as source code and fail due the -Werror flag.
    add_subdirectory(${teaserpp_SOURCE_DIR} ${teaserpp_BINARY_DIR} EXCLUDE_FROM_ALL)
    get_target_property(teaserpp_include_dirs teaserpp INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(teaserpp PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${teaserpp_include_dirs}")
  endif()
endif()
