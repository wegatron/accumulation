cmake_minimum_required(VERSION 2.6)

set( CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};$ENV{HOME}/usr/share/cmake/Modules/")

project(ipopt_demo)

include($ENV{HOME}/usr/share/cmake/Modules/geo_sim_sdk.cmake)

#include_directories(CMAKE_CURRENT_SOURCE_DIR/include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11 -fopenmp -fPIC")

###
##FIND_PACKAGE
###
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# add_subdirectory(src)

# file(GLOB headers *.h)
# file(GLOB src *.cpp)

add_executable(ipopt_demo main.cpp)

target_link_libraries(ipopt_demo ipopt)

# add_library(lib_name SHARED headers src)
# add_subdirectory()

# aux_source_directory_with_headers(. src headers)

# add_library_from_dir(name mode dir)
# add_executable_from_dir(name dir)

# default_install_cxx(TARGET name)
# default_install_c(TARGET name)
