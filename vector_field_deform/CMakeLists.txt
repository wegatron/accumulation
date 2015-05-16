cmake_minimum_required(VERSION 2.8)

enable_language(Fortran)

set( CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};$ENV{HOME}/usr/share/cmake/Modules/")

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(EIGEN3_FOUND)

# set(SRC_DIR "{${CMAKE_CURRENT_SOURCE_DIR},$ENV{HOME}/usr/share/maxima}")

# add_custom_command(OUTPUT functions.f
#   COMMAND "maxima" ARGS --userdir=${SRC_DIR} -b functions.mac
#    COMMAND "sed" ARGS -r "'s/ sqrt\\(([0-9]+)\\)/ sqrt\(\\1\\.0\)/g'" -i functions.f
#    COMMAND "sed" ARGS -r "'s/\\[/\(/g'" -i functions.f
#    COMMAND "sed" ARGS -r "'s/\\]/\)/g'" -i functions.f
#   DEPENDS functions.mac)

# add_library(functions SHARED functions.f)

file(GLOB src *.cpp)
file(GLOB headers *.h)

add_library(vector_fields SHARED ${src} ${headers})
add_subdirectory(test)
