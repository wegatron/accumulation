cmake_minimum_required(VERSION 2.8)

enable_language(Fortran)

set( CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};$ENV{HOME}/usr/share/cmake/Modules/")

find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(EIGEN3_FOUND)

set(SRC_DIR "{${CMAKE_CURRENT_SOURCE_DIR},$ENV{HOME}/usr/share/maxima}")

add_custom_command(OUTPUT collision_ball_energy.f90
  COMMAND "maxima" ARGS --userdir=${SRC_DIR} -b collision_ball_energy.mac
  DEPENDS collision_ball_energy.mac)
add_library(collision_energy SHARED collision_ball_energy.f90)

# add_executable(test_collision_energy test.cpp)

# target_link_libraries(test_collision_energy
#          collision_energy
#          spring
# )
