cmake_minimum_required(VERSION 2.6)

project(dynamic_link_example)

set(APPNAME dl_test)

include($ENV{HOME}/usr/share/cmake/Modules/geo_sim_sdk.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

include_geo_sim_sdk()
link_geo_sim_sdk()

set(CMAKE_C_FLAGS "$ENV{CFLAGS} -O3")
set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -fopenmp ${CMAKE_CXX_FLAGS}")

###
##FIND_PACKAGE
###

include_directories(
        /usr/include/glib-2.0
        /usr/lib/x86_64-linux-gnu/glib-2.0/include/
)

# add_subdirectory(src)

# file(GLOB source *.cpp)

# aux_source_directory_with_headers(. sources headers)

add_executable(${APPNAME}  main.cpp)

add_library(dl_solver SHARED dl_solver.h dl_solver.cpp)

add_library(solver_pack SHARED dllib/zsw_solver.h dllib/zsw_solver.cpp)

target_link_libraries(${APPNAME}
  dl_solver
  gmodule-2.0
)

# add_library_from_dir(name mode dir)
# add_executable_from_dir(name dir)

## install header
# install(DIRECTORY include/
#   DESTINATION include/
#   FILES_MATCHING PATTERN "**.h" REGEX ".svn.*" EXCLUDE)

## install bin or library in /os/bit/compiler/
# geo_sim_sdk_install_cxx(TARGET name)

## install bin or library in /os/bit/
# geo_sim_sdk_install_c(TARGET name)
