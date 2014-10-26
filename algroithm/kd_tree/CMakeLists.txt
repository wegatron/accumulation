cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH "$ENV{HOME}/usr/share/cmake/Modules/;${CMAKE_MODULE_PATH}")

project(kdtree)

set(APPNAME test_kdtree)

include($ENV{HOME}/usr/share/cmake/Modules/geo_sim_sdk.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

include_geo_sim_sdk()
link_geo_sim_sdk()

set(CMAKE_C_FLAGS "$ENV{CFLAGS} -O3")
set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -fopenmp ${CMAKE_CXX_FLAGS}")

###
##FIND_PACKAGE
###

# add_subdirectory(src)

# file(GLOB source *.cpp)

aux_source_directory_with_headers(. sources headers)

add_executable(${APPNAME} ${sources} ${headers})

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
