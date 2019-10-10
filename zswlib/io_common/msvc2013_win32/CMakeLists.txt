cmake_minimum_required(VERSION 3.10)
project(zsw_io)

set(CMAKE_CXX_STANDARD 11)

include(GenerateExportHeader)

if(WIN32)
include($ENV{HOME}/usr/cmake_modules/cmake_self_win32.cmake)
# include($ENV{HOME}/usr/cmake_modules/common.cmake)

set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_RELEASE_POSTFIX "")
endif(WIN32)

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

#for python
find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

#PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common features filters kdtree ml octree registration io segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

#boost
find_package(Boost COMPONENTS  filesystem regex REQUIRED)
# add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#for openmp
find_package(OpenMP)
if (OPENMP_FOUND)
message("openmp enabled!!!")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB headers ../*.h ../*.hpp)
file(GLOB src ../*.cpp)

add_library(io_common SHARED
${headers}
${src}
)
generate_export_header(io_common EXPORT_FILE_NAME ${CMAKE_CURRENT_SOURCE_DIR}/io_common_export.h)
target_compile_definitions(io_common PRIVATE io_common_EXPORTS)

target_link_libraries(io_common
${VTK_LIBRARIES}
${PCL_LIBRARIES}
${Boost_LIBRARIES}
)

install(TARGETS io_common
ARCHIVE DESTINATION lib
LIBRARY DESTINATION  lib
RUNTIME DESTINATION  bin
)

install(FILES ${headers} DESTINATION include)

# # 可执行程序安装的基本设置
# install(TARGETS demo_io RUNTIME DESTINATION .)
# install(CODE 
# "include(BundleUtilities)
# fixup_bundle(${CMAKE_INSTALL_PREFIX}/demo_io${CMAKE_RELEASE_POSTFIX}.exe \"\" \"${path}\")")"