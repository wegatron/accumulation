cmake_minimum_required(VERSION 3.10)
project(zsw_io)

set(CMAKE_CXX_STANDARD 14)

include(GenerateExportHeader)

if(WIN32)
include($ENV{HOME}/usr/cmake_modules/cmake_self.cmake)
include($ENV{HOME}/usr/cmake_modules/common.cmake)
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

#PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common features filters kdtree ml octree registration io segmentation visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

#boost
find_package(Boost COMPONENTS  filesystem regex REQUIRED)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB headers *.h *.hpp)
file(GLOB src *.cpp)

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

add_executable(vtk_demo
vtk_demo.cxx)

target_link_libraries(vtk_demo
io_common
${VTK_LIBRARIES}
${PCL_LIBRARIES}
${Boost_LIBRARIES}
)

if(WIN32)
# 可执行程序的后缀需要额外设置
set_target_properties(vtk_demo PROPERTIES
DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
COMPILE_FLAGS "/MP")

# configure the project to set some debug parameters in VS
configure_file(${CMAKE_SOURCE_DIR}/vs.user.in vtk_demo.vcxproj.user)

# 设置启动项目, 必须在最外层的CMAKELists.txt中设置
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT vtk_demo)
endif(WIN32)

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