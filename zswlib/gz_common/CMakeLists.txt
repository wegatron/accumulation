cmake_minimum_required(VERSION 3.10)
project(gz_common)

set(CMAKE_CXX_STANDARD 14)

set(app_name gz_common)

include($ENV{HOME}/usr/cmake_modules/cmake_self.cmake)
include($ENV{HOME}/usr/cmake_modules/common.cmake)
include(GenerateExportHeader)

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

file(GLOB headers *.h *.hpp)
file(GLOB src *.cpp)

add_executable(${app_name}
demo.cxx)

add_library(gz_common
${headers}
${src}
)

generate_export_header(gz_common)

target_link_libraries(${app_name}
gz_common
${VTK_LIBRARIES}
${PCL_LIBRARIES}
)

# 可执行程序的后缀需要额外设置
set_target_properties(${app_name} PROPERTIES
DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
COMPILE_FLAGS "/MP")

# configure the project to set some debug parameters in VS
configure_file(${CMAKE_SOURCE_DIR}/vs.user.in ${app_name}.vcxproj.user)

# 设置启动项目, 必须在最外层的CMAKELists.txt中设置
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${app_name})

# # 可执行程序安装的基本设置
# install(TARGETS ${app_name} RUNTIME DESTINATION .)
# install(CODE 
# "include(BundleUtilities)
# fixup_bundle(${CMAKE_INSTALL_PREFIX}/${app_name}${CMAKE_RELEASE_POSTFIX}.exe \"\" \"${path}\")")"