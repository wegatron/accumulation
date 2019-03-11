cmake_minimum_required(VERSION 3.10)
project(dump_test)

set(CMAKE_CXX_STANDARD 11)

set(app_name dump_test)

# include($ENV{HOME}/usr/cmake_modules/cmake_self.cmake)
include($ENV{HOME}/usr/cmake_modules/common.cmake)

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)
file(GLOB headers *.h *.hpp)
file(GLOB src *.cpp)
add_executable(${app_name}
${headers}
${src}
)

target_link_libraries(${app_name}
Dbghelp
)

# 可执行程序的后缀需要额外设置
set_target_properties(${app_name} PROPERTIES
DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
COMPILE_FLAGS "/MP")

# configure the project to set some debug parameters in VS
# configure_file(${CMAKE_SOURCE_DIR}/vs.user.in ${app_name}.vcxproj.user)

# 设置启动项目, 必须在最外层的CMAKELists.txt中设置
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${app_name})

# 可执行程序安装的基本设置
# install(TARGETS ${app_name} RUNTIME DESTINATION .)
# install(CODE 
# "include(BundleUtilities)
# fixup_bundle(${CMAKE_INSTALL_PREFIX}/${app_name}${CMAKE_RELEASE_POSTFIX}.exe \"\" \"${path}\")")"