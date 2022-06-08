cmake_minimum_required(VERSION 3.10)
project(opengles_windows)

set(CMAKE_CXX_STANDARD 17)

set(app_name opengles_windows)

list(APPEND CMAKE_PREFIX_PATH "e:/usr/glm/cmake/glm")

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(GLM REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})
link_directories(${GLM_LIBRARY_DIRS})
add_definitions(${GLM_DEFINITIONS})


include_directories(e:/usr/pvr_sdk/include)
set(prv_gles_libs 
e:/usr/pvr_sdk/lib/Windows_x86_64/libGLESv2.lib;e:/usr/pvr_sdk/lib/Windows_x86_64/libEGL.lib;e:/usr/pvr_sdk/lib/Windows_x86_64/libGLES_CM.lib)

file(GLOB headers *.h *.hpp)
file(GLOB src *.cpp)
add_executable(${app_name}
${headers}
${src}
)

target_link_libraries(${app_name}
${prv_gles_libs}
)


configure_file(${CMAKE_SOURCE_DIR}/vs.user.in ${app_name}.vcxproj.user)

# 可执行程序的后缀需要额外设置
set_target_properties(${app_name} PROPERTIES
DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
COMPILE_FLAGS "/MP")

# 设置启动项目, 必须在最外层的CMAKELists.txt中设置
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${app_name})