cmake_minimum_required(VERSION 3.10)
project(visit_struct)

set(CMAKE_CXX_STANDARD 14)

set(app_name visit_struct)

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
file(GLOB headers *.h *.hpp)
file(GLOB src *.cpp)
add_executable(${app_name}
${headers}
${src}
)