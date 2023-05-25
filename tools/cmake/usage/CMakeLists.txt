cmake_minimum_required(VERSION 3.10)
project(test_lib_use)

set(CMAKE_CXX_STANDARD 11)

set(app_name test_lib_use)

list(APPEND CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/../install/lib/cmake)


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

find_package(test_lib 1.0)

target_link_libraries(${app_name}
    test_lib::test_lib
)

