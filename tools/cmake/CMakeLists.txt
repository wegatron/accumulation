cmake_minimum_required(VERSION 3.10)
project(test_lib VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)

# 设置编译输出的路径
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

file(GLOB headers include/*.h include/*.hpp)
file(GLOB src *.cpp)

add_library(test_lib
    ${headers}
    ${src})

set_target_properties(test_lib PROPERTIES PUBLIC_HEADER ${headers})

target_include_directories(test_lib PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_include_directories(test_lib INTERFACE  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>  # <prefix>/include/mylib
)

# install
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        test_libConfigVersion.cmake
        VERSION ${PACKAGE_VERSION}
        COMPATIBILITY AnyNewerVersion  # 表示该函数库向下兼容
)

install(TARGETS test_lib
        EXPORT test_lib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include)

install(EXPORT test_lib
        FILE test_lib.cmake
        NAMESPACE test_lib::
        DESTINATION lib/cmake)

configure_file(test_lib_config.cmake.in test_libConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/test_libConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/test_libConfigVersion.cmake"
        DESTINATION lib/cmake
        )