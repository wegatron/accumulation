project(test)

cmake_minimum_required(VERSION 2.6)

#set(CMAKE_EXE_LINKER_FLAGS "-Wl,--enable-new-dtags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath='$ORIGIN',--enable-new-dtags")

#boost
find_package(Boost COMPONENTS  filesystem REQUIRED)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


add_executable(test_demo main.cpp)

target_link_libraries(test_demo  ${Boost_LIBRARIES})

install(TARGETS test_demo RUNTIME DESTINATION .)
install(CODE
"include(BundleUtilities)
fixup_bundle(${CMAKE_INSTALL_PREFIX}/test_demo \"\" \"\")")
