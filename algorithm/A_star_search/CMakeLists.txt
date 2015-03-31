cmake_minimum_required(VERSION 2.8)
project(test)

# c++ 11支持
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")

# openscenegraph库
# find_package(OpenSceneGraph COMPONENTS osgUtil osgDB osgText osgViewer osgGA osgManipulator REQUIRED)
# include_directories(${OPENSCENEGRAPH_INCLUDE_DIR})

file( GLOB header_files *.h )
file( GLOB source_files *.cpp)

add_executable(test
  ${source_files}
  ${header_files}
)

# target_link_libraries(test  #
# ${OPENSCENEGRAPH_LIBRARIES} #
# )                           #
