cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC ${CMAKE_CXX_FLAGS}")
set( CMAKE_AUTOMOC ON )
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package( Qt4 REQUIRED QtCore QtGui QtOpenGL)

include( ${QT_USE_FILE} )

file (GLOB header *.h)
file (GLOB src *.cpp)

add_executable(basic ${header} ${src})

target_link_libraries(basic
  ${QT_LIBRARIES}
  )