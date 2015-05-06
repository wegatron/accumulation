cmake_minimum_required(VERSION 2.8)

set( CMAKE_AUTOMOC ON )
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC ${CMAKE_CXX_FLAGS}")

find_package( Qt4 REQUIRED QtCore QtGui)

include( ${QT_USE_FILE} )

aux_source_directory(./ src)

add_executable(cmenu_demo ${src})

target_link_libraries(cmenu_demo
  ${QT_LIBRARIES}
  )