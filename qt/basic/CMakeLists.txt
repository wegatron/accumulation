cmake_minimum_required(VERSION 2.8)

set( CMAKE_AUTOMOC ON )
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package( Qt4 REQUIRED QtCore QtGui QtOpenGL)

include( ${QT_USE_FILE} )

aux_source_directory(./ src)

add_executable(basic ${src})

target_link_libraries(basic
  ${QT_LIBRARIES}
  )