PROJECT(zsw_log)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -Wall")

set(Boost_USE_STATIC_LIBS        ON)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
# find_package( Boost COMPONENTS filesystem system unit_test_framework REQUIRED)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
else(Boost_FOUND)
  message("CAN NOT FIND BOOST")
endif(Boost_FOUND)
# link   ${Boost_LIBRARIES}  Boost_UNIT_TEST_FRAMEWORK_LIBRARY



SET(APPNAME test)

AUX_SOURCE_DIRECTORY(. test_src)

ADD_EXECUTABLE(${APPNAME} ${test_src})

TARGET_LINK_LIBRARIES( ${APPNAME}

  )