project(test)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -rdynamic")

set(tests test_error_ctrl)

foreach(tmp_test ${tests})
  add_executable(${tmp_test} ${tmp_test}.cpp)
endforeach(tmp_test)

