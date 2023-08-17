cmake_minimum_required(VERSION 3.22)
project(ball_pivoting_algorithm_cppimpl)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg") # For debug

find_package(Open3D REQUIRED)

add_executable(
  ball_pivoting_algorithm
  main.cpp)
target_link_libraries(ball_pivoting_algorithm
  ${Open3D_LIBRARIES})
