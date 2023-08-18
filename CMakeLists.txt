cmake_minimum_required(VERSION 3.22)
project(ball_pivoting_algorithm_cppimpl)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include/c++/11 -I/usr/include/x86_64-linux-gnu/c++/11") # to add standard include dir to compile_command.json
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg") # For debug


find_package(Open3D REQUIRED)

add_executable(
  ball_pivoting_algorithm
  main.cpp)

target_include_directories(
  ball_pivoting_algorithm
  PUBLIC
  /usr/include/c++/11
  /usr/include/x86_64-linux-gnu/c++/11
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(ball_pivoting_algorithm
  ${Open3D_LIBRARIES})


add_executable(
  test
  test.cpp)

target_include_directories(
  test
  PUBLIC
  /usr/include/c++/11
  /usr/include/x86_64-linux-gnu/c++/11
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(test
  ${Open3D_LIBRARIES}
  gtest
  gtest_main
  pthread)
