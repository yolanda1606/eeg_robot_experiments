cmake_minimum_required(VERSION 3.4)
project(stack_cpp)

set(CMAKE_CXX_STANDARD 14)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(stack src/main.cpp)
target_link_libraries(stack Franka::Franka Eigen3::Eigen)