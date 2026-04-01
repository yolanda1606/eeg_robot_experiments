cmake_minimum_required(VERSION 3.4)
project(shape_sorter_cpp)

set(CMAKE_CXX_STANDARD 14)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(shape_sorter src/main.cpp)
target_link_libraries(shape_sorter Franka::Franka Eigen3::Eigen)