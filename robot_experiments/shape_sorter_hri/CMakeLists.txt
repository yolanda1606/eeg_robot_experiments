cmake_minimum_required(VERSION 3.4)
project(shape_sorter_hri)
set(CMAKE_CXX_STANDARD 14)
# THE MAGIC LINE: Tell CMake to use our custom driver, NOT the ROS 2 one!
set(Franka_DIR "/home/sysgen/Projects/Yolanda/libfranka/build")

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(shape_sorter src/main.cpp)
target_link_libraries(shape_sorter Franka::Franka Eigen3::Eigen)