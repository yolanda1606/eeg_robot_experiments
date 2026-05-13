cmake_minimum_required(VERSION 3.4)
project(pick_and_place_cpp)

# Revert back to C++14
set(CMAKE_CXX_STANDARD 14)

# THE MAGIC LINE: Tell CMake to use our custom driver, NOT the ROS 2 one!
set(Franka_DIR "/home/sysgen/Projects/Yolanda/libfranka/build")

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(pick_place src/main.cpp)
target_link_libraries(pick_place Franka::Franka Eigen3::Eigen)

# --- NEW: Your pose reader tool ---
add_executable(read_pose src/read_pose.cpp)
target_link_libraries(read_pose Franka::Franka)