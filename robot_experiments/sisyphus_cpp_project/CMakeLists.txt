cmake_minimum_required(VERSION 3.4)
project(sisyphus_hri)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find Franka and Eigen libraries
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

# This is the bulletproof way to tell C++ where the Eigen math headers are
include_directories(${EIGEN3_INCLUDE_DIR})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(sisyphus_task src/main.cpp)

# Notice we changed Eigen3::Eigen3 to Eigen3::Eigen
target_link_libraries(sisyphus_task Franka::Franka Eigen3::Eigen Threads::Threads)