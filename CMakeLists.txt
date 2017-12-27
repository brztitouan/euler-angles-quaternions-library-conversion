cmake_minimum_required(VERSION 2.8)

project(euler)

# use C++11
set(CMAKE_CXX_FLAGS "-std=c++11")

# Find Eigen
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(Eigen3 REQUIRED)

# Library
set(SOURCES
    include/euler/io.cpp
    include/euler/rotations.cpp
    )
add_library(euler_rotations ${SOURCES})
target_include_directories(euler_rotations PUBLIC 
    include
    ${EIGEN3_INCLUDE_DIR}
    )

# Application
add_executable(euler src/euler.cpp)
target_link_libraries(euler euler_rotations)

# Install
#install(TARGETS euler DESTINATION bin)
