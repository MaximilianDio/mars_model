# CMake minimum version requirement
cmake_minimum_required(VERSION 3.12)

# Project name
project(mars_model)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# ##########
# Options
# ##########
option(PACKAGE_TESTS "Build the tests" ON)

# Add include directory
include_directories(include)

# Add library
add_library(mars_model_lib src/mars_model.cpp)

add_subdirectory(examples)

# Testing
if(PACKAGE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()