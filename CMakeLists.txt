cmake_minimum_required(VERSION 3.10)

project(goap VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(
    goap INTERFACE
)

target_include_directories(
    goap INTERFACE include
)

enable_testing()
add_subdirectory(tests)