cmake_minimum_required(VERSION 3.10)

project(goap VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

add_library(
    goap INTERFACE
)

enable_testing()
add_subdirectory(tests)