cmake_minimum_required (VERSION 2.1)
project(main)

SET(CMAKE_C_COMPILER, g++)

include_directories(lib)
add_subdirectory(src bin)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
