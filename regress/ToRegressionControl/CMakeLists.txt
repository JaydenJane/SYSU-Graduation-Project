cmake_minimum_required (VERSION 2.1)
project(main)

SET(CMAKE_C_COMPILER g++)
include_directories(lib VLib/include CLib)
link_directories(/usr/lib/x86_64-linux-gnu)
add_subdirectory(VLib bin)
add_subdirectory(lib bin)
add_subdirectory(CLib bin)

aux_source_directory(. DIR_SRCS)
add_executable(main ${DIR_SRCS})
target_link_libraries(main apiLib vLib cLib pthread)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
