#cmake_minimum_required(VERSION 2.8)

project(rtdb)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/rtdb)

add_subdirectory(rtdb)
add_subdirectory(comm)
add_subdirectory(example)
