cmake_minimum_required(VERSION 3.6 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)

# Where executables will be stored
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

project(lidar_processing_pipeline)

add_subdirectory(processing/downsampling)
