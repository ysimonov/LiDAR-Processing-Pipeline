cmake_minimum_required(VERSION 3.6 FATAL_ERROR)
project(lidar_processing_pipeline)

# Default C Standard
set(CMAKE_C_STANDARD 11)

# Default C++ Standard
set(CMAKE_CXX_STANDARD 14)

# Compile options
add_compile_options("-O2" "-Wall" "-funroll-loops" "-lpthread")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/processing/include
    ${PROJECT_SOURCE_DIR}/processing/src
    ${PROJECT_SOURCE_DIR}/visualization/include
    ${PROJECT_SOURCE_DIR}/visualization/src
)

file(GLOB SOURCES
    # -----> LOCAL (PROCESSING) <-----
    # INCLUDES
    "${PROJECT_SOURCE_DIR}/processing/include/preprocessing.hpp"
    "${PROJECT_SOURCE_DIR}/processing/include/segmentation.hpp"
    "${PROJECT_SOURCE_DIR}/processing/include/clustering.hpp"
    "${PROJECT_SOURCE_DIR}/processing/include/polygonization.hpp"
    "${PROJECT_SOURCE_DIR}/processing/include/tracking.hpp"
    "${PROJECT_SOURCE_DIR}/processing/include/lidar_processing_pipeline.hpp"
    # SOURCES
    "${PROJECT_SOURCE_DIR}/processing/src/lidar_processing_pipeline.cpp"
    "${PROJECT_SOURCE_DIR}/main.cpp"

    # -----> LOCAL (PLOTTING) <-----
    # INCLUDES
    "${PROJECT_SOURCE_DIR}/visualization/include/plotting.hpp"
    # SOURCES
    "${PROJECT_SOURCE_DIR}/visualization/src/plotting.cpp"
)

# VTK (PLOTTING)
find_package(VTK 9.2 REQUIRED)

# EIGEN (PROCESSING)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# PCL (PROCESSING)
find_package(PCL 1.7.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Subdirectory libraries
add_subdirectory(processing)
add_subdirectory(visualization)

# Add Executable
add_executable(lidar_processing_pipeline main.cpp ${SOURCES})

# Linking
target_link_libraries(lidar_processing_pipeline PROCESSING::processing)
target_link_libraries(lidar_processing_pipeline VISUALIZATION::visualization)

# target_link_libraries(lidar_processing_pipeline Eigen3::Eigen)
# target_link_libraries(lidar_processing_pipeline ${VTK_LIBRARIES})
# target_link_libraries(lidar_processing_pipeline ${PCL_LIBRARIES})
