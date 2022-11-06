project(processing_src)

include(GNUInstallDirs)

add_library(processing SHARED src/lidar_processing_pipeline.cpp)

target_include_directories(processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(processing Eigen3::Eigen)
target_link_libraries(processing ${PCL_LIBRARIES})

add_library(PROCESSING::processing ALIAS processing)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)

install(TARGETS processing
  EXPORT PROCESSINGConfig
  LIBRARY DESTINATION lib
)

export(TARGETS processing
  NAMESPACE PROCESSING::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/PROCESSINGConfig.cmake"
)

install(EXPORT PROCESSINGConfig
  DESTINATION "${CMAKE_INSTALL_DATADIR}/processing/cmake"
  NAMESPACE PROCESSING::
)