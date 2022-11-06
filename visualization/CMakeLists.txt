project(visualization_src)

include(GNUInstallDirs)

add_library(visualization SHARED src/plotting.cpp)

target_include_directories(visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(visualization Eigen3::Eigen)
target_link_libraries(visualization ${VTK_LIBRARIES})

add_library(VISUALIZATION::visualization ALIAS visualization)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)

install(TARGETS visualization
  EXPORT VISUALIZATIONConfig
  LIBRARY DESTINATION lib
)

export(TARGETS visualization
  NAMESPACE visualization::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/VISUALIZATIONConfig.cmake"
)

install(EXPORT VISUALIZATIONConfig
  DESTINATION "${CMAKE_INSTALL_DATADIR}/visualization/cmake"
  NAMESPACE VISUALIZATION::
)