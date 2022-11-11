project(downsampling)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Executable for a publisher node
add_executable(${PROJECT_NAME} src/node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)

# Install ensures that ros2 run command can find the executable file
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)