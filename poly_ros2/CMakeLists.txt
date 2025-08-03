cmake_minimum_required(VERSION 3.5)
project(poly_ros2 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add poly_lib as a local subdirectory
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../poly_lib ${CMAKE_BINARY_DIR}/poly_lib_build)

# Create executable from source
add_executable(poly_node src/poly_node.cpp)

# Link dependencies
ament_target_dependencies(poly_node rclcpp)
target_link_libraries(poly_node poly_lib)

# Install executable
install(TARGETS poly_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
