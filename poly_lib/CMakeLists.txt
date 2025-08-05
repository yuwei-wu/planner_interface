cmake_minimum_required(VERSION 3.8)
project(poly_lib)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(traj_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add library
add_library(${PROJECT_NAME} SHARED
  src/poly_node.cpp
)

# Include directories (Add Eigen3 include dirs explicitly)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Link ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  std_msgs
  rclcpp
  traj_msgs
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Export CMake config for downstream packages
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
