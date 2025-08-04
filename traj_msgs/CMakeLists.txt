cmake_minimum_required(VERSION 3.5)
project(traj_msgs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)  # For time/duration if needed

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BdDervi.msg"
  "msg/Discrete.msg"
  "msg/Polynomial.msg"
  "msg/Bspline.msg"
  "msg/MultiTraj.msg"
  "msg/SingleTraj.msg"
  "msg/PiecewisePoly.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
