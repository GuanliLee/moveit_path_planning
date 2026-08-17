cmake_minimum_required(VERSION 3.8)
project(path_planning_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanToJoints.srv"
  "srv/PlanToPose.srv"
  DEPENDENCIES geometry_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
