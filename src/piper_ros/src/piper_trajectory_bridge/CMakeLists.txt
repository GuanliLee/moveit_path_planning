cmake_minimum_required(VERSION 3.16)
project(piper_trajectory_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(piper_trajectory_bridge_core
  src/trajectory_bridge.cpp
)
target_include_directories(piper_trajectory_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(piper_trajectory_bridge_core
  builtin_interfaces
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

add_executable(piper_trajectory_bridge
  src/main.cpp
)
target_link_libraries(piper_trajectory_bridge
  piper_trajectory_bridge_core
)
ament_target_dependencies(piper_trajectory_bridge
  rclcpp
)

install(TARGETS
  piper_trajectory_bridge_core
  piper_trajectory_bridge
  EXPORT export_piper_trajectory_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_trajectory_bridge
    test/test_trajectory_bridge.cpp
  )
  target_link_libraries(test_trajectory_bridge
    piper_trajectory_bridge_core
  )
  ament_target_dependencies(test_trajectory_bridge
    builtin_interfaces
    control_msgs
    rclcpp
    rclcpp_action
    sensor_msgs
    trajectory_msgs
  )
endif()

ament_export_targets(export_piper_trajectory_bridge HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

ament_package()
