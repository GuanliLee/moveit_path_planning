cmake_minimum_required(VERSION 3.16)
project(path_planning_server)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(path_planning_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(path_planning_server_node
  src/main.cpp
  src/path_planning_node.cpp
  src/static_scene_loader.cpp
)

target_include_directories(path_planning_server_node PRIVATE
  include
)

ament_target_dependencies(path_planning_server_node
  ament_index_cpp
  control_msgs
  geometric_shapes
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  path_planning_interfaces
  rclcpp
  rclcpp_action
  shape_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  yaml_cpp_vendor
)
target_link_libraries(path_planning_server_node yaml-cpp::yaml-cpp)

install(TARGETS path_planning_server_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/dual_joint_state_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dual_joint_state_adapter
)
install(PROGRAMS
  scripts/grasp_bridge_state_machine.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME grasp_bridge_state_machine
)
install(FILES
  scripts/capture_sync.py
  scripts/handeye_transform.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config generated launch meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_dual_joint_state_adapter
    test/test_dual_joint_state_adapter.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(
    test_scene_001_geometry
    test/test_scene_001_geometry.py
  )
  ament_add_pytest_test(
    test_dual_piper_urdf
    test/test_dual_piper_urdf.py
  )
  ament_add_pytest_test(
    test_gripper_urdf
    test/test_gripper_urdf.py
  )
  ament_add_pytest_test(
    test_plan_to_pose_interface
    test/test_plan_to_pose_interface.py
  )
  ament_add_pytest_test(
    test_capture_sync
    test/test_capture_sync.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(
    test_handeye_transform
    test/test_handeye_transform.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
  ament_add_pytest_test(
    test_grasp_bridge_state_machine
    test/test_grasp_bridge_state_machine.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
endif()

ament_package()
