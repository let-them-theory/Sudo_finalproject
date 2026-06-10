cmake_minimum_required(VERSION 3.8)
project(e0509_gripper_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(dsr_msgs2 REQUIRED)

# Include directories
include_directories(include)

# Gripper Service Node (C++)
add_executable(gripper_service_node src/gripper_service_node.cpp)
ament_target_dependencies(gripper_service_node
  rclcpp std_msgs std_srvs dsr_msgs2)

# Gripper Joint Publisher (C++)
add_executable(gripper_joint_publisher src/gripper_joint_publisher.cpp)
ament_target_dependencies(gripper_joint_publisher
  rclcpp std_msgs sensor_msgs control_msgs)

# Gazebo Bridge (C++)
add_executable(gazebo_bridge src/gazebo_bridge.cpp)
ament_target_dependencies(gazebo_bridge
  rclcpp sensor_msgs trajectory_msgs std_msgs)

# Install C++ executables
install(TARGETS
  gripper_service_node
  gripper_joint_publisher
  gazebo_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  urdf
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts (keep for backwards compatibility / non-realtime scripts)
install(PROGRAMS
  scripts/gripper.py
  scripts/digital_twin_bridge.py
  scripts/robot_slider_control.py
  scripts/curobo_planner_node.py
  scripts/marker_tracking_node.py
  scripts/object_tracking_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
