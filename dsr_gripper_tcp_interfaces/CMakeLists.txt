cmake_minimum_required(VERSION 3.8)
project(dsr_gripper_tcp_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperState.msg"
  "srv/GetMotionProfile.srv"
  "srv/GetPosition.srv"
  "srv/GetState.srv"
  "srv/SetMotionProfile.srv"
  "srv/SetPosition.srv"
  "srv/SetTorque.srv"
  "action/SafeGrasp.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
