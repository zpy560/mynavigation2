cmake_minimum_required(VERSION 3.5)
project(byd_custom_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisControl.msg"
  "msg/ControlRes.msg"
  "msg/MotionState.msg"
  "msg/NaviSegment.msg"
  "action/FollowFixedPath.action"
  "action/NavigationService.action"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
