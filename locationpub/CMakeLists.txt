cmake_minimum_required(VERSION 3.8)
project(locationpub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(locationpub src/locationpub_node.cpp)
ament_target_dependencies(locationpub
  geometry_msgs
  rclcpp
  tf2_ros
)

install(TARGETS locationpub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
