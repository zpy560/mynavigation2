cmake_minimum_required(VERSION 3.8)
project(laserpub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(laserpub src/laserpub_node.cpp)
ament_target_dependencies(laserpub
  rclcpp
  sensor_msgs
)

install(TARGETS laserpub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
