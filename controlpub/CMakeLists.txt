cmake_minimum_required(VERSION 3.8)
project(controlpub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(byd_custom_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog_wrapper REQUIRED)

add_executable(controlpub_node src/controlpub_node.cpp)
ament_target_dependencies(controlpub_node
  byd_custom_msgs
  geometry_msgs
  rclcpp
  spdlog_wrapper
)

install(TARGETS controlpub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
