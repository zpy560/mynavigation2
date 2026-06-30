cmake_minimum_required(VERSION 3.16)
project(spdlog_wrapper)

# 设置C和C++标准
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)


add_library(${PROJECT_NAME}
  SHARED
  src/spdlog_wrapper.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  spdlog::spdlog
  yaml-cpp)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  rcutils
  ament_index_cpp)


install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# 生成并安装 CMake 导出文件
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rcutils spdlog yaml-cpp)

ament_package()