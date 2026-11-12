# 构建结构：协调器进程、固定路径插件、诊断和边界可视化分别生成目标。
# 头文件、Launch 和资源安装到各自前缀；BUILD_TESTING 控制回归目标，不代表测试在构建时自动运行。
cmake_minimum_required(VERSION 3.8)
project(nav2_regulated_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖在配置阶段发现；这里的 REQUIRED 失败是构建环境问题，不是运行时导航结果。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(byd_custom_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(spdlog_wrapper REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_rotation_shim_controller REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_smac_planner REQUIRED)

# 协调器进程将按职责拆分的 navigator_* 文件链接在一起，共享同一 RegulatedNavigator 实例状态。
add_executable(regulated_navigator_node
  src/regulated_navigator_node.cpp
  src/regulated_navigator.cpp
  src/tf_diagnostics.cpp
  src/chassis_control_subscriber.cpp
  src/motion_state_subscriber.cpp
  src/navigator_goal_handler.cpp
  src/navigator_planning.cpp
  src/navigator_fixed_path.cpp
  src/navigator_autonomous_fixed.cpp
  src/navigator_control.cpp
  src/navigator_monitor.cpp
  src/navigator_task_manager.cpp
  src/navigation_utils.cpp
  src/planning_module.cpp
  src/control_module.cpp)
target_include_directories(regulated_navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(regulated_navigator_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(regulated_navigator_node
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  nav2_msgs
  nav2_util
  std_msgs
  nav_msgs
  byd_custom_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  spdlog_wrapper
  visualization_msgs
)

# Controller 与 GoalChecker 编译为同一插件共享库，由 pluginlib 根据 XML 导出声明加载。
add_library(nav2_regulated_modules_fixed_path_controller SHARED
  src/fixed_path_controller.cpp
  src/fixed_path_goal_checker.cpp)
target_include_directories(nav2_regulated_modules_fixed_path_controller PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(nav2_regulated_modules_fixed_path_controller PUBLIC cxx_std_17)
ament_target_dependencies(nav2_regulated_modules_fixed_path_controller rclcpp rclcpp_lifecycle geometry_msgs nav2_core nav2_costmap_2d nav2_util nav_msgs pluginlib tf2 tf2_ros tf2_geometry_msgs spdlog_wrapper)

add_library(nav2_regulated_modules_autonomous_controller SHARED src/autonomous_mppi_controller.cpp)
target_include_directories(nav2_regulated_modules_autonomous_controller PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(nav2_regulated_modules_autonomous_controller PUBLIC cxx_std_17)
ament_target_dependencies(nav2_regulated_modules_autonomous_controller nav2_rotation_shim_controller nav2_controller nav2_core nav2_costmap_2d nav2_util geometry_msgs nav_msgs rclcpp rclcpp_lifecycle pluginlib tf2 tf2_ros)

add_library(nav2_regulated_modules_autonomous_fixed_controller SHARED src/autonomous_fixed_controller.cpp)
target_include_directories(nav2_regulated_modules_autonomous_fixed_controller PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(nav2_regulated_modules_autonomous_fixed_controller PUBLIC cxx_std_17)
ament_target_dependencies(nav2_regulated_modules_autonomous_fixed_controller nav2_core nav2_costmap_2d nav2_util geometry_msgs nav_msgs std_msgs byd_custom_msgs rclcpp rclcpp_lifecycle pluginlib tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(nav2_regulated_modules_autonomous_fixed_controller nav2_regulated_modules_fixed_path_controller)
install(TARGETS nav2_regulated_modules_autonomous_fixed_controller LIBRARY DESTINATION lib)
install(FILES include/nav2_regulated_modules/autonomous_fixed_controller.hpp DESTINATION include/nav2_regulated_modules)
install(FILES include/nav2_regulated_modules/fixed_path_controller.hpp DESTINATION include/nav2_regulated_modules)
install(FILES include/nav2_regulated_modules/tf_diagnostics.hpp DESTINATION include/nav2_regulated_modules)
install(FILES include/nav2_regulated_modules/detail/tf_timing.hpp DESTINATION include/nav2_regulated_modules/detail)
install(FILES include/nav2_regulated_modules/detail/fixed_path_start_speed_guard.hpp DESTINATION include/nav2_regulated_modules/detail)
install(FILES include/nav2_regulated_modules/detail/fixed_path_progress.hpp DESTINATION include/nav2_regulated_modules/detail)
install(FILES include/nav2_regulated_modules/detail/autonomous_fixed_path.hpp DESTINATION include/nav2_regulated_modules/detail)
install(FILES include/nav2_regulated_modules/detail/autonomous_fixed_localization.hpp DESTINATION include/nav2_regulated_modules/detail)
install(FILES include/nav2_regulated_modules/detail/autonomous_fixed_terminal_review.hpp DESTINATION include/nav2_regulated_modules/detail)
install(FILES include/nav2_regulated_modules/detail/autonomous_fixed_terminal_trim.hpp DESTINATION include/nav2_regulated_modules/detail)
pluginlib_export_plugin_description_file(nav2_core autonomous_fixed_controller_plugin.xml)

# 仅第四模式装载此包装；复用一次 Hybrid A* 后，在冻结前校正格心偏差。
add_library(nav2_regulated_modules_autonomous_fixed_planner SHARED src/autonomous_fixed_hybrid_planner.cpp)
target_compile_features(nav2_regulated_modules_autonomous_fixed_planner PUBLIC cxx_std_17)
ament_target_dependencies(nav2_regulated_modules_autonomous_fixed_planner nav2_smac_planner nav2_core nav2_costmap_2d nav2_util geometry_msgs nav_msgs rclcpp rclcpp_lifecycle pluginlib tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS nav2_regulated_modules_autonomous_fixed_planner LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(nav2_core autonomous_fixed_planner_plugin.xml)

# 诊断进程独立于控制周期；边界可视化也只发布显示消息，不进入速度计算。
add_executable(velocity_diagnostics_node src/velocity_diagnostics.cpp)
target_compile_features(velocity_diagnostics_node PUBLIC cxx_std_17)
ament_target_dependencies(velocity_diagnostics_node rclcpp geometry_msgs nav_msgs byd_custom_msgs spdlog_wrapper)

add_executable(collision_boundary_visualizer_node src/collision_boundary_visualizer.cpp)
target_compile_features(collision_boundary_visualizer_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(collision_boundary_visualizer_node rclcpp geometry_msgs visualization_msgs)

install(TARGETS nav2_regulated_modules_fixed_path_controller nav2_regulated_modules_autonomous_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS regulated_navigator_node collision_boundary_visualizer_node velocity_diagnostics_node
  DESTINATION lib/${PROJECT_NAME})

# 新增自主头文件使用已导出的 include 根目录；旧固定路径安装布局保持。
install(FILES include/nav2_regulated_modules/autonomous_mppi_controller.hpp DESTINATION include/nav2_regulated_modules)
install(FILES include/nav2_regulated_modules/detail/autonomous_navigation.hpp DESTINATION include/nav2_regulated_modules/detail)

# 安装公开头文件及运行资源；symlink-install 下部分安装路径依赖 build 或源目录，不可直接整目录清理。
install(DIRECTORY
  include
  DESTINATION include
)

install(DIRECTORY
  launch
  maps
  params
  rviz
  src
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core fixed_path_controller_plugin.xml)
ament_export_include_directories(include)
ament_export_libraries(nav2_regulated_modules_fixed_path_controller)
ament_export_dependencies(nav2_core nav2_costmap_2d pluginlib nav2_controller nav2_rotation_shim_controller tf2_msgs)

include(CTest)
# 将 collision 测试客户端安装为 ros2 run 可执行脚本；安装不代表它会随 Launch 自动运行。
install(PROGRAMS tools/collision_action_test.py DESTINATION lib/${PROJECT_NAME})
# 以下注册纯算法／Launch／窗口测试；CTest 执行与 colcon 构建是两个步骤，未执行不能宣称测试通过。
if(BUILD_TESTING)
  add_executable(autonomous_fixed_localization_test test/autonomous_fixed_localization_test.cpp)
  target_include_directories(autonomous_fixed_localization_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(autonomous_fixed_localization_test PRIVATE cxx_std_17)
  add_test(NAME autonomous_fixed_localization_test COMMAND autonomous_fixed_localization_test)
  add_executable(autonomous_fixed_terminal_review_test test/autonomous_fixed_terminal_review_test.cpp)
  target_include_directories(autonomous_fixed_terminal_review_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(autonomous_fixed_terminal_review_test PRIVATE cxx_std_17)
  add_test(NAME autonomous_fixed_terminal_review_test COMMAND autonomous_fixed_terminal_review_test)
  add_executable(autonomous_fixed_controller_test test/autonomous_fixed_controller_test.cpp)
  target_include_directories(autonomous_fixed_controller_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(autonomous_fixed_controller_test PRIVATE cxx_std_17)
  ament_target_dependencies(autonomous_fixed_controller_test nav2_core nav2_controller nav2_costmap_2d nav2_util geometry_msgs nav_msgs std_msgs byd_custom_msgs rclcpp rclcpp_lifecycle tf2 tf2_ros tf2_geometry_msgs)
  target_link_libraries(autonomous_fixed_controller_test nav2_regulated_modules_autonomous_fixed_controller)
  add_test(NAME autonomous_fixed_controller_test COMMAND autonomous_fixed_controller_test)
  set_tests_properties(autonomous_fixed_controller_test PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=194;ROS_LOCALHOST_ONLY=1" TIMEOUT 30)
  add_test(NAME autonomous_fixed_motion_state_controller_test COMMAND autonomous_fixed_controller_test --motion-state)
  set_tests_properties(autonomous_fixed_motion_state_controller_test PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=194;ROS_LOCALHOST_ONLY=1" TIMEOUT 30)
  add_executable(autonomous_fixed_path_test test/autonomous_fixed_path_test.cpp)
  target_include_directories(autonomous_fixed_path_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(autonomous_fixed_path_test PRIVATE cxx_std_17)
  ament_target_dependencies(autonomous_fixed_path_test nav_msgs tf2 tf2_geometry_msgs)
  add_test(NAME autonomous_fixed_path_test COMMAND autonomous_fixed_path_test)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME collision_action_tool_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_collision_action_tool.py)
  add_test(NAME logging_routing_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_logging_routing.py)
  add_test(NAME velocity_logging_launch_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/velocity_logging_launch_test.py)
  add_test(NAME collision_launch_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_collision_launch.py)
  add_test(NAME autonomous_launch_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autonomous_launch.py)
  add_test(NAME autonomous_fixed_launch_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autonomous_fixed_launch.py)
  add_executable(autonomous_navigation_test test/autonomous_navigation_test.cpp)
  target_include_directories(autonomous_navigation_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(autonomous_navigation_test PRIVATE cxx_std_17)
  add_test(NAME autonomous_navigation_test COMMAND autonomous_navigation_test)
  add_executable(autonomous_terminal_collision_test test/autonomous_terminal_collision_test.cpp)
  target_include_directories(autonomous_terminal_collision_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(autonomous_terminal_collision_test PRIVATE cxx_std_17)
  ament_target_dependencies(autonomous_terminal_collision_test nav2_costmap_2d)
  add_test(NAME autonomous_terminal_collision_test COMMAND autonomous_terminal_collision_test)
  add_executable(autonomous_tf_timing_test test/autonomous_tf_timing_test.cpp)
  target_include_directories(autonomous_tf_timing_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(autonomous_tf_timing_test PRIVATE cxx_std_17)
  ament_target_dependencies(autonomous_tf_timing_test nav2_controller nav2_rotation_shim_controller nav2_core rclcpp rclcpp_lifecycle tf2 tf2_ros nav_msgs geometry_msgs)
  target_link_libraries(autonomous_tf_timing_test nav2_regulated_modules_autonomous_controller)
  add_test(NAME autonomous_tf_timing_test COMMAND autonomous_tf_timing_test)
  add_executable(terminal_convergence_test test/terminal_convergence_test.cpp)
  target_include_directories(terminal_convergence_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(terminal_convergence_test nav2_core)
  add_test(NAME terminal_convergence_test COMMAND terminal_convergence_test)
  add_executable(braking_margin_test test/braking_margin_test.cpp)
  target_include_directories(braking_margin_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(braking_margin_test PRIVATE cxx_std_17)
  add_test(NAME braking_margin_test COMMAND braking_margin_test)
  add_executable(terminal_position_test test/terminal_position_test.cpp)
  target_include_directories(terminal_position_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(terminal_position_test PRIVATE cxx_std_17)
  add_test(NAME terminal_position_test COMMAND terminal_position_test)
  add_executable(path_sample_test test/path_sample_test.cpp)
  target_include_directories(path_sample_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(path_sample_test PRIVATE cxx_std_17)
  add_test(NAME path_sample_test COMMAND path_sample_test)
  add_executable(fixed_path_progress_test test/fixed_path_progress_test.cpp)
  target_include_directories(fixed_path_progress_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(fixed_path_progress_test PRIVATE cxx_std_17)
  add_test(NAME fixed_path_progress_test COMMAND fixed_path_progress_test)
  add_executable(fixed_path_start_speed_guard_test test/fixed_path_start_speed_guard_test.cpp)
  target_include_directories(fixed_path_start_speed_guard_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(fixed_path_start_speed_guard_test PRIVATE cxx_std_17)
  add_test(NAME fixed_path_start_speed_guard_test COMMAND fixed_path_start_speed_guard_test)
endif()

pluginlib_export_plugin_description_file(nav2_core autonomous_controller_plugin.xml)

ament_package()
