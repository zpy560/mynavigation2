// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_bt_navigator/bt_navigator.hpp"

#include <memory>
#include <string>
#include <utility>
#include <set>
#include <limits>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator(rclcpp::NodeOptions options)
: nav2_util::LifecycleNode("bt_navigator", "",
    options.automatically_declare_parameters_from_overrides(true))
{
  // SpdlogWrapper::init("nav2_bt_navigator", get_name());
  LOG_INFO("Creating");
  LOG_INFO("Creating BT navigator action servers and behavior-tree plugin registry");

  // 中文注释：BT Navigator 不直接写死导航流程，而是加载行为树节点库；
  // XML 行为树中的 action、condition、control 节点都来自这些plugin
  const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_compute_and_track_route_bt_node",
    "nav2_compute_route_bt_node",
    "nav2_smooth_path_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_assisted_teleop_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_drive_on_heading_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_globally_updated_goal_condition_bt_node",
    "nav2_is_path_valid_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_truncate_path_local_action_bt_node",
    "nav2_goal_updater_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_path_expiring_timer_condition",
    "nav2_distance_traveled_condition_bt_node",
    "nav2_single_trigger_bt_node",
    "nav2_goal_updated_controller_bt_node",
    "nav2_is_battery_low_condition_bt_node",
    "nav2_navigate_through_poses_action_bt_node",
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node",
    "nav2_planner_selector_bt_node",
    "nav2_controller_selector_bt_node",
    "nav2_goal_checker_selector_bt_node",
    "nav2_controller_cancel_bt_node",
    "nav2_path_longer_on_approach_bt_node",
    "nav2_wait_cancel_bt_node",
    "nav2_spin_cancel_bt_node",
    "nav2_assisted_teleop_cancel_bt_node",
    "nav2_back_up_cancel_bt_node",
    "nav2_drive_on_heading_cancel_bt_node",
    "nav2_is_battery_charging_condition_bt_node"
  };

  declare_parameter_if_not_declared(
    this, "plugin_lib_names", rclcpp::ParameterValue(plugin_libs));
  declare_parameter_if_not_declared(
    this, "transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    this, "global_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter_if_not_declared(
    this, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter_if_not_declared(
    this, "odom_topic", rclcpp::ParameterValue(std::string("odom")));
}

BtNavigator::~BtNavigator()
{
}

nav2_util::CallbackReturn
BtNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Configuring");
  LOG_INFO("Configuring BT navigator TF, odometry and navigator plugins");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_base_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  odom_topic_ = get_parameter("odom_topic").as_string();

  // Libraries to pull plugins (BT Nodes) from
  // 中文：Libraries to pull 插件。s (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  // 中文注释：单目标点导航和多目标点导航分别由两个 navigator 实现，
  // 它们共享 TF、速度估计、插件互斥器和 BT 插件库。
  pose_navigator_ = std::make_unique<nav2_bt_navigator::NavigateToPoseNavigator>();
  poses_navigator_ = std::make_unique<nav2_bt_navigator::NavigateThroughPosesNavigator>();

  nav2_bt_navigator::FeedbackUtils feedback_utils;
  feedback_utils.tf = tf_;
  feedback_utils.global_frame = global_frame_;
  feedback_utils.robot_frame = robot_frame_;
  feedback_utils.transform_tolerance = transform_tolerance_;

  // 中文注释：反馈依赖当前机器人位姿和速度；OdomSmoother 用短时间窗口平滑里程计，
  // 避免 BT feedback 中的速度抖动直接暴露给上层。
  // Odometry smoother object for getting current speed
  // 中文：用于获取当前速度的里程计平滑器对象。
  odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(shared_from_this(), 0.3, odom_topic_);

  if (!pose_navigator_->on_configure(
      shared_from_this(), plugin_lib_names, feedback_utils, &plugin_muxer_, odom_smoother_))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!poses_navigator_->on_configure(
      shared_from_this(), plugin_lib_names, feedback_utils, &plugin_muxer_, odom_smoother_))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Activating");
  LOG_INFO("Activating NavigateToPose and NavigateThroughPoses behavior trees");

  if (!poses_navigator_->on_activate() || !pose_navigator_->on_activate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // create bond connection
  // 中文：创建 bond 连接。
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Deactivating");

  if (!poses_navigator_->on_deactivate() || !pose_navigator_->on_deactivate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  // 中文：销毁 bond 连接。
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Cleaning up");

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  if (!poses_navigator_->on_cleanup() || !pose_navigator_->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  poses_navigator_.reset();
  pose_navigator_.reset();

  LOG_INFO("Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_bt_navigator

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// 中文：将组件注册到 class_loader。
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// 中文：这相当于组件入口，使组件所在库被加载时可以被发现。
// is being loaded into a running process.
// 中文：当组件库被加载到运行中的进程时可被发现。
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_bt_navigator::BtNavigator)
