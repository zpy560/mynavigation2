// Copyright (c) 2021 Samsung Research
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

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "nav2_bt_navigator/navigators/navigate_to_pose.hpp"
#include "spdlog_wrapper.hpp"

namespace nav2_bt_navigator
{

bool
NavigateToPoseNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("goal"));
  }

  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }

  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  // Odometry smoother object for getting current speed
  // 中文：用于获取当前速度的里程计平滑器对象。
  odom_smoother_ = odom_smoother;

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPoseNavigator::onGoalPoseReceived, this, std::placeholders::_1));
  return true;
}

std::string
NavigateToPoseNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_nav_to_pose_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    node->declare_parameter<std::string>(
      "default_nav_to_pose_bt_xml",
      pkg_share_dir +
      "/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml");
  }

  node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
NavigateToPoseNavigator::cleanup()
{
  goal_sub_.reset();
  self_client_.reset();
  return true;
}

bool
NavigateToPoseNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  initializeGoalPose(goal);

  return true;
}

void
NavigateToPoseNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr /*result*/,
  const nav2_behavior_tree::BtStatus /*final_bt_status*/)
{
}

void
NavigateToPoseNavigator::onLoop()
{
  // action server feedback (pose, duration of task,
  // 中文：action server feedback，包括位姿、任务耗时、
  // number of recoveries, and distance remaining to goal)
  // 中文：恢复次数和到目标的剩余距离。
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);

  auto blackboard = bt_action_server_->getBlackboard();

  try {
    // Get current path points
    // 中文：获取当前路径点。
    nav_msgs::msg::Path current_path;
    blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path);

    // Find the closest pose to current pose on global path
    // 中文：在全局路径上查找距离当前位姿最近的位姿。
    auto find_closest_pose_idx =
      [&current_pose, &current_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            current_pose, current_path.poses[curr_idx]);
          if (curr_dist < curr_min_dist) {
            curr_min_dist = curr_dist;
            closest_pose_idx = curr_idx;
          }
        }
        return closest_pose_idx;
      };

    // Calculate distance on the path
    // 中文：计算路径上的距离。
    double distance_remaining =
      nav2_util::geometry_utils::calculate_path_length(current_path, find_closest_pose_idx());

    // Default value for time remaining
    // 中文：剩余时间默认值。
    rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

    // Get current speed
    // 中文：获取当前速度。
    geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
    double current_linear_speed = std::hypot(current_odom.linear.x, current_odom.linear.y);

    // Calculate estimated time taken to goal if speed is higher than 1cm/s
    // 中文：如果速度高于 1cm/s，则估算到达目标所需时间。
    // and at least 10cm to go
    // 中文：并且剩余距离至少为 10cm。
    if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
      estimated_time_remaining =
        rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
    }

    feedback_msg->distance_remaining = distance_remaining;
    feedback_msg->estimated_time_remaining = estimated_time_remaining;
  } catch (...) {
    // Ignore
    // 中文：忽略。
  }

  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->current_pose = current_pose;
  feedback_msg->navigation_time = clock_->now() - start_time_;

  bt_action_server_->publishFeedback(feedback_msg);
}

void
NavigateToPoseNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  LOG_INFO("Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // 中文：如果待处理目标请求的 BT 与当前目标相同，则接受该待处理目标。
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // 中文：如果待处理目标的 behavior_tree 字段为空，则表示请求默认 BT 文件。
    // accept the pending goal if the current goal is running the default BT file
    // 中文：如果当前目标正在运行默认 BT 文件，则接受待处理目标。
    initializeGoalPose(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

void
NavigateToPoseNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
{
  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);

  LOG_INFO("Begin navigating from current location ({:.2f}, {:.2f}) to ({:.2f}, {:.2f})", current_pose.pose.position.x, current_pose.pose.position.y, goal->pose.pose.position.x, goal->pose.pose.position.y);

  // Reset state for new action feedback
  // 中文：为新的 action feedback 重置状态。
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  // 中文：更新 blackboard 上的目标位姿。
  blackboard->set<geometry_msgs::msg::PoseStamped>(goal_blackboard_id_, goal->pose);
}

void
NavigateToPoseNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  ActionT::Goal goal;
  goal.pose = *pose;
  self_client_->async_send_goal(goal);
}

}  // namespace nav2_bt_navigator
