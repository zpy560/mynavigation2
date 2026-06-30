// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_WAYPOINT_HPP_
#pragma once

#include <string>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

namespace nav2_waypoint_follower
{

/**
 * @brief Simple plugin based on WaypointTaskExecutor, lets robot to wait for a
  * 中文：基于 WaypointTaskExecutor 的简单插件，让机器人等待一段时间。
 *        user input at waypoint arrival.
 */
class InputAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
/**
 * @brief Construct a new Input At Waypoint Arrival object
  * 中文：构造新的 InputAtWaypointArrival 对象。
 *
 */
  InputAtWaypoint();

  /**
   * @brief Destroy the Input At Waypoint Arrival object
   * 中文：销毁 InputAtWaypointArrival 对象。
   *
   */
  ~InputAtWaypoint();

  /**
   * @brief declares and loads parameters used
   * 中文：声明并加载使用到的参数。
   * @param parent parent node
   * 中文：父节点。
   * @param plugin_name name of plugin
   * 中文：插件名称。
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);

  /**
   * @brief Processor
   * 中文：处理器。
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return if task execution failed
   * 中文：任务执行失败时返回 false。
   */
  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

protected:
  /**
   * @brief Processor callback
   * 中文：处理器回调。
   * @param msg empty message
   */
  void Cb(const std_msgs::msg::Empty::SharedPtr msg);

  bool input_received_;
  bool is_enabled_;
  rclcpp::Duration timeout_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
  rclcpp::Clock::SharedPtr clock_;
  std::mutex mutex_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
};

}  // namespace nav2_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_WAYPOINT_HPP_
