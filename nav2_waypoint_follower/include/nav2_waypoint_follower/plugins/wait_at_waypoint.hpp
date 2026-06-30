// Copyright (c) 2020 Fetullah Atas
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

#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__WAIT_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__WAIT_AT_WAYPOINT_HPP_
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

namespace nav2_waypoint_follower
{

/**
 * @brief Simple plugin based on WaypointTaskExecutor, lets robot to sleep for a
  * 中文：基于 WaypointTaskExecutor 的简单插件，让机器人在路点处等待一段时间。
 *        specified amount of time at waypoint arrival. You can reference this class to define
 *        your own task and rewrite the body for it.
 *
 */
class WaitAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
/**
 * @brief Construct a new Wait At Waypoint Arrival object
  * 中文：构造新的 WaitAtWaypointArrival 对象。
 *
 */
  WaitAtWaypoint();

  /**
   * @brief Destroy the Wait At Waypoint Arrival object
   * 中文：销毁 WaitAtWaypointArrival 对象。
   *
   */
  ~WaitAtWaypoint();

  /**
   * @brief declares and loads parameters used (waypoint_pause_duration_)
   * 中文：声明并加载使用到的参数 waypoint_pause_duration_。
   *
   * @param parent parent node that plugin will be created withing(waypoint_follower in this case)
   * 中文：插件将在其中创建的父节点，此处为 waypoint_follower。
   * @param plugin_name
   * 中文：插件。_name
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);


  /**
   * @brief Override this to define the body of your task that you would like to execute once the robot arrived to waypoint
   * 中文：重写该函数以定义机器人到达路点后要执行的任务主体。
   *
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return true if task execution was successful
   * 中文：任务执行成功时返回 true。
   * @return if task execution failed
   * 中文：任务执行失败时返回 false。
   */
  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

protected:
  // the robot will sleep waypoint_pause_duration_ milliseconds
  // 中文：机器人会等待 waypoint_pause_duration_ 毫秒。
  int waypoint_pause_duration_;
  bool is_enabled_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace nav2_waypoint_follower
#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__WAIT_AT_WAYPOINT_HPP_
