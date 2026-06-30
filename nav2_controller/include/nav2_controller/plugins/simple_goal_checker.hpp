/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace nav2_controller
{

/**
 * @class SimpleGoalChecker
 * @brief Goal Checker plugin that only checks the position difference
  * 中文：只检查位置偏差的 GoalChecker 插件。
 *
 * This class can be stateful if the stateful parameter is set to true (which it is by default).
  * 中文：如果 stateful 参数为 true（默认值），该类可以保持状态。
 * This means that the goal checker will not check if the xy position matches again once it is found to be true.
  * 中文：这意味着一旦 xy 位置满足条件，goal checker 后续不会再次检查 xy 是否匹配。
 */
class SimpleGoalChecker : public nav2_core::GoalChecker
{
public:
  SimpleGoalChecker();
  // Standard GoalChecker Interface
  // 中文：标准 GoalChecker 接口。
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool stateful_, check_xy_;
  bool symmetric_yaw_tolerance_;
  // Cached squared xy_goal_tolerance_
  // 中文：缓存的 xy_goal_tolerance_ 平方值。
  double xy_goal_tolerance_sq_;
  // Dynamic parameters handler
  // 中文：动态参数处理器。
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   * 中文：检测到参数变化时执行的回调。
   * @param parameters list of changed parameters
   * 中文：已变化的参数列表。
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__SIMPLE_GOAL_CHECKER_HPP_
