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

#ifndef NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_THROUGH_POSES_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_THROUGH_POSES_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_bt_navigator/navigator.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace nav2_bt_navigator
{

/**
 * @class NavigateThroughPosesNavigator
 * @brief A navigator for navigating to a a bunch of intermediary poses
 */
class NavigateThroughPosesNavigator
  : public nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateThroughPoses>
{
public:
  using ActionT = nav2_msgs::action::NavigateThroughPoses;
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  /**
   * @brief A constructor for NavigateThroughPosesNavigator
   */
  NavigateThroughPosesNavigator()
  : Navigator() {}

  /**
   * @brief A configure state transition to configure navigator's state
   * @param node Weakptr to the lifecycle node
   * 中文：Lifecycle node 的 WeakPtr。
   * @param odom_smoother Object to get current smoothed robot's speed
   * 中文：用于获取机器人当前平滑速度的对象。
   */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  /**
   * @brief Get action name for this navigator
   * 中文：获取该 navigator 的 action 名称。
   * @return string Name of action server
   * 中文：string action server 名称。
   */
  std::string getName() override {return std::string("navigate_through_poses");}

  /**
   * @brief Get navigator's default BT
   * 中文：获取 navigator 的默认 BT。
   * @param node WeakPtr to the lifecycle node
   * 中文：Lifecycle node 的 WeakPtr。
   * @return string Filepath to default XML
   * 中文：string 默认 XML 文件路径。
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * 中文：BT action server 收到新目标时调用的回调。
   * Can be used to check if goal is valid and put values on
   * 中文：可用于检查目标是否有效，并向 blackboard 写入值。
   * the blackboard which depend on the received goal
   * 中文：这些 blackboard 值依赖收到的目标。
   * @param goal Action template's goal message
   * 中文：action 模板的目标消息。
   * @return bool if goal was received successfully to be processed
   * 中文：bool 目标是否已成功接收并可处理。
   */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * 中文：定义 BT 单次迭代执行内容的回调。
   * Can be used to publish action feedback
   * 中文：可用于发布 action feedback。
   */
  void onLoop() override;

  /**
   * @brief A callback that is called when a preempt is requested
   * 中文：请求抢占时调用的回调。
   */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that is called when a the action is completed, can fill in
   * 中文：action 完成时调用的回调，可填充结果。
   * action result message or indicate that this action is done.
   * 中文：action 结果消息或指示该 action 已完成。
   * @param result Action template result message to populate
   * 中文：需要填充的 action 模板结果消息。
   * @param final_bt_status Resulting status of the behavior tree execution that may be
   * 中文：行为树执行后的结果状态，可在填充结果时引用。
   * referenced while populating the result.
   * 中文：填充结果时可引用该状态。
   */
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  /**
   * @brief Goal pose initialization on the blackboard
   * 中文：在 blackboard 上初始化目标位姿。
   */
  void initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;
  std::string goals_blackboard_id_;
  std::string path_blackboard_id_;

  // Odometry smoother object
  // 中文：里程计平滑器对象。
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATORS__NAVIGATE_THROUGH_POSES_HPP_
