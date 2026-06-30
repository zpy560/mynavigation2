// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
#define NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
// spdlog wrapper for module-specific logging
// 中文：用于模块专属日志的 spdlog wrapper。
#include "spdlog_wrapper.hpp"

namespace nav2_waypoint_follower
{

enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

/**
 * @class nav2_waypoint_follower::WaypointFollower
 * @brief An action server that uses behavior tree for navigating a robot to its
  * 中文：使用行为树将机器人导航到
 * goal position.
  * 中文：目标位置的 action server。
 */
class WaypointFollower : public nav2_util::LifecycleNode
{
public:
  using ActionT = nav2_msgs::action::FollowWaypoints;
  using ClientT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using ActionClient = rclcpp_action::Client<ClientT>;

  /**
   * @brief A constructor for nav2_waypoint_follower::WaypointFollower class
   * 中文：nav2_waypoint_follower::WaypointFollower 类的构造函数。
   * @param options Additional options to control creation of the node.
   * 中文：用于控制节点创建的附加选项。
   */
  explicit WaypointFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for nav2_waypoint_follower::WaypointFollower class
   * 中文：nav2_waypoint_follower::WaypointFollower 类的析构函数。
   */
  ~WaypointFollower();

protected:
  /**
   * @brief Configures member variables
   * 中文：配置成员变量。
   *
   * Initializes action server for "follow_waypoints"
   * 中文：初始化 "follow_waypoints" action server。
   * @param state Reference to LifeCycle node state
   * 中文：Lifecycle 节点状态引用。
   * @return SUCCESS or FAILURE
   * 中文：SUCCESS 或 FAILURE。
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * 中文：激活 action server。
   * @param state Reference to LifeCycle node state
   * 中文：Lifecycle 节点状态引用。
   * @return SUCCESS or FAILURE
   * 中文：SUCCESS 或 FAILURE。
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * 中文：停用 action server。
   * @param state Reference to LifeCycle node state
   * 中文：Lifecycle 节点状态引用。
   * @return SUCCESS or FAILURE
   * 中文：SUCCESS 或 FAILURE。
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
   * 中文：重置成员变量。
   * @param state Reference to LifeCycle node state
   * 中文：Lifecycle 节点状态引用。
   * @return SUCCESS or FAILURE
   * 中文：SUCCESS 或 FAILURE。
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * 中文：进入 shutdown 状态时调用。
   * @param state Reference to LifeCycle node state
   * 中文：Lifecycle 节点状态引用。
   * @return SUCCESS or FAILURE
   * 中文：SUCCESS 或 FAILURE。
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Action server callbacks
   * 中文：Action server 回调。
   */
  void followWaypoints();

  /**
   * @brief Action client result callback
   * 中文：Action client 结果回调。
   * @param result Result of action server updated asynchronously
   * 中文：异步更新的 action server 结果。
   */
  void resultCallback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult & result);

  /**
   * @brief Action client goal response callback
   * 中文：Action client 目标响应回调。
   * @param goal Response of action server updated asynchronously
   * 中文：异步更新的 action server 响应。
   */
  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr & goal);

  /**
   * @brief Callback executed when a parameter change is detected
   * 中文：检测到参数变化时执行的回调。
   * @param event ParameterEvent message
   * 中文：ParameterEvent 消息。
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  // 中文：动态参数处理器。
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Our action server
  // 中文：本 action server。
  std::unique_ptr<ActionServer> action_server_;
  ActionClient::SharedPtr nav_to_pose_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;
  bool stop_on_failure_;
  ActionStatus current_goal_status_;
  int loop_rate_;
  std::vector<int> failed_ids_;

  // Task Execution At Waypoint Plugin
  // 中文：路点任务执行插件。
  pluginlib::ClassLoader<nav2_core::WaypointTaskExecutor>
  waypoint_task_executor_loader_;
  pluginlib::UniquePtr<nav2_core::WaypointTaskExecutor>
  waypoint_task_executor_;
  std::string waypoint_task_executor_id_;
  std::string waypoint_task_executor_type_;
};

}  // namespace nav2_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
