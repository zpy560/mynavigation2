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

#include "nav2_waypoint_follower/waypoint_follower.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

namespace nav2_waypoint_follower
{

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

WaypointFollower::WaypointFollower(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("waypoint_follower", "", options),
  waypoint_task_executor_loader_("nav2_waypoint_follower",
    "nav2_core::WaypointTaskExecutor")
{
  // SpdlogWrapper::init("nav2_waypoint_follower", get_name());
  LOG_INFO("Creating");
  LOG_INFO("Waypoint follower sequences NavigateToPose goals and waypoint task plugins");

  // 中文注释：WaypointFollower 不直接控制底盘；它把路点列表拆成多个
  // NavigateToPose goal，逐个交给 bt_navigator，并在每个路点到达后执行任务plugin
  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);
  nav2_util::declare_parameter_if_not_declared(
    this, std::string("waypoint_task_executor_plugin"),
    rclcpp::ParameterValue(std::string("wait_at_waypoint")));
  nav2_util::declare_parameter_if_not_declared(
    this, std::string("wait_at_waypoint.plugin"),
    rclcpp::ParameterValue(std::string("nav2_waypoint_follower::WaitAtWaypoint")));
}

WaypointFollower::~WaypointFollower()
{
}

nav2_util::CallbackReturn
WaypointFollower::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Configuring");
  LOG_INFO("Configuring waypoint action server, NavigateToPose client and task executor");

  auto node = shared_from_this();

  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();
  waypoint_task_executor_id_ = get_parameter("waypoint_task_executor_plugin").as_string();
  LOG_INFO(
    "Waypoint follower parameters stop_on_failure={}, loop_rate={}, task_executor={}",
    stop_on_failure_, loop_rate_, waypoint_task_executor_id_.c_str());

  // 中文注释：NavigateToPose action client 放在独立 callback group 中，
  // 主循环可同步等待导航结果，同时不阻塞 follow_waypoints action server
  callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, get_node_base_interface());

  nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "navigate_to_pose", callback_group_);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "follow_waypoints", std::bind(&WaypointFollower::followWaypoints, this));

  try {
    // 中文注释：任务插件定义“到达路点后做什么”，默认 wait_at_waypoint；
    // 例如等待、拍照、外部输入确认都走同一个 WaypointTaskExecutor 接口。
    waypoint_task_executor_type_ = nav2_util::get_plugin_type_param(
      this,
      waypoint_task_executor_id_);
    waypoint_task_executor_ = waypoint_task_executor_loader_.createUniqueInstance(
      waypoint_task_executor_type_);
    LOG_INFO("Created waypoint_task_executor : {} of type {}", waypoint_task_executor_id_.c_str(), waypoint_task_executor_type_.c_str());
    waypoint_task_executor_->initialize(node, waypoint_task_executor_id_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create waypoint_task_executor. Exception: %s", ex.what());
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Activating");
  LOG_INFO("Activating waypoint follower action server");

  action_server_->activate();

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  // 中文：添加动态参数回调。
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&WaypointFollower::dynamicParametersCallback, this, _1));

  // create bond connection
  // 中文：创建 bond 连接。
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Deactivating");

  action_server_->deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  // 中文：销毁 bond 连接。
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Cleaning up");

  action_server_.reset();
  nav_to_pose_client_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
WaypointFollower::followWaypoints()
{
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  // Check if request is valid
  // 中文：检查请求是否有效。
  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
    return;
  }

  LOG_INFO("Received follow waypoint request with {} waypoints.", static_cast<int>(goal->poses.size()));

  if (goal->poses.size() == 0) {
    LOG_INFO("FollowWaypoints request is empty, returning success immediately");
    action_server_->succeeded_current(result);
    return;
  }

  rclcpp::WallRate r(loop_rate_);
  uint32_t goal_index = 0;
  bool new_goal = true;

  while (rclcpp::ok()) {
    // Check if asked to stop processing action
    // 中文：检查是否请求停止处理 action。
    if (action_server_->is_cancel_requested()) {
      auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
      callback_group_executor_.spin_until_future_complete(cancel_future);
      // for result callback processing
      // 中文：用于处理结果回调。
      callback_group_executor_.spin_some();
      action_server_->terminate_all();
      return;
    }

    // Check if asked to process another action
    // 中文：检查是否请求处理另一个 action。
    if (action_server_->is_preempt_requested()) {
      LOG_INFO("Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
      goal_index = 0;
      new_goal = true;
    }

    // Check if we need to send a new goal
    // 中文：检查是否需要发送新的目标。
    if (new_goal) {
      new_goal = false;
      ClientT::Goal client_goal;
      client_goal.pose = goal->poses[goal_index];
      LOG_INFO(
        "Sending waypoint {} to NavigateToPose at ({:.3f}, {:.3f})",
        goal_index, client_goal.pose.pose.position.x, client_goal.pose.pose.position.y);

      // 中文注释：每个路点都复用 bt_navigator 的 NavigateToPose 能力；
      // 本节点只根据返回状态决定继续、停止或记录 missed_waypoints。
      auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
      send_goal_options.result_callback =
        std::bind(&WaypointFollower::resultCallback, this, std::placeholders::_1);
      send_goal_options.goal_response_callback =
        std::bind(&WaypointFollower::goalResponseCallback, this, std::placeholders::_1);
      future_goal_handle_ =
        nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
      current_goal_status_ = ActionStatus::PROCESSING;
    }

    feedback->current_waypoint = goal_index;
    action_server_->publish_feedback(feedback);

    if (current_goal_status_ == ActionStatus::FAILED) {
      failed_ids_.push_back(goal_index);

      if (stop_on_failure_) {
        RCLCPP_WARN(
          get_logger(), "Failed to process waypoint %i in waypoint "
          "list and stop on failure is enabled."
          " Terminating action.", goal_index);
        result->missed_waypoints = failed_ids_;
        action_server_->terminate_current(result);
        failed_ids_.clear();
        return;
      } else {
        LOG_INFO("Failed to process waypoint {},"
          " moving to next.", goal_index);
      }
    } else if (current_goal_status_ == ActionStatus::SUCCEEDED) {
      LOG_INFO("Succeeded processing waypoint {}, processing waypoint task execution", goal_index);
      bool is_task_executed = waypoint_task_executor_->processAtWaypoint(
        goal->poses[goal_index], goal_index);
      LOG_INFO("Task execution at waypoint {} {}", goal_index, is_task_executed ? "succeeded" : "failed!");
      // if task execution was failed and stop_on_failure_ is on , terminate action
      // 中文：如果任务执行失败且 stop_on_failure_ 开启，则终止 action。
      if (!is_task_executed && stop_on_failure_) {
        failed_ids_.push_back(goal_index);
        RCLCPP_WARN(
          get_logger(), "Failed to execute task at waypoint %i "
          " stop on failure is enabled."
          " Terminating action.", goal_index);
        result->missed_waypoints = failed_ids_;
        action_server_->terminate_current(result);
        failed_ids_.clear();
        return;
      } else {
        LOG_INFO("Handled task execution on waypoint {},"
          " moving to next.", goal_index);
      }
    }

    if (current_goal_status_ != ActionStatus::PROCESSING &&
      current_goal_status_ != ActionStatus::UNKNOWN)
    {
      // Update server state
      // 中文：更新 server 状态。
      goal_index++;
      new_goal = true;
      if (goal_index >= goal->poses.size()) {
        LOG_INFO("Completed all {} waypoints requested.", goal->poses.size());
        result->missed_waypoints = failed_ids_;
        action_server_->succeeded_current(result);
        failed_ids_.clear();
        return;
      }
    } else {
      if (static_cast<int>(now().seconds()) % 30 == 0) {
        LOG_INFO("Processing waypoint {}...", goal_index);
      }
    }

    callback_group_executor_.spin_some();
    r.sleep();
  }
}

void
WaypointFollower::resultCallback(
  const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult & result)
{
  if (result.goal_id != future_goal_handle_.get()->get_goal_id()) {
    RCLCPP_DEBUG(
      get_logger(),
      "Goal IDs do not match for the current goal handle and received result."
      "Ignoring likely due to receiving result for an old goal.");
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      LOG_INFO("NavigateToPose result succeeded for current waypoint");
      current_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      LOG_INFO("NavigateToPose result aborted for current waypoint");
      current_goal_status_ = ActionStatus::FAILED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      LOG_INFO("NavigateToPose result canceled for current waypoint");
      current_goal_status_ = ActionStatus::FAILED;
      return;
    default:
      current_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }
}

void
WaypointFollower::goalResponseCallback(
  const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr & goal)
{
  if (!goal) {
    RCLCPP_ERROR(
      get_logger(),
      "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
}

rcl_interfaces::msg::SetParametersResult
WaypointFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  // No locking required as action server is running on same single threaded executor
  // 中文：action server 运行在同一个单线程 executor 上，因此不需要加锁。
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "loop_rate") {
        loop_rate_ = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "stop_on_failure") {
        stop_on_failure_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_waypoint_follower

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// 中文：将组件注册到 class_loader。
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// 中文：这相当于组件入口，使组件所在库被加载时可以被发现。
// is being loaded into a running process.
// 中文：当组件库被加载到运行中的进程时可被发现。
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_waypoint_follower::WaypointFollower)
