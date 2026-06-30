// Copyright (c) 2019 Intel Corporation
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

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_controller/controller_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("controller_server", "", options),
  progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker"),
  default_progress_checker_id_{"progress_checker"},
  default_progress_checker_type_{"nav2_controller::SimpleProgressChecker"},
  goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
  default_goal_checker_ids_{"goal_checker"},
  default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"},
  lp_loader_("nav2_core", "nav2_core::Controller"),
  default_ids_{"FollowPath"},
  default_types_{"dwb_core::DWBLocalPlanner"}
{
  LOG_INFO("Creating controller server");
  LOG_INFO("get_namespace(),{}", get_namespace());
  LOG_INFO("Controller server owns local_costmap and loads progress, goal-checker and controller plugins");

  // 中文注释：ControllerServer 负责局部控制闭环；进度检查、到达判定和控制器
  // 都是插件，默认组合是 SimpleProgressChecker、SimpleGoalChecker 和 DWBLocalPlanner。
  declare_parameter("controller_frequency", 20.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));

  declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("publish_zero_velocity", rclcpp::ParameterValue(true));

  // The costmap node is used in the implementation of the controller
  // 中文：costmap 节点用于控制器实现。
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap");
}

ControllerServer::~ControllerServer()
{
  progress_checker_.reset();
  goal_checkers_.clear();
  controllers_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();

  LOG_INFO("Configuring controller interface");
  LOG_INFO("Configuring controller plugins, local costmap and FollowPath action server");

  get_parameter("progress_checker_plugin", progress_checker_id_);
  if (progress_checker_id_ == default_progress_checker_id_) {
    nav2_util::declare_parameter_if_not_declared(
      node, default_progress_checker_id_ + ".plugin",
      rclcpp::ParameterValue(default_progress_checker_type_));
  }

  LOG_INFO("getting goal checker plugins..");
  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (goal_checker_ids_ == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_goal_checker_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }

  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  controller_types_.resize(controller_ids_.size());
  goal_checker_types_.resize(goal_checker_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  LOG_INFO("Controller frequency set to {:.4f}Hz", controller_frequency_);

  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  get_parameter("failure_tolerance", failure_tolerance_);
  get_parameter("publish_zero_velocity", publish_zero_velocity_);

  // 中文注释：局部 costmap 在 controller 内部运行，用于实时避障和轨迹评分；
  // 独立线程保证 costmap 更新不阻塞 action 执行回调。
  costmap_ros_->configure();
  // Launch a thread to run the costmap node
  // 中文：启动线程运行 costmap 节点。
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  // 中文注释：progress checker 判断机器人是否持续取得进展，
  // 防止局部控制器长时间原地卡住但 action 不失败。
  try {
    progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
    progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
    LOG_INFO("Created progress_checker : {} of type {}", progress_checker_id_.c_str(), progress_checker_type_.c_str());
    progress_checker_->initialize(node, progress_checker_id_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create progress_checker. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  // 中文注释：goal checker 定义“到达目标”的几何容差，可按任务切换不同判定策略。
  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = nav2_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      nav2_core::GoalChecker::Ptr goal_checker =
        goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      LOG_INFO("Created goal checker : {} of type {}", goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
      goal_checker->initialize(node, goal_checker_ids_[i], costmap_ros_);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create goal checker. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  LOG_INFO("Controller Server has {} goal checkers available.", goal_checker_ids_concat_.c_str());

  // 中文注释：controller 插件把全局路径、局部 costmap、TF 和里程计转换成 cmd_vel。
  // 这里仅完成插件加载和配置，真正控制循环在 follow_path action 中执行。
  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      nav2_core::Controller::Ptr controller =
        lp_loader_.createUniqueInstance(controller_types_[i]);
      LOG_INFO("Created controller : {} of type {}", controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(
        node, controller_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create controller. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  LOG_INFO("Controller Server has {} controllers available.", controller_ids_concat_.c_str());

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // 中文注释：BT Navigator 通过 follow_path action 驱动控制器；
  // action 生命周期决定控制循环何时开始、取消和结束。
  // Create the action server that we implement with our followPath method
  // 中文：创建由 followPath 方法实现的 action server。
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "follow_path",
    std::bind(&ControllerServer::computeControl, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  // Set subscribtion to the speed limiting topic
  // 中文：设置速度限制 topic 订阅。
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10),
    std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Activating");
  LOG_INFO("Activating controller server publishers, local costmap and action server");

  costmap_ros_->activate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->activate();
  }
  vel_publisher_->on_activate();
  action_server_->activate();

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  // 中文：添加动态参数回调。
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ControllerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  // 中文：创建 bond 连接。
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Deactivating");

  action_server_->deactivate();
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->deactivate();
  }

  /*
   * The costmap is also a lifecycle node, so it may have already fired on_deactivate
   * 中文：costmap 也是 lifecycle node，因此可能已经触发过 on_deactivate。
   * via rcl preshutdown cb. Despite the rclcpp docs saying on_shutdown callbacks fire
   * 中文：这是通过 rcl preshutdown 回调触发的。尽管 rclcpp 文档说明 on_shutdown 回调会
   * in the order added, the preshutdown callbacks clearly don't per se, due to using an
   * 中文：按添加顺序触发，但 preshutdown 回调显然并非如此，因为底层使用了
   * unordered_set iteration. Once this issue is resolved, we can maybe make a stronger
   * 中文：unordered_set 迭代。该问题解决后，也许可以做出更强的
   * ordering assumption: https://github.com/ros2/rclcpp/issues/2096
   * 中文：顺序假设：https://github.com/ros2/rclcpp/issues/2096
   */
  costmap_ros_->deactivate();

  publishZeroVelocity();
  vel_publisher_->on_deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  // 中文：销毁 bond 连接。
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Cleaning up");

  // Cleanup the helper classes
  // 中文：清理辅助类。
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->cleanup();
  }
  controllers_.clear();

  goal_checkers_.clear();

  costmap_ros_->cleanup();


  // Release any allocated resources
  // 中文：释放已分配的资源。
  action_server_.reset();
  odom_sub_.reset();
  costmap_thread_.reset();
  vel_publisher_.reset();
  speed_limit_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  LOG_INFO("Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool ControllerServer::findControllerId(
  const std::string & c_name,
  std::string & current_controller)
{
  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No controller was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with controller name %s, "
        "which does not exist. Available controllers are: %s.",
        c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

bool ControllerServer::findGoalCheckerId(
  const std::string & c_name,
  std::string & current_goal_checker)
{
  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No goal checker was specified in parameter 'current_goal_checker'."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with goal_checker name %s in parameter"
        " 'current_goal_checker', which does not exist. Available goal checkers are: %s.",
        c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

void ControllerServer::computeControl()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  LOG_INFO("Received a goal, begin computing control effort.");

  try {
    std::string c_name = action_server_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      action_server_->terminate_current();
      return;
    }

    std::string gc_name = action_server_->get_current_goal()->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      action_server_->terminate_current();
      return;
    }

    setPlannerPath(action_server_->get_current_goal()->path);
    progress_checker_->reset();

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        LOG_INFO("Goal was canceled. Stopping the robot.");
        action_server_->terminate_all();
        publishZeroVelocity();
        return;
      }

      // Don't compute a trajectory until costmap is valid (after clear costmap)
      // 中文：在 costmap 有效前不要计算轨迹（例如清除 costmap 之后）。
      rclcpp::Rate r(100);
      while (!costmap_ros_->isCurrent()) {
        r.sleep();
      }

      updateGlobalPath();

      computeAndPublishVelocity();

      if (isGoalReached()) {
        LOG_INFO("Reached the goal!");
        break;
      }

      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
          get_logger(), "Control loop missed its desired rate of %.4fHz",
          controller_frequency_);
        loop_rate.reset();
      }
    }
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    action_server_->terminate_current();
    return;
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  if (publish_zero_velocity_) {
    publishZeroVelocity();
  }

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  // 中文：TODO(orduno) #861 处理待处理的抢占请求并设置控制器名称。
  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Invalid path, Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);

  end_pose_ = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();

  RCLCPP_DEBUG(
    get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose_.pose.position.x, end_pose_.pose.position.y);

  current_path_ = path;
}

void ControllerServer::computeAndPublishVelocity()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    throw nav2_core::PlannerException("Failed to obtain robot pose");
  }

  if (!progress_checker_->check(pose)) {
    throw nav2_core::PlannerException("Failed to make progress");
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());

  geometry_msgs::msg::TwistStamped cmd_vel_2d;

  try {
    cmd_vel_2d =
      controllers_[current_controller_]->computeVelocityCommands(
      pose,
      nav_2d_utils::twist2Dto3D(twist),
      goal_checkers_[current_goal_checker_].get());
    last_valid_cmd_time_ = now();
  } catch (nav2_core::PlannerException & e) {
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      cmd_vel_2d.twist.angular.x = 0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      cmd_vel_2d.twist.linear.x = 0;
      cmd_vel_2d.twist.linear.y = 0;
      cmd_vel_2d.twist.linear.z = 0;
      cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel_2d.header.stamp = now();
      if ((now() - last_valid_cmd_time_).seconds() > failure_tolerance_ &&
        failure_tolerance_ != -1.0)
      {
        throw nav2_core::PlannerException("Controller patience exceeded");
      }
    } else {
      throw nav2_core::PlannerException(e.what());
    }
  }

  std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  // Find the closest pose to current pose on global path
  // 中文：在全局路径上查找距离当前位姿最近的位姿。
  nav_msgs::msg::Path & current_path = current_path_;
  auto find_closest_pose_idx =
    [&pose, &current_path]() {
      size_t closest_pose_idx = 0;
      double curr_min_dist = std::numeric_limits<double>::max();
      for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
        double curr_dist = nav2_util::geometry_utils::euclidean_distance(
          pose, current_path.poses[curr_idx]);
        if (curr_dist < curr_min_dist) {
          curr_min_dist = curr_dist;
          closest_pose_idx = curr_idx;
        }
      }
      return closest_pose_idx;
    };

  feedback->distance_to_goal =
    nav2_util::geometry_utils::calculate_path_length(current_path_, find_closest_pose_idx());
  action_server_->publish_feedback(feedback);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
}

void ControllerServer::updateGlobalPath()
{
  if (action_server_->is_preempt_requested()) {
    LOG_INFO("Passing new path to controller.");
    auto goal = action_server_->accept_pending_goal();
    std::string current_controller;
    if (findControllerId(goal->controller_id, current_controller)) {
      current_controller_ = current_controller;
    } else {
      LOG_INFO("Terminating action, invalid controller {} requested.", goal->controller_id.c_str());
      action_server_->terminate_current();
      return;
    }
    std::string current_goal_checker;
    if (findGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      LOG_INFO("Terminating action, invalid goal checker {} requested.", goal->goal_checker_id.c_str());
      action_server_->terminate_current();
      return;
    }
    setPlannerPath(goal->path);
  }
}

void ControllerServer::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void ControllerServer::publishZeroVelocity()
{
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();
  publishVelocity(velocity);
}

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose)) {
    return false;
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());
  geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist);

  geometry_msgs::msg::PoseStamped transformed_end_pose;
  rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));
  nav_2d_utils::transformPose(
    costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
    end_pose_, transformed_end_pose, tolerance);

  return goal_checkers_[current_goal_checker_]->isGoalReached(
    pose.pose, transformed_end_pose.pose,
    velocity);
}

bool ControllerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

void ControllerServer::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
{
  ControllerMap::iterator it;
  for (it = controllers_.begin(); it != controllers_.end(); ++it) {
    it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
}

rcl_interfaces::msg::SetParametersResult
ControllerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // 中文：如果正在尝试修改某个插件的参数，此处可以直接跳过。
    // as they handle parameter changes themselves and don't need to lock the mutex
    // 中文：因为插件会自行处理参数变化，不需要锁住该 mutex。
    if (name.find('.') != std::string::npos) {
      continue;
    }

    if (!dynamic_params_lock_.try_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to dynamically change Parameters while the controller is currently running");
      result.successful = false;
      result.reason =
        "Unable to dynamically change Parameters while the controller is currently running";
      return result;
    }

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "min_x_velocity_threshold") {
        min_x_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_y_velocity_threshold") {
        min_y_velocity_threshold_ = parameter.as_double();
      } else if (name == "min_theta_velocity_threshold") {
        min_theta_velocity_threshold_ = parameter.as_double();
      } else if (name == "failure_tolerance") {
        failure_tolerance_ = parameter.as_double();
      }
    }

    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// 中文：将组件注册到 class_loader。
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// 中文：这相当于组件入口，使组件所在库被加载时可以被发现。
// is being loaded into a running process.
// 中文：当组件库被加载到运行中的进程时可被发现。
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_controller::ControllerServer)
