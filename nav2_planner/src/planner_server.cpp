// Copyright (c) 2018 Intel Corporation
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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

#include "nav2_planner/planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("planner_server", "", options),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_ids_{"GridBased", "GridBasedAstar", "Smac2D", "SmacHybrid", "SmacLattice", "ThetaStar"},
  default_types_{"nav2_navfn_planner/NavfnPlanner", "nav2_navfn_planner/NavfnPlanner", "nav2_smac_planner/SmacPlanner2D", "nav2_smac_planner/SmacPlannerHybrid", "nav2_smac_planner/SmacPlannerLattice", "nav2_theta_star_planner/ThetaStarPlanner"},
  costmap_(nullptr)
{
  // SpdlogWrapper::init("nav2_planner", get_name());
  LOG_INFO("Creating");
  LOG_INFO("Planner server owns global_costmap and loads global planner plugins");

  // 中文注释：PlannerServer 只管理全局规划；具体算法由 planner_plugins 参数决定，
  // 默认加载 NavfnPlanner，运行时通过 pluginlib 统一创建和生命周期管理。
  // 声明该节点的parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("selected_planner", rclcpp::ParameterValue(std::string("")));
  declare_parameter("expected_planner_frequency", 1.0);

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
      LOG_INFO("default_ids_[i],{},default_types_[i],{}", default_ids_[i], default_types_[i]);
    }
  }

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");
}

PlannerServer::~PlannerServer()
{
  /*
   * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
   * never called.
   */
  planners_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Configuring");
  LOG_INFO("Configuring planner plugins, global costmap and ComputePath action servers");

  costmap_ros_->configure();
  costmap_ = costmap_ros_->getCostmap();

  // 中文注释：全局 costmap 是规划算法的环境输入；这里先完成 costmap 生命周期配置，
  // 再把同一份 costmap_ros_ 传给每个全局规划plugin
  if (!costmap_ros_->getUseRadius()) {
    collision_checker_ =
      std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
      costmap_);
  }

  // Launch a thread to run the costmap node
  // 中文：启动线程运行 costmap 节点。
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  // 中文注释：每个 planner id 会映射到一个插件类型，例如 GridBased -> NavfnPlanner；
  // 插件创建失败直接让生命周期配置失败，避免系统带着缺失规划器进入 active。
  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = nav2_util::get_plugin_type_param(
        node, planner_ids_[i]);
      nav2_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      LOG_INFO("Created global planner plugin {} of type {}", planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->configure(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  LOG_INFO("Planner Server has {} planners available.", planner_ids_concat_.c_str());

  get_parameter("selected_planner", selected_planner_id_);
  if (!selected_planner_id_.empty()) {
    if (planners_.find(selected_planner_id_) == planners_.end()) {
      RCLCPP_WARN(
        get_logger(), "Selected planner %s is not configured. Planner names are: %s",
        selected_planner_id_.c_str(), planner_ids_concat_.c_str());
    } else {
      LOG_INFO("Planner server selected planner interface {}", selected_planner_id_.c_str());
    }
  }

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  // 中文：初始化发布器和订阅器。
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // 中文注释：对外提供两个 action：单目标点规划和多目标点串联规划；
  // BT Navigator 会通过这些 action 调用 planner server
  // Create the action servers for path planning to a pose and through poses
  // 中文：Create the action server。s for path planning to a pose and through poses
  action_server_pose_ = std::make_unique<ActionServerToPose>(
    shared_from_this(),
    "compute_path_to_pose",
    std::bind(&PlannerServer::computePlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
    shared_from_this(),
    "compute_path_through_poses",
    std::bind(&PlannerServer::computePlanThroughPoses, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Activating");
  LOG_INFO("Activating planner server action servers, publishers and planner plugins");

  plan_publisher_->on_activate();
  action_server_pose_->activate();
  action_server_poses_->activate();
  costmap_ros_->activate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();

  is_path_valid_service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
    "is_path_valid",
    std::bind(
      &PlannerServer::isPathValid, this,
      std::placeholders::_1, std::placeholders::_2));

  // Add callback for dynamic parameters
  // 中文：添加动态参数回调。
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PlannerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  // 中文：创建 bond 连接。
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Deactivating");

  action_server_pose_->deactivate();
  action_server_poses_->deactivate();
  plan_publisher_->on_deactivate();

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

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  // 中文：销毁 bond 连接。
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Cleaning up");

  action_server_pose_.reset();
  action_server_poses_.reset();
  plan_publisher_.reset();
  tf_.reset();

  costmap_ros_->cleanup();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }

  planners_.clear();
  costmap_thread_.reset();
  costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  LOG_INFO("Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void PlannerServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool PlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    LOG_INFO("Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool PlannerServer::getStartPose(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal,
  geometry_msgs::msg::PoseStamped & start)
{
  if (goal->use_start) {
    start = goal->start;
  } else if (!costmap_ros_->getRobotPose(start)) {
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::transformPosesToGlobalFrame(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  geometry_msgs::msg::PoseStamped & curr_start,
  geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
  {
    RCLCPP_WARN(
      get_logger(), "Could not transform the start or goal pose in the costmap frame");
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::validatePath(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  if (path.poses.size() == 0) {
    RCLCPP_WARN(
      get_logger(), "Planning algorithm %s failed to generate a valid"
      " path to (%.2f, %.2f)", planner_id.c_str(),
      goal.pose.position.x, goal.pose.position.y);
    action_server->terminate_current();
    return false;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Found valid path of size %zu to (%.2f, %.2f)",
    path.poses.size(), goal.pose.position.x,
    goal.pose.position.y);

  return true;
}

void
PlannerServer::computePlanThroughPoses()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = this->now();

  // Initialize the ComputePathToPose goal and result
  // 中文：Initialize the Compute生成的路径。ToPose goal and result
  auto goal = action_server_poses_->get_current_goal();
  auto result = std::make_shared<ActionThroughPoses::Result>();
  nav_msgs::msg::Path concat_path;

  try {
    if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_poses_, goal);

    if (goal->goals.size() == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Compute path through poses requested a plan with no viapoint poses, returning.");
      action_server_poses_->terminate_current();
    }

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_poses_, goal, start)) {
      return;
    }

    // Get consecutive paths through these points
    geometry_msgs::msg::PoseStamped curr_start, curr_goal;
    for (unsigned int i = 0; i != goal->goals.size(); i++) {
      // Get starting point
      if (i == 0) {
        curr_start = start;
      } else {
        // pick the end of the last planning task as the start for the next one
        // to allow for path tolerance deviations
        curr_start = concat_path.poses.back();
        curr_start.header = concat_path.header;
      }
      curr_goal = goal->goals[i];

      // Transform them into the global frame
      if (!transformPosesToGlobalFrame(action_server_poses_, curr_start, curr_goal)) {
        return;
      }

      // Get plan from start -> goal
      nav_msgs::msg::Path curr_path = getPlan(curr_start, curr_goal, goal->planner_id);

      // check path for validity
      if (!validatePath(action_server_poses_, curr_goal, curr_path, goal->planner_id)) {
        return;
      }

      // Concatenate paths together
      concat_path.poses.insert(
        concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
      concat_path.header = curr_path.header;
    }

    // Publish the plan for visualization purposes
    result->path = concat_path;
    publishPlan(result->path);

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_poses_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "%s plugin failed to plan through %zu points with final goal (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goals.size(), goal->goals.back().pose.position.x,
      goal->goals.back().pose.position.y, ex.what());
    action_server_poses_->terminate_current();
  }
}

void
PlannerServer::computePlan()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = this->now();

  // Initialize the ComputePathToPose goal and result
  // 中文：Initialize the Compute生成的路径。ToPose goal and result
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  try {
    if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_pose_, goal);

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_pose_, goal, start)) {
      return;
    }

    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (!transformPosesToGlobalFrame(action_server_pose_, start, goal_pose)) {
      return;
    }

    result->path = getPlan(start, goal_pose, goal->planner_id);

    if (!validatePath(action_server_pose_, goal_pose, result->path, goal->planner_id)) {
      return;
    }

    // Publish the plan for visualization purposes
    publishPlan(result->path);

    auto cycle_duration = this->now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_pose_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goal.pose.position.x,
      goal->goal.pose.position.y, ex.what());
    action_server_pose_->terminate_current();
  }
}

nav_msgs::msg::Path
PlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id)
{
  RCLCPP_DEBUG(
    get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  const std::string requested_planner_id = selected_planner_id_.empty() ?
    planner_id : selected_planner_id_;

  if (planners_.find(requested_planner_id) != planners_.end()) {
    if (!selected_planner_id_.empty() && planner_id != selected_planner_id_) {
      RCLCPP_WARN_ONCE(
        get_logger(), "Action requested planner %s, but selected_planner is %s. "
        "Using selected_planner from configuration.",
        planner_id.c_str(), selected_planner_id_.c_str());
    }
    return planners_[requested_planner_id]->createPlan(start, goal);
  } else {
    if (planners_.size() == 1 && requested_planner_id.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    } else if (planners_.size() == 1) {
      RCLCPP_WARN_ONCE(
        get_logger(), "Requested planner %s is not configured. "
        "Server will use only configured plugin %s instead."
        " This warning will appear once.", requested_planner_id.c_str(), planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner %s is not a valid planner. "
        "Planner names are: %s", requested_planner_id.c_str(),
        planner_ids_concat_.c_str());
    }
  }

  return nav_msgs::msg::Path();
}

void
PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

void PlannerServer::isPathValid(
  const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
  response->is_valid = true;

  if (request->path.poses.empty()) {
    response->is_valid = false;
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  unsigned int closest_point_index = 0;
  if (costmap_ros_->getRobotPose(current_pose)) {
    float current_distance = std::numeric_limits<float>::max();
    float closest_distance = current_distance;
    geometry_msgs::msg::Point current_point = current_pose.pose.position;
    for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
      geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

      current_distance = nav2_util::geometry_utils::euclidean_distance(
        current_point,
        path_point);

      if (current_distance < closest_distance) {
        closest_point_index = i;
        closest_distance = current_distance;
      }
    }

    /**
     * The lethal check starts at the closest point to avoid points that have already been passed
     * and may have become occupied
     */
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    unsigned int mx = 0;
    unsigned int my = 0;

    bool use_radius = costmap_ros_->getUseRadius();

    unsigned int cost = nav2_costmap_2d::FREE_SPACE;

    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      auto & position = request->path.poses[i].pose.position;
      if (use_radius) {
        if (costmap_->worldToMap(position.x, position.y, mx, my)) {
          cost = costmap_->getCost(mx, my);
        } else {
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        }
      } else {
        nav2_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
        auto theta = tf2::getYaw(request->path.poses[i].pose.orientation);
        cost = static_cast<unsigned int>(collision_checker_->footprintCostAtPose(
            position.x, position.y, theta, footprint));
      }

      if (use_radius &&
        (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
      {
        response->is_valid = false;
        break;
      } else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        response->is_valid = false;
        break;
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult
PlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
        } else {
          RCLCPP_WARN(
            get_logger(),
            "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
            " than 0.0 to turn on duration overrrun warning messages", parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "selected_planner") {
        const auto selected_planner = parameter.as_string();
        if (!selected_planner.empty() && planners_.find(selected_planner) == planners_.end()) {
          result.successful = false;
          result.reason = "selected_planner is not configured";
          return result;
        }
        selected_planner_id_ = selected_planner;
        LOG_INFO("Planner server selected planner interface {}", selected_planner_id_.c_str());
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// 中文：将组件注册到 class_loader。
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// 中文：这相当于组件入口，使组件所在库被加载时可以被发现。
// is being loaded into a running process.
// 中文：当组件库被加载到运行中的进程时可被发现。
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_planner::PlannerServer)
