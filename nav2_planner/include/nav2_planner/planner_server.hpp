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

#ifndef NAV2_PLANNER__PLANNER_SERVER_HPP_
#define NAV2_PLANNER__PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "spdlog_wrapper.hpp"

namespace nav2_planner
{
/**
 * @class nav2_planner::PlannerServer
 * @brief An action server implements the behavior tree's ComputePathToPose
  * 中文：实现行为树 ComputePathToPose 接口的 action server，
 * interface and hosts various plugins of different algorithms to compute plans.
  * 中文：并托管多种全局规划算法插件来计算路径。
 */
class PlannerServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_planner::PlannerServer
   * 中文：nav2_planner::PlannerServer 的构造函数。
   * @param options Additional options to control creation of the node.
   * 中文：用于控制节点创建的附加选项。
   */
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for nav2_planner::PlannerServer
   * 中文：nav2_planner::PlannerServer 的析构函数。
   */
  ~PlannerServer();

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;

  /**
   * @brief Method to get plan from the desired plugin
   * 中文：从指定规划器插件获取路径。
   * @param start starting pose
   * @param goal goal request
   * @return Path
   * 中文：生成的路径。
   */
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);

protected:
  /**
   * @brief Configure member variables and initializes planner
   * 中文：配置成员变量并初始化规划器。
   * @param state Reference to LifeCycle node state
   * 中文：Lifecycle 节点状态引用。
   * @return SUCCESS or FAILURE
   * 中文：SUCCESS 或 FAILURE。
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   * 中文：激活成员变量。
   * @param state Reference to LifeCycle node state
   * 中文：Lifecycle 节点状态引用。
   * @return SUCCESS or FAILURE
   * 中文：SUCCESS 或 FAILURE。
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   * 中文：停用成员变量。
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

  using ActionToPose = nav2_msgs::action::ComputePathToPose;
  using ActionThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using ActionServerToPose = nav2_util::SimpleActionServer<ActionToPose>;
  using ActionServerThroughPoses = nav2_util::SimpleActionServer<ActionThroughPoses>;

  /**
   * @brief Check if an action server is valid / active
   * 中文：检查 action server 是否有效或处于 active 状态。
   * @param action_server Action server to test
   * 中文：需要检查的 action server。
   * @return SUCCESS or FAILURE
   * 中文：SUCCESS 或 FAILURE。
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * 中文：检查 action server 是否存在待处理的取消请求。
   * @param action_server Action server to test
   * 中文：需要检查的 action server。
   * @return SUCCESS or FAILURE
   * 中文：SUCCESS 或 FAILURE。
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Wait for costmap to be valid with updated sensor data or repopulate after a
   * 中文：等待 costmap 使用最新传感器数据变为有效，或在
   * clearing recovery. Blocks until true without timeout.
   * 中文：清除恢复行为后重新填充；该函数会一直阻塞直到有效，不设置超时。
   */
  void waitForCostmap();

  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * 中文：检查 action server 是否存在抢占请求，并用新的抢占目标替换当前目标。
   * with the new preemption goal.
   * 中文：用新的抢占目标替换。
   * @param action_server Action server to get updated goal if required
   * 中文：必要时从该 action server 获取更新目标。
   * @param goal Goal to overwrite
   * 中文：需要覆盖的目标。
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);

  /**
   * @brief Get the starting pose from costmap or message, if valid
   * 中文：如果有效，则从 costmap 或请求消息中获取起始位姿。
   * @param action_server Action server to terminate if required
   * 中文：必要时需要终止的 action server。
   * @param goal Goal to find start from
   * 中文：用于查找起点的目标请求。
   * @param start The starting pose to use
   * 中文：最终使用的起始位姿。
   * @return bool If successful in finding a valid starting pose
   * 中文：bool 是否成功找到有效起始位姿。
   */
  template<typename T>
  bool getStartPose(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal,
    geometry_msgs::msg::PoseStamped & start);

  /**
   * @brief Transform start and goal poses into the costmap
   * 中文：将起始位姿和目标位姿转换到 costmap
   * global frame for path planning plugins to utilize
   * 中文：的全局坐标系，供路径规划插件使用。
   * @param action_server Action server to terminate if required
   * 中文：必要时需要终止的 action server。
   * @param start The starting pose to transform
   * 中文：需要转换的起始位姿。
   * @param goal Goal pose to transform
   * 中文：需要转换的目标位姿。
   * @return bool If successful in transforming poses
   * 中文：bool 位姿转换是否成功。
   */
  template<typename T>
  bool transformPosesToGlobalFrame(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    geometry_msgs::msg::PoseStamped & curr_start,
    geometry_msgs::msg::PoseStamped & curr_goal);

  /**
   * @brief Validate that the path contains a meaningful path
   * 中文：验证路径是否包含有意义的路径内容。
   * @param action_server Action server to terminate if required
   * 中文：必要时需要终止的 action server。
   * @param Goal Current goal
   * @param path Current path
   * 中文：当前路径。
   * @param planner_id The planner ID used to generate the path
   * 中文：用于生成路径的 planner ID。
   * @return bool If path is valid
   * 中文：bool 路径是否有效。
   */
  template<typename T>
  bool validatePath(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    const geometry_msgs::msg::PoseStamped & curr_goal,
    const nav_msgs::msg::Path & path,
    const std::string & planner_id);

  // Our action server implements the ComputePathToPose action
  // 中文：本 action server 实现 ComputePathToPose action。
  std::unique_ptr<ActionServerToPose> action_server_pose_;
  std::unique_ptr<ActionServerThroughPoses> action_server_poses_;

  /**
   * @brief The action server callback which calls planner to get the path
   * 中文：action server 回调，用于调用规划器获取路径。
   * ComputePathToPose
   * 中文：Compute生成的路径。ToPose
   */
  void computePlan();

  /**
   * @brief The action server callback which calls planner to get the path
   * 中文：action server 回调，用于调用规划器获取路径。
   * ComputePathThroughPoses
   * 中文：Compute生成的路径。ThroughPoses
   */
  void computePlanThroughPoses();

  /**
   * @brief The service callback to determine if the path is still valid
   * 中文：用于判断路径是否仍然有效的 service 回调。
   * @param request to the service
   * @param response from the service
   */
  void isPathValid(
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);

  /**
   * @brief Publish a path for visualization purposes
   * 中文：发布路径用于可视化。
   * @param path Reference to Global Path
   * 中文：全局路径引用。
   */
  void publishPlan(const nav_msgs::msg::Path & path);

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
  std::mutex dynamic_params_lock_;

  // Planner
  // 中文：规划器。
  PlannerMap planners_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  std::string planner_ids_concat_;
  std::string selected_planner_id_;

  // TF buffer
  // 中文：TF 缓冲区。
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap
  // 中文：全局 Costmap。
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  // Publishers for the path
  // 中文：路径发布器。
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // Service to deterime if the path is valid
  // 中文：用于判断路径是否有效的 service。
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr is_path_valid_service_;
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__PLANNER_SERVER_HPP_
