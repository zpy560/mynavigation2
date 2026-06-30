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

#ifndef NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
#define NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "spdlog_wrapper.hpp"

namespace nav2_controller
{

class ProgressChecker;
/**
 * @class nav2_controller::ControllerServer
 * @brief This class hosts variety of plugins of different algorithms to
  * 中文：该类托管多种不同算法插件，
 * complete control tasks from the exposed FollowPath action server.
  * 中文：用于完成 FollowPath action server 暴露的控制任务。
 */
class ControllerServer : public nav2_util::LifecycleNode
{
public:
  using ControllerMap = std::unordered_map<std::string, nav2_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, nav2_core::GoalChecker::Ptr>;

  /**
   * @brief Constructor for nav2_controller::ControllerServer
   * 中文：nav2_controller::ControllerServer 的构造函数。
   * @param options Additional options to control creation of the node.
   * 中文：用于控制节点创建的附加选项。
   */
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for nav2_controller::ControllerServer
   * 中文：nav2_controller::ControllerServer 的析构函数。
   */
  ~ControllerServer();

protected:
  /**
   * @brief Configures controller parameters and member variables
   * 中文：配置控制器参数和成员变量。
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * 中文：配置控制器插件和 costmap；初始化 odom 订阅器、
   * velocity publisher and follow path action server.
   * 中文：速度发布器和 follow path action server。
   * @param state LifeCycle Node's state
   * 中文：Lifecycle 节点状态。
   * @return Success or Failure
   * 中文：成功或失败。
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * 中文：控制器初始化失败时抛出。
   * plugin
   * 中文：插件。
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   * 中文：激活成员变量。
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * 中文：激活控制器、costmap、速度发布器和 follow path action
   * server
   * 中文：server。
   * @param state LifeCycle Node's state
   * 中文：Lifecycle 节点状态。
   * @return Success or Failure
   * 中文：成功或失败。
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   * 中文：停用成员变量。
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * 中文：停用 follow path action server、控制器、costmap 和速度
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * 中文：发布器；进入 deactivate 前会先将速度置零。
   * @param state LifeCycle Node's state
   * 中文：Lifecycle 节点状态。
   * @return Success or Failure
   * 中文：成功或失败。
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   * 中文：调用 cleanup 状态并重置成员变量。
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * 中文：调用控制器和 costmap 的 cleanup 状态，并重置其余
   * variables
   * 中文：变量。
   * @param state LifeCycle Node's state
   * 中文：Lifecycle 节点状态。
   * @return Success or Failure
   * 中文：成功或失败。
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * 中文：进入 Shutdown 状态时调用。
   * @param state LifeCycle Node's state
   * 中文：Lifecycle 节点状态。
   * @return Success or Failure
   * 中文：成功或失败。
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = nav2_msgs::action::FollowPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowPath action
  // 中文：本 action server 实现 FollowPath action。
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * 中文：FollowPath action server 回调；处理 action server 更新并
   * spins server until goal is reached
   * 中文：持续运行 server，直到目标到达。
   *
   * Provides global path to controller received from action client. Twist
   * 中文：将 action client 传入的全局路径提供给控制器；机器人 Twist
   * velocities for the robot are calculated and published using controller at
   * 中文：速度由控制器按指定频率计算并发布，
   * the specified rate till the goal is reached.
   * 中文：直到目标到达。
   * @throw nav2_core::PlannerException
   * 中文：nav2_core::PlannerException
   */
  void computeControl();

  /**
   * @brief Find the valid controller ID name for the given request
   * 中文：为给定请求查找有效的 controller ID 名称。
   *
   * @param c_name The requested controller name
   * 中文：请求中的控制器名称。
   * @param name Reference to the name to use for control if any valid available
   * 中文：如果存在有效 smoother，则输出要使用的名称引用。
   * @return bool Whether it found a valid controller to use
   * 中文：bool 是否找到可用的有效控制器。
   */
  bool findControllerId(const std::string & c_name, std::string & name);

  /**
   * @brief Find the valid goal checker ID name for the specified parameter
   * 中文：为指定参数查找有效的 goal checker ID 名称。
   *
   * @param c_name The goal checker name
   * 中文：goal checker 名称。
   * @param name Reference to the name to use for goal checking if any valid available
   * 中文：如果存在有效 goal checker，则输出用于到达判定的名称引用。
   * @return bool Whether it found a valid goal checker to use
   * 中文：bool 是否找到可用的有效 goal checker。
   */
  bool findGoalCheckerId(const std::string & c_name, std::string & name);

  /**
   * @brief Assigns path to controller
   * 中文：将路径分配给控制器。
   * @param path Path received from action server
   * 中文：从 action server 收到的路径。
   */
  void setPlannerPath(const nav_msgs::msg::Path & path);
  /**
   * @brief Calculates velocity and publishes to "cmd_vel" topic
   * 中文：计算速度并发布到 "cmd_vel" topic。
   */
  void computeAndPublishVelocity();
  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * 中文：使用从 action server 收到的更新路径调用 setPlannerPath 方法。
   * action server
   * 中文：action server。
   */
  void updateGlobalPath();
  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
   * 中文：调用速度发布器，将速度发布到 "cmd_vel" topic。
   * @param velocity Twist velocity to be published
   * 中文：要发布的 Twist 速度。
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);
  /**
   * @brief Calls velocity publisher to publish zero velocity
   * 中文：调用速度发布器发布零速度。
   */
  void publishZeroVelocity();
  /**
   * @brief Checks if goal is reached
   * 中文：检查目标是否到达。
   * @return true or false
   * 中文：true 或 false。
   */
  bool isGoalReached();
  /**
   * @brief Obtain current pose of the robot
   * 中文：获取机器人当前位姿。
   * @param pose To store current pose of the robot
   * 中文：用于存储机器人当前位姿。
   * @return true if able to obtain current pose of the robot, else false
   * 中文：若成功获取机器人当前位姿则返回 true，否则返回 false。
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief get the thresholded velocity
   * 中文：获取阈值处理后的速度。
   * @param velocity The current velocity from odometry
   * 中文：来自里程计的当前速度。
   * @param threshold The minimum velocity to return non-zero
   * 中文：返回非零值所需的最小速度。
   * @return double velocity value
   * 中文：double 速度值。
   */
  double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  /**
   * @brief get the thresholded Twist
   * 中文：获取阈值处理后的 Twist。
   * @param Twist The current Twist from odometry
   * 中文：来自里程计的当前 Twist。
   * @return Twist Twist after thresholds applied
   * 中文：Twist 应用阈值后的 Twist。
   */
  nav_2d_msgs::msg::Twist2D getThresholdedTwist(const nav_2d_msgs::msg::Twist2D & twist)
  {
    nav_2d_msgs::msg::Twist2D twist_thresh;
    twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
    twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
    twist_thresh.theta = getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
    return twist_thresh;
  }

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

  // The controller needs a costmap node
  // 中文：控制器需要一个 costmap 节点。
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  // 中文：发布器和订阅器。
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

  // Progress Checker Plugin
  // 中文：进度检查器插件。
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_checker_loader_;
  nav2_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // Goal Checker Plugin
  // 中文：目标检查器插件。
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string goal_checker_ids_concat_, current_goal_checker_;

  // Controller Plugins
  // 中文：控制器插件。
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, current_controller_;

  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  double failure_tolerance_;
  bool publish_zero_velocity_;

  // Whether we've published the single controller warning yet
  // 中文：是否已经发布过单控制器警告。
  geometry_msgs::msg::PoseStamped end_pose_;

  // Last time the controller generated a valid command
  // 中文：控制器上一次生成有效命令的时间。
  rclcpp::Time last_valid_cmd_time_;

  // Current path container
  // 中文：当前路径容器。
  nav_msgs::msg::Path current_path_;

private:
  /**
    * @brief Callback for speed limiting messages
    * 中文：速度限制消息回调。
    * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
    * 中文：nav2_msgs::msg::SpeedLimit 共享指针。
    */
  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
