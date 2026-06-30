// Copyright (c) 2021 RoboTech Vision
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

#ifndef NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_
#define NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "spdlog_wrapper.hpp"

namespace nav2_smoother
{

/**
 * @class nav2_smoother::SmootherServer
 * @brief This class hosts variety of plugins of different algorithms to
  * 中文：该类托管多种不同算法插件，
 * smooth or refine a path from the exposed SmoothPath action server.
  * 中文：通过暴露的 SmoothPath action server 平滑或细化路径。
 */
class SmootherServer : public nav2_util::LifecycleNode
{
public:
  using SmootherMap = std::unordered_map<std::string, nav2_core::Smoother::Ptr>;

  /**
   * @brief A constructor for nav2_smoother::SmootherServer
   * 中文：nav2_smoother::SmootherServer 的构造函数。
   * @param options Additional options to control creation of the node.
   * 中文：用于控制节点创建的附加选项。
   */
  explicit SmootherServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for nav2_smoother::SmootherServer
   * 中文：nav2_smoother::SmootherServer 的析构函数。
   */
  ~SmootherServer();

protected:
  /**
   * @brief 配置 smoother 参数和成员variables
   *
   * Configures smoother plugin and costmap; Initialize odom subscriber,
   * 中文：配置 smoother 插件和 costmap；初始化 odom 订阅器、
   * velocity publisher and smooth path action server.
   * 中文：速度发布器和 smooth path action server。
   * @param state LifeCycle Node's state
   * 中文：Lifecycle 节点状态。
   * @return Success or Failure
   * 中文：成功或失败。
   * @throw pluginlib::PluginlibException When failed to initialize smoother
   * 中文：smoother 初始化失败时抛出。
   * plugin
   * 中文：插件。
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Loads smoother plugins from parameter file
   * 中文：从参数文件加载 smoother 插件。
   * @return bool if successfully loaded the plugins
   * 中文：bool 插件是否成功加载。
   */
  bool loadSmootherPlugins();

  /**
   * @brief Activates member variables
   * 中文：激活成员变量。
   *
   * Activates smoother, costmap, velocity publisher and smooth path action
   * 中文：激活 smoother、costmap、速度发布器和 smooth path action。
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
   * Deactivates smooth path action server, smoother, costmap and velocity
   * 中文：停用 smooth path action server、smoother、costmap 和速度发布器。
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
   * Smoother and costmap clean up state is called, and resets rest of the
   * 中文：调用 smoother 和 costmap 的 cleanup 状态，并重置其余变量。
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

  using Action = nav2_msgs::action::SmoothPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  /**
   * @brief SmoothPath action server callback. Handles action server updates and
   * 中文：SmoothPath action server 回调，处理 action server 更新并持续运行。
   * spins server until goal is reached
   * 中文：持续运行 server，直到目标到达。
   *
   * Provides global path to smoother received from action client. Local
   * 中文：将 action client 传入的全局路径提供给 smoother。路径的局部
   * section of the path is optimized using smoother.
   * 中文：片段会由 smoother 优化。
   * @throw nav2_core::PlannerException
   * 中文：nav2_core::PlannerException
   */
  void smoothPlan();

  /**
   * @brief Find the valid smoother ID name for the given request
   * 中文：为给定请求查找有效的 smoother ID 名称。
   *
   * @param c_name The requested smoother name
   * 中文：请求中的 smoother 名称。
   * @param name Reference to the name to use for control if any valid available
   * 中文：如果存在有效 smoother，则输出要使用的名称引用。
   * @return bool Whether it found a valid smoother to use
   * 中文：bool 是否找到可用的有效 smoother。
   */
  bool findSmootherId(const std::string & c_name, std::string & name);

  // Our action server implements the SmoothPath action
  // 中文：本 action server 实现 SmoothPath action。
  std::unique_ptr<ActionServer> action_server_;

  // Transforms
  // 中文：坐标变换。
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Publishers and subscribers
  // 中文：发布器和订阅器。
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // Smoother Plugins
  // 中文：Smoother 插件。
  pluginlib::ClassLoader<nav2_core::Smoother> lp_loader_;
  SmootherMap smoothers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> smoother_ids_;
  std::vector<std::string> smoother_types_;
  std::string smoother_ids_concat_, current_smoother_;

  // Utilities
  // 中文：工具对象。
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__NAV2_SMOOTHER_HPP_
