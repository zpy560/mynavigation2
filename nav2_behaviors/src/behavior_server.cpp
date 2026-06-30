// Copyright (c) 2018 Samsung Research America
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
// limitations under the License. Reserved.

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "nav2_util/node_utils.hpp"
#include "nav2_behaviors/behavior_server.hpp"

namespace behavior_server
{

BehaviorServer::BehaviorServer(const rclcpp::NodeOptions & options)
: LifecycleNode("behavior_server", "", options),
  plugin_loader_("nav2_core", "nav2_core::Behavior"),
  default_ids_{"spin", "backup", "drive_on_heading", "wait"},
  default_types_{"nav2_behaviors/Spin",
    "nav2_behaviors/BackUp",
    "nav2_behaviors/DriveOnHeading",
    "nav2_behaviors/Wait"}
{
  LOG_INFO("Creating behavior server");
  LOG_INFO("Behavior server will load recovery/action plugins and monitor local costmap collisions");

  // 中文注释：BehaviorServer 承接行为树中的恢复/辅助动作，例如 Spin、BackUp、
  // DriveOnHeading、Wait；具体行为通过 behavior_plugins 参数加载。
  declare_parameter(
    "costmap_topic",
    rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
  declare_parameter(
    "footprint_topic",
    rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));
  declare_parameter("cycle_frequency", rclcpp::ParameterValue(10.0));
  declare_parameter("behavior_plugins", default_ids_);

  get_parameter("behavior_plugins", behavior_ids_);
  if (behavior_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  declare_parameter(
    "global_frame",
    rclcpp::ParameterValue(std::string("odom")));
  declare_parameter(
    "robot_base_frame",
    rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter(
    "transform_tolerance",
    rclcpp::ParameterValue(0.1));
}


BehaviorServer::~BehaviorServer()
{
  behaviors_.clear();
}

nav2_util::CallbackReturn
BehaviorServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Configuring");
  LOG_INFO("Configuring behavior plugins and collision checker");

  // 中文注释：恢复行为通常依赖局部代价地图和机器人 footprint 判断是否安全；
  // 这里先建立 TF、costmap 和 footprint 订阅，再把 collision_checker 交给每个plugin
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  std::string costmap_topic, footprint_topic, robot_base_frame;
  double transform_tolerance;
  this->get_parameter("costmap_topic", costmap_topic);
  this->get_parameter("footprint_topic", footprint_topic);
  this->get_parameter("transform_tolerance", transform_tolerance);
  this->get_parameter("robot_base_frame", robot_base_frame);
  LOG_INFO(
    "Behavior server inputs costmap_topic={}, footprint_topic={}, robot_base_frame={}, transform_tolerance={}",
    costmap_topic.c_str(), footprint_topic.c_str(), robot_base_frame.c_str(), transform_tolerance);
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    shared_from_this(), costmap_topic);
  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
    shared_from_this(), footprint_topic, *tf_, robot_base_frame, transform_tolerance);

  collision_checker_ = std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_sub_, this->get_name());

  behavior_types_.resize(behavior_ids_.size());
  if (!loadBehaviorPlugins()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}


bool
BehaviorServer::loadBehaviorPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != behavior_ids_.size(); i++) {
    behavior_types_[i] = nav2_util::get_plugin_type_param(node, behavior_ids_[i]);
    try {
      LOG_INFO("Creating behavior plugin {} of type {}", behavior_ids_[i].c_str(), behavior_types_[i].c_str());
      // 中文注释：每个插件内部会创建自己的 action server；
      // BT Navigator 通过行为树节点调用这些 action 完成恢复动作。
      behaviors_.push_back(plugin_loader_.createUniqueInstance(behavior_types_[i]));
      behaviors_.back()->configure(node, behavior_ids_[i], tf_, collision_checker_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create behavior %s of type %s."
        " Exception: %s", behavior_ids_[i].c_str(), behavior_types_[i].c_str(),
        ex.what());
      return false;
    }
  }

  return true;
}

nav2_util::CallbackReturn
BehaviorServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Activating");
  LOG_INFO("Activating behavior plugins");
  LOG_INFO("Activating {} behavior plugins", behaviors_.size());
  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->activate();
  }

  // create bond connection
  // 中文：创建 bond 连接。
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BehaviorServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Deactivating");

  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->deactivate();
  }

  // destroy bond connection
  // 中文：销毁 bond 连接。
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BehaviorServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  LOG_INFO("Cleaning up");

  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->cleanup();
  }

  behaviors_.clear();
  transform_listener_.reset();
  tf_.reset();
  footprint_sub_.reset();
  costmap_sub_.reset();
  collision_checker_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BehaviorServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  LOG_INFO("Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // end namespace behavior_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// 中文：将组件注册到 class_loader。
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// 中文：这相当于组件入口，使组件所在库被加载时可以被发现。
// is being loaded into a running process.
// 中文：当组件库被加载到运行中的进程时可被发现。
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_server::BehaviorServer)
