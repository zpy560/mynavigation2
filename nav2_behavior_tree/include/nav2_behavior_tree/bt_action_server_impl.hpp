// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_util/node_utils.hpp"
#include "spdlog_wrapper.hpp"

namespace nav2_behavior_tree
{

template<class ActionT>
BtActionServer<ActionT>::BtActionServer(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & action_name,
  const std::vector<std::string> & plugin_lib_names,
  const std::string & default_bt_xml_filename,
  OnGoalReceivedCallback on_goal_received_callback,
  OnLoopCallback on_loop_callback,
  OnPreemptCallback on_preempt_callback,
  OnCompletionCallback on_completion_callback)
: action_name_(action_name),
  default_bt_xml_filename_(default_bt_xml_filename),
  plugin_lib_names_(plugin_lib_names),
  node_(parent),
  on_goal_received_callback_(on_goal_received_callback),
  on_loop_callback_(on_loop_callback),
  on_preempt_callback_(on_preempt_callback),
  on_completion_callback_(on_completion_callback)
{
  LOG_INFO(
    "BtActionServer constructor entry: action_name={}, default_bt_xml={}, plugin_count={}",
    action_name_.c_str(), default_bt_xml_filename_.c_str(), plugin_lib_names_.size());

  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare this node's parameters
  if (!node->has_parameter("bt_loop_duration")) {
    node->declare_parameter("bt_loop_duration", 10);
  }
  if (!node->has_parameter("default_server_timeout")) {
    node->declare_parameter("default_server_timeout", 20);
  }
  if (!node->has_parameter("always_reload_bt_xml")) {
    node->declare_parameter("always_reload_bt_xml", false);
  }
  if (!node->has_parameter("wait_for_service_timeout")) {
    node->declare_parameter("wait_for_service_timeout", 1000);
  }

  LOG_INFO("BtActionServer constructor exit: action_name={}", action_name_.c_str());
}

template<class ActionT>
BtActionServer<ActionT>::~BtActionServer()
{}

template<class ActionT>
bool BtActionServer<ActionT>::on_configure()
{
  LOG_INFO("BtActionServer on_configure entry: action_name={}", action_name_.c_str());

  auto node = node_.lock();
  if (!node) {
    LOG_INFO("BtActionServer on_configure failed: action_name={}, node lock failed", action_name_.c_str());
    throw std::runtime_error{"Failed to lock node"};
  }

  // Name client node after action name
  std::string client_node_name = action_name_;
  std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
  // Use suffix '_rclcpp_node' to keep parameter file consistency #1773
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r",
      std::string("__node:=") +
      std::string(node->get_name()) + "_" + client_node_name + "_rclcpp_node",
      "--"});

  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  // Declare parameters for common client node applications to share with BT nodes
  // Declare if not declared in case being used an external application, then copying
  // all of the main node's parameters to the client for BT nodes to obtain
  nav2_util::declare_parameter_if_not_declared(
    node, "global_frame", rclcpp::ParameterValue(std::string("map")));
  nav2_util::declare_parameter_if_not_declared(
    node, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  nav2_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::copy_all_parameters(node, client_node_);

  action_server_ = std::make_shared<ActionServer>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this));

  // Get parameters for BT timeouts
  int timeout;
  node->get_parameter("bt_loop_duration", timeout);
  bt_loop_duration_ = std::chrono::milliseconds(timeout);
  node->get_parameter("default_server_timeout", timeout);
  default_server_timeout_ = std::chrono::milliseconds(timeout);
  int wait_for_service_timeout;
  node->get_parameter("wait_for_service_timeout", wait_for_service_timeout);
  wait_for_service_timeout_ = std::chrono::milliseconds(wait_for_service_timeout);
  node->get_parameter("always_reload_bt_xml", always_reload_bt_xml_);

  LOG_INFO(
    "BtActionServer on_configure params: action_name={}, bt_loop_duration_ms={}, "
    "default_server_timeout_ms={}, wait_for_service_timeout_ms={}, always_reload_bt_xml={}, "
    "plugin_count={}, client_node={}",
    action_name_.c_str(), bt_loop_duration_.count(), default_server_timeout_.count(),
    wait_for_service_timeout_.count(), always_reload_bt_xml_, plugin_lib_names_.size(),
    client_node_->get_name());

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>(
    "wait_for_service_timeout",
    wait_for_service_timeout_);

  LOG_INFO("BtActionServer on_configure exit: action_name={}", action_name_.c_str());
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_activate()
{
  LOG_INFO(
    "BtActionServer on_activate entry: action_name={}, default_bt_xml={}",
    action_name_.c_str(), default_bt_xml_filename_.c_str());

  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    LOG_INFO(
      "BtActionServer on_activate failed: action_name={}, default_bt_xml={}",
      action_name_.c_str(), default_bt_xml_filename_.c_str());
    RCLCPP_ERROR(logger_, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return false;
  }
  action_server_->activate();
  LOG_INFO(
    "BtActionServer on_activate exit: action_name={}, loaded_bt_xml={}",
    action_name_.c_str(), current_bt_xml_filename_.c_str());
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_deactivate()
{
  LOG_INFO("BtActionServer on_deactivate entry: action_name={}", action_name_.c_str());
  action_server_->deactivate();
  LOG_INFO("BtActionServer on_deactivate exit: action_name={}", action_name_.c_str());
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_cleanup()
{
  LOG_INFO(
    "BtActionServer on_cleanup entry: action_name={}, current_bt_xml={}",
    action_name_.c_str(), current_bt_xml_filename_.c_str());

  client_node_.reset();
  action_server_.reset();
  topic_logger_.reset();
  plugin_lib_names_.clear();
  current_bt_xml_filename_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_.rootNode());
  bt_.reset();

  LOG_INFO("BtActionServer on_cleanup exit: action_name={}", action_name_.c_str());
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Empty filename is default for backward compatibility
  auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

  LOG_INFO(
    "BtActionServer loadBehaviorTree entry: action_name={}, requested_bt_xml={}, "
    "resolved_bt_xml={}, current_bt_xml={}, always_reload_bt_xml={}",
    action_name_.c_str(), bt_xml_filename.c_str(), filename.c_str(),
    current_bt_xml_filename_.c_str(), always_reload_bt_xml_);

  // Use previous BT if it is the existing one and always reload flag is not set to true
  if (!always_reload_bt_xml_ && current_bt_xml_filename_ == filename) {
    LOG_INFO(
      "BtActionServer loadBehaviorTree reused current tree: action_name={}, bt_xml={}",
      action_name_.c_str(), filename.c_str());
    RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
    return true;
  }

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(filename);

  if (!xml_file.good()) {
    LOG_INFO(
      "BtActionServer loadBehaviorTree failed to open file: action_name={}, bt_xml={}",
      action_name_.c_str(), filename.c_str());
    RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", filename.c_str());
    return false;
  }

  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  // Create the Behavior Tree from the XML input
  try {
    tree_ = bt_->createTreeFromText(xml_string, blackboard_);
    for (auto & blackboard : tree_.blackboard_stack) {
      blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);
      blackboard->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
      blackboard->set<std::chrono::milliseconds>(
        "wait_for_service_timeout",
        wait_for_service_timeout_);
    }
  } catch (const std::exception & e) {
    LOG_INFO(
      "BtActionServer loadBehaviorTree exception: action_name={}, bt_xml={}, error={}",
      action_name_.c_str(), filename.c_str(), e.what());
    RCLCPP_ERROR(logger_, "Exception when loading BT: %s", e.what());
    return false;
  }

  topic_logger_ = std::make_unique<RosTopicLogger>(client_node_, tree_);

  current_bt_xml_filename_ = filename;
  LOG_INFO(
    "BtActionServer loadBehaviorTree exit: action_name={}, bt_xml={}, xml_size_bytes={}, "
    "blackboard_stack_size={}",
    action_name_.c_str(), current_bt_xml_filename_.c_str(), xml_string.size(),
    tree_.blackboard_stack.size());
  return true;
}

template<class ActionT>
void BtActionServer<ActionT>::executeCallback()
{
  LOG_INFO(
    "BtActionServer executeCallback entry: action_name={}, current_bt_xml={}",
    action_name_.c_str(), current_bt_xml_filename_.c_str());

  if (!on_goal_received_callback_(action_server_->get_current_goal())) {
    LOG_INFO("BtActionServer executeCallback rejected goal: action_name={}", action_name_.c_str());
    action_server_->terminate_current();
    return;
  }

  LOG_INFO("BtActionServer executeCallback accepted goal: action_name={}", action_name_.c_str());

  auto is_canceling = [&]() {
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
        return true;
      }
      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
        return true;
      }
      return action_server_->is_cancel_requested();
    };

  auto on_loop = [&]() {
      if (action_server_->is_preempt_requested() && on_preempt_callback_) {
        on_preempt_callback_(action_server_->get_pending_goal());
      }
      topic_logger_->flush();
      on_loop_callback_();
    };

  // Execute the BT that was previously created in the configure step
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);

  LOG_INFO(
    "BtActionServer executeCallback behavior tree finished: action_name={}, status={}",
    action_name_.c_str(), static_cast<int>(rc));

  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not needed.
  bt_->haltAllActions(tree_.rootNode());
  LOG_INFO("BtActionServer executeCallback halted actions: action_name={}", action_name_.c_str());

  // Give server an opportunity to populate the result message or simple give
  // an indication that the action is complete.
  auto result = std::make_shared<typename ActionT::Result>();
  on_completion_callback_(result, rc);

  switch (rc) {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      action_server_->succeeded_current(result);
      LOG_INFO("BtActionServer executeCallback exit: action_name={}, result=SUCCEEDED", action_name_.c_str());
      RCLCPP_INFO(logger_, "Goal succeeded");
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      action_server_->terminate_current(result);
      LOG_INFO("BtActionServer executeCallback exit: action_name={}, result=FAILED", action_name_.c_str());
      RCLCPP_ERROR(logger_, "Goal failed");
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      action_server_->terminate_all(result);
      LOG_INFO("BtActionServer executeCallback exit: action_name={}, result=CANCELED", action_name_.c_str());
      RCLCPP_INFO(logger_, "Goal canceled");
      break;
  }
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
