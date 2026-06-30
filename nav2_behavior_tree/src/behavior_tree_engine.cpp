// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
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

#include "nav2_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace nav2_behavior_tree
{

BehaviorTreeEngine::BehaviorTreeEngine(
  const std::vector<std::string> & plugin_libraries)
{
  // 中文注释：行为树工厂只负责注册节点类型；每个动态库会向 factory_
  // 注册自己的 BT 节点，后续 XML 解析时按节点名实例化。
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

BtStatus
BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // 中文注释：BT 主循环每次 tickRoot 推进一次决策树；action 节点内部再通过
  // ROS action/service 调 planner、controller、recovery 等服务节点。
  // Loop until something happens with ROS or the node completes
  try {
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      if (cancelRequested()) {
        tree->rootNode()->halt();
        return BtStatus::CANCELED;
      }

      result = tree->tickRoot();

      onLoop();

      if (!loopRate.sleep()) {
        RCLCPP_WARN(
          rclcpp::get_logger("BehaviorTreeEngine"),
          "Behavior Tree tick rate %0.2f was exceeded!",
          1.0 / (loopRate.period().count() * 1.0e-9));
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  // 中文注释：blackboard 是行为树节点之间共享上下文的位置，
  // 例如目标点、路径、TF buffer 和 action client 都会挂在这里。
  return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree
BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void
BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
  if (!root_node) {
    return;
  }

  // this halt signal should propagate through the entire tree.
  root_node->halt();

  // 中文注释：取消导航时必须确保所有 RUNNING 节点都收到 halt，
  // 否则底层 action 可能继续执行，导致导航状态和机器人运动不一致。
  // but, just in case...
  auto visitor = [](BT::TreeNode * node) {
      if (node->status() == BT::NodeStatus::RUNNING) {
        node->halt();
      }
    };
  BT::applyRecursiveVisitor(root_node, visitor);
}

}  // namespace nav2_behavior_tree
