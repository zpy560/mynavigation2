// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_
#define NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_core/behavior.hpp"
#include "spdlog_wrapper.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_behaviors
{

enum class Status : int8_t
{
  SUCCEEDED = 1,
  FAILED = 2,
  RUNNING = 3,
};

using namespace std::chrono_literals;  //NOLINT

/**
 * @class nav2_behaviors::Behavior
 * @brief An action server Behavior base class implementing the action server and basic factory.
  * 中文：实现 action server 和基础工厂逻辑的 Behavior 基类。
 */
template<typename ActionT>
class TimedBehavior : public nav2_core::Behavior
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  /**
   * @brief A TimedBehavior constructor
   * 中文：TimedBehavior 构造函数。
   */
  TimedBehavior()
  : action_server_(nullptr),
    cycle_frequency_(10.0),
    enabled_(false),
    transform_tolerance_(0.0)
  {
  }

  virtual ~TimedBehavior() = default;

  // Derived classes can override this method to catch the command and perform some checks
  // 中文：派生类可以重写该方法以接收命令并执行检查。
  // before getting into the main loop. The method will only be called
  // 中文：进入主循环前调用；该方法只会被调用
  // once and should return SUCCEEDED otherwise behavior will return FAILED.
  // 中文：一次，并且应返回 SUCCEEDED，否则 behavior 会返回 FAILED。
  virtual Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) = 0;


  // This is the method derived classes should mainly implement
  // 中文：这是派生类主要需要实现的方法。
  // and will be called cyclically while it returns RUNNING.
  // 中文：当其返回 RUNNING 时会被周期性调用。
  // Implement the behavior such that it runs some unit of work on each call
  // 中文：实现 behavior 时，每次调用应执行一个工作单元，
  // and provides a status. The Behavior will finish once SUCCEEDED is returned
  // 中文：并返回状态；一旦返回 SUCCEEDED，Behavior 即结束。
  // It's up to the derived class to define the final commanded velocity.
  // 中文：最终命令速度由派生类定义。
  virtual Status onCycleUpdate() = 0;

  // an opportunity for derived classes to do something on configuration
  // 中文：为派生类在配置阶段执行自定义逻辑提供机会。
  // if they chose
  // 中文：如果派生类选择这样做。
  virtual void onConfigure()
  {
  }

  // an opportunity for derived classes to do something on cleanup
  // 中文：为派生类在 cleanup 阶段执行自定义逻辑提供机会。
  // if they chose
  // 中文：如果派生类选择这样做。
  virtual void onCleanup()
  {
  }

  // an opportunity for a derived class to do something on action completion
  // 中文：为派生类在 action 完成时执行自定义逻辑提供机会。
  virtual void onActionCompletion()
  {
  }

  // configure the server on lifecycle setup
  // 中文：在 lifecycle setup 阶段配置 server。
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) override
  {
    node_ = parent;
    auto node = node_.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    LOG_INFO("Configuring {}", name.c_str());

    behavior_name_ = name;
    tf_ = tf;

    node->get_parameter("cycle_frequency", cycle_frequency_);
    node->get_parameter("global_frame", global_frame_);
    node->get_parameter("robot_base_frame", robot_base_frame_);
    node->get_parameter("transform_tolerance", transform_tolerance_);

    action_server_ = std::make_shared<ActionServer>(
      node, behavior_name_,
      std::bind(&TimedBehavior::execute, this));

    collision_checker_ = collision_checker;

    vel_pub_ = node->template create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    onConfigure();
  }

  // Cleanup server on lifecycle transition
  // 中文：在 lifecycle 状态转换时清理 server。
  void cleanup() override
  {
    action_server_.reset();
    vel_pub_.reset();
    onCleanup();
  }

  // Activate server on lifecycle transition
  // 中文：在 lifecycle 状态转换时激活 server。
  void activate() override
  {
    LOG_INFO("Activating {}", behavior_name_.c_str());

    vel_pub_->on_activate();
    action_server_->activate();
    enabled_ = true;
  }

  // Deactivate server on lifecycle transition
  // 中文：在 lifecycle 状态转换时停用 server。
  void deactivate() override
  {
    vel_pub_->on_deactivate();
    action_server_->deactivate();
    enabled_ = false;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string behavior_name_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  std::shared_ptr<ActionServer> action_server_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  double cycle_frequency_;
  double enabled_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  rclcpp::Duration elasped_time_{0, 0};

  // Clock
  // 中文：时钟。
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  // 中文：日志器。
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_behaviors")};

  // Main execution callbacks for the action server implementation calling the Behavior's
  // 中文：action server 实现的主执行回调，用于调用 Behavior 的
  // onRun and cycle functions to execute a specific behavior
  // 中文：onRun 和 cycle 函数来执行具体 behavior。
  void execute()
  {
    LOG_INFO("Running {}", behavior_name_.c_str());

    if (!enabled_) {
      RCLCPP_WARN(
        logger_,
        "Called while inactive, ignoring request.");
      return;
    }

    if (onRun(action_server_->get_current_goal()) != Status::SUCCEEDED) {
      LOG_INFO("Initial checks failed for {}", behavior_name_.c_str());
      action_server_->terminate_current();
      return;
    }

    auto start_time = clock_->now();

    // Initialize the ActionT result
    // 中文：初始化 ActionT 结果。
    auto result = std::make_shared<typename ActionT::Result>();

    rclcpp::WallRate loop_rate(cycle_frequency_);

    while (rclcpp::ok()) {
      elasped_time_ = clock_->now() - start_time;
      if (action_server_->is_cancel_requested()) {
        LOG_INFO("Canceling {}", behavior_name_.c_str());
        stopRobot();
        result->total_elapsed_time = elasped_time_;
        action_server_->terminate_all(result);
        onActionCompletion();
        return;
      }

      // TODO(orduno) #868 Enable preempting a Behavior on-the-fly without stopping
      if (action_server_->is_preempt_requested()) {
        RCLCPP_ERROR(
          logger_, "Received a preemption request for %s,"
          " however feature is currently not implemented. Aborting and stopping.",
          behavior_name_.c_str());
        stopRobot();
        result->total_elapsed_time = clock_->now() - start_time;
        action_server_->terminate_current(result);
        onActionCompletion();
        return;
      }

      switch (onCycleUpdate()) {
        case Status::SUCCEEDED:
          LOG_INFO("{} completed successfully", behavior_name_.c_str());
          result->total_elapsed_time = clock_->now() - start_time;
          action_server_->succeeded_current(result);
          onActionCompletion();
          return;

        case Status::FAILED:
          RCLCPP_WARN(logger_, "%s failed", behavior_name_.c_str());
          result->total_elapsed_time = clock_->now() - start_time;
          action_server_->terminate_current(result);
          onActionCompletion();
          return;

        case Status::RUNNING:

        default:
          loop_rate.sleep();
          break;
      }
    }
  }

  // Stop the robot with a commanded velocity
  // 中文：通过命令速度让机器人停止。
  void stopRobot()
  {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = 0.0;
    cmd_vel->linear.y = 0.0;
    cmd_vel->angular.z = 0.0;

    vel_pub_->publish(std::move(cmd_vel));
  }
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_
