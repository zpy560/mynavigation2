#include "nav2_regulated_modules/chassis_control_subscriber.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "spdlog_wrapper.hpp"

namespace nav2_regulated_modules
{

ChassisControlSubscriber::ChassisControlSubscriber(nav2_util::LifecycleNode & node, MotionStateSubscriber & motion_state_subscriber, ChassisControlConfig config) : node_(node), motion_state_subscriber_(motion_state_subscriber), config_(std::move(config)) {
  if (config_.output_topic.empty() || !std::isfinite(config_.motion_state_timeout) || config_.motion_state_timeout <= 0.0 || !std::isfinite(config_.max_linear_velocity) || config_.max_linear_velocity <= 0.0 || !std::isfinite(config_.max_angular_velocity) || config_.max_angular_velocity <= 0.0 || !std::isfinite(config_.default_linear_acceleration) || config_.default_linear_acceleration <= 0.0 || !std::isfinite(config_.default_angular_acceleration) || config_.default_angular_acceleration <= 0.0 || !std::isfinite(config_.max_linear_acceleration) || config_.max_linear_acceleration <= 0.0 || !std::isfinite(config_.max_angular_acceleration) || config_.max_angular_acceleration <= 0.0 || !std::isfinite(config_.linear_stop_threshold) || config_.linear_stop_threshold < 0.0 || !std::isfinite(config_.angular_stop_threshold) || config_.angular_stop_threshold < 0.0 || !std::isfinite(config_.linear_kp) || config_.linear_kp < 0.0 || !std::isfinite(config_.linear_ki) || config_.linear_ki < 0.0 || !std::isfinite(config_.angular_kp) || config_.angular_kp < 0.0 || !std::isfinite(config_.angular_ki) || config_.angular_ki < 0.0 || !std::isfinite(config_.linear_integral_limit) || config_.linear_integral_limit <= 0.0 || !std::isfinite(config_.angular_integral_limit) || config_.angular_integral_limit <= 0.0) {throw std::invalid_argument("ChassisControl 闭环参数非法");}
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  subscription_ = node.create_subscription<byd_custom_msgs::msg::ChassisControl>("/downstream/chassis_control", qos, std::bind(&ChassisControlSubscriber::onChassisControl, this, std::placeholders::_1));
  publisher_ = node.create_publisher<byd_custom_msgs::msg::ControlRes>(config_.output_topic, qos);
}

void ChassisControlSubscriber::onChassisControl(const byd_custom_msgs::msg::ChassisControl::ConstSharedPtr message) {
  TargetCommand command;
  command.operation = message->op;
  if (!std::isfinite(message->linear_velocity) || !std::isfinite(message->angular_velocity) || !std::isfinite(message->acceleration) || message->linear_velocity < 0.0F || message->angular_velocity < 0.0F || message->acceleration < 0.0F) {
    LOG_ERROR("拒绝非法 ChassisControl：op={}，linear_velocity={}，angular_velocity={}，acceleration={}", static_cast<unsigned int>(message->op), message->linear_velocity, message->angular_velocity, message->acceleration);
    bool publish_zero = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      publish_zero = active_;
      clearControlStateLocked();
    }
    if (publish_zero) {publishZero();}
    return;
  }
  if (message->op == byd_custom_msgs::msg::ChassisControl::OP_FORWARD || message->op == byd_custom_msgs::msg::ChassisControl::OP_BACKWARD) {
    command.linear_velocity = message->op == byd_custom_msgs::msg::ChassisControl::OP_FORWARD ? message->linear_velocity : -message->linear_velocity;
    command.linear_acceleration = message->acceleration > 0.0F ? std::min(static_cast<double>(message->acceleration), config_.max_linear_acceleration) : config_.default_linear_acceleration;
    command.angular_acceleration = config_.default_angular_acceleration;
  } else if (message->op == byd_custom_msgs::msg::ChassisControl::OP_TURN_LEFT || message->op == byd_custom_msgs::msg::ChassisControl::OP_TURN_RIGHT) {
    command.angular_velocity = message->op == byd_custom_msgs::msg::ChassisControl::OP_TURN_LEFT ? message->angular_velocity : -message->angular_velocity;
    command.linear_acceleration = config_.default_linear_acceleration;
    command.angular_acceleration = message->acceleration > 0.0F ? std::min(static_cast<double>(message->acceleration), config_.max_angular_acceleration) : config_.default_angular_acceleration;
  } else {
    LOG_ERROR("拒绝未知 ChassisControl op={}", static_cast<unsigned int>(message->op));
    bool publish_zero = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      publish_zero = active_;
      clearControlStateLocked();
    }
    if (publish_zero) {publishZero();}
    return;
  }
  command.linear_velocity = std::clamp(command.linear_velocity, -config_.max_linear_velocity, config_.max_linear_velocity);
  command.angular_velocity = std::clamp(command.angular_velocity, -config_.max_angular_velocity, config_.max_angular_velocity);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      LOG_WARN("忽略 Lifecycle 未激活时收到的 ChassisControl");
      return;
    }
    if (requiresStopBeforeSwitchLocked(command)) {
      pending_command_ = command;
      has_pending_command_ = true;
      target_command_.linear_velocity = 0.0;
      target_command_.angular_velocity = 0.0;
      target_command_.linear_acceleration = command.linear_acceleration;
      target_command_.angular_acceleration = command.angular_acceleration;
      linear_integral_ = 0.0;
      angular_integral_ = 0.0;
      LOG_INFO("ChassisControl 方向切换，先闭环减速到零，pending_op={}", static_cast<unsigned int>(command.operation));
    } else {
      applyTargetLocked(command);
      has_pending_command_ = false;
    }
    has_command_ = true;
  }
  LOG_DEBUG("收到 ChassisControl：op={}，linear_velocity={}，angular_velocity={}，acceleration={}", static_cast<unsigned int>(message->op), message->linear_velocity, message->angular_velocity, message->acceleration);
  processControlCommand();
}

void ChassisControlSubscriber::activate() {
  motion_state_subscriber_.reset();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    clearControlStateLocked();
    active_ = true;
    last_control_time_ = std::chrono::steady_clock::now();
  }
  LOG_INFO("ChassisControl 事件驱动闭环已激活，output_topic={}", config_.output_topic);
}

void ChassisControlSubscriber::deactivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  clearControlStateLocked();
  LOG_INFO("ChassisControl 事件驱动闭环已停用");
}

void ChassisControlSubscriber::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  clearControlStateLocked();
  motion_state_subscriber_.reset();
}

void ChassisControlSubscriber::processControlCommand() {
  const auto now = std::chrono::steady_clock::now();
  const auto state = motion_state_subscriber_.latestState();
  bool publish_zero = false;
  bool publish_control = false;
  double linear_output = 0.0;
  double angular_output = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {return;}
    const auto publisher_count = node_.count_publishers(config_.output_topic);
    if (publisher_count > 1U) {
      if (!publisher_conflict_) {LOG_ERROR("检测到 {} 个 {} Publisher，停止 ChassisControl 闭环输出", publisher_count, config_.output_topic);}
      publisher_conflict_ = true;
      clearControlStateLocked();
      return;
    }
    if (publisher_conflict_) {
      publisher_conflict_ = false;
      LOG_INFO("{} Publisher 冲突已解除，等待新的 ChassisControl", config_.output_topic);
    }
    if (!has_command_) {return;}
    const bool state_timed_out = !state.valid || std::chrono::duration<double>(now - state.receive_time).count() > config_.motion_state_timeout;
    if (state_timed_out) {
      if (!motion_state_fault_) {LOG_ERROR("MotionState 超时或无有效反馈，停止闭环并等待新的 ChassisControl");}
      motion_state_fault_ = true;
      clearControlStateLocked();
      publish_zero = true;
    } else {
      if (motion_state_fault_) {LOG_INFO("MotionState 已恢复，等待新的 ChassisControl"); motion_state_fault_ = false;}
      double dt = std::chrono::duration<double>(now - last_control_time_).count();
      if (!std::isfinite(dt) || dt <= 0.0) {dt = 0.02;}
      dt = std::min(dt, 0.1);
      last_control_time_ = now;
      reference_linear_velocity_ = approach(reference_linear_velocity_, target_command_.linear_velocity, target_command_.linear_acceleration * dt);
      reference_angular_velocity_ = approach(reference_angular_velocity_, target_command_.angular_velocity, target_command_.angular_acceleration * dt);
      const double linear_error = reference_linear_velocity_ - state.linear_velocity;
      const double angular_error = reference_angular_velocity_ - state.angular_velocity;
      const double candidate_linear_integral = std::clamp(linear_integral_ + linear_error * dt, -config_.linear_integral_limit, config_.linear_integral_limit);
      const double candidate_angular_integral = std::clamp(angular_integral_ + angular_error * dt, -config_.angular_integral_limit, config_.angular_integral_limit);
      double desired_linear_output = reference_linear_velocity_ + config_.linear_kp * linear_error + config_.linear_ki * candidate_linear_integral;
      double desired_angular_output = reference_angular_velocity_ + config_.angular_kp * angular_error + config_.angular_ki * candidate_angular_integral;
      desired_linear_output = std::clamp(desired_linear_output, -config_.max_linear_velocity, config_.max_linear_velocity);
      desired_angular_output = std::clamp(desired_angular_output, -config_.max_angular_velocity, config_.max_angular_velocity);
      if (reference_linear_velocity_ > 0.0) {desired_linear_output = std::max(0.0, desired_linear_output);} else if (reference_linear_velocity_ < 0.0) {desired_linear_output = std::min(0.0, desired_linear_output);} else {desired_linear_output = 0.0;}
      if (reference_angular_velocity_ > 0.0) {desired_angular_output = std::max(0.0, desired_angular_output);} else if (reference_angular_velocity_ < 0.0) {desired_angular_output = std::min(0.0, desired_angular_output);} else {desired_angular_output = 0.0;}
      output_linear_velocity_ = approach(output_linear_velocity_, desired_linear_output, target_command_.linear_acceleration * dt);
      output_angular_velocity_ = approach(output_angular_velocity_, desired_angular_output, target_command_.angular_acceleration * dt);
      linear_integral_ = candidate_linear_integral;
      angular_integral_ = candidate_angular_integral;
      if (has_pending_command_ && isStoppedLocked(state)) {
        applyTargetLocked(pending_command_);
        has_pending_command_ = false;
        linear_integral_ = 0.0;
        angular_integral_ = 0.0;
        LOG_INFO("底盘已停稳，开始执行 pending ChassisControl，op={}", static_cast<unsigned int>(target_command_.operation));
      }
      linear_output = output_linear_velocity_;
      angular_output = output_angular_velocity_;
      publish_control = true;
      LOG_DEBUG("ChassisControl 闭环：target_v={}，target_w={}，reference_v={}，reference_w={}，feedback_v={}，feedback_w={}，output_v={}，output_w={}", target_command_.linear_velocity, target_command_.angular_velocity, reference_linear_velocity_, reference_angular_velocity_, state.linear_velocity, state.angular_velocity, linear_output, angular_output);
    }
  }
  if (publish_zero) {publishZero();}
  if (publish_control) {publishControl(linear_output, angular_output);}
}

void ChassisControlSubscriber::clearControlStateLocked() {
  target_command_ = TargetCommand{};
  pending_command_ = TargetCommand{};
  has_command_ = false;
  has_pending_command_ = false;
  reference_linear_velocity_ = 0.0;
  reference_angular_velocity_ = 0.0;
  output_linear_velocity_ = 0.0;
  output_angular_velocity_ = 0.0;
  linear_integral_ = 0.0;
  angular_integral_ = 0.0;
}

bool ChassisControlSubscriber::requiresStopBeforeSwitchLocked(const TargetCommand & command) const {
  const bool linear_sign_change = target_command_.linear_velocity * command.linear_velocity < 0.0;
  const bool angular_sign_change = target_command_.angular_velocity * command.angular_velocity < 0.0;
  const bool linear_to_angular = (std::abs(reference_linear_velocity_) > config_.linear_stop_threshold || std::abs(output_linear_velocity_) > config_.linear_stop_threshold) && std::abs(command.angular_velocity) > 0.0;
  const bool angular_to_linear = (std::abs(reference_angular_velocity_) > config_.angular_stop_threshold || std::abs(output_angular_velocity_) > config_.angular_stop_threshold) && std::abs(command.linear_velocity) > 0.0;
  return has_command_ && (linear_sign_change || angular_sign_change || linear_to_angular || angular_to_linear);
}

bool ChassisControlSubscriber::isStoppedLocked(const MotionStateSnapshot & state) const {return std::abs(state.linear_velocity) <= config_.linear_stop_threshold && std::abs(state.angular_velocity) <= config_.angular_stop_threshold && std::abs(reference_linear_velocity_) <= config_.linear_stop_threshold && std::abs(reference_angular_velocity_) <= config_.angular_stop_threshold && std::abs(output_linear_velocity_) <= config_.linear_stop_threshold && std::abs(output_angular_velocity_) <= config_.angular_stop_threshold;}

void ChassisControlSubscriber::applyTargetLocked(const TargetCommand & command) {
  target_command_ = command;
  linear_integral_ = 0.0;
  angular_integral_ = 0.0;
}

void ChassisControlSubscriber::publishControl(const double linear_velocity, const double angular_velocity) {
  byd_custom_msgs::msg::ControlRes output;
  output.v = linear_velocity;
  output.w = angular_velocity;
  output.v_lift = 0.0;
  output.w_rotation = 0.0;
  publisher_->publish(output);
}

void ChassisControlSubscriber::publishZero() {publishControl(0.0, 0.0);}

double ChassisControlSubscriber::approach(const double current, const double target, const double maximum_delta) {
  if (current < target) {return std::min(current + maximum_delta, target);}
  if (current > target) {return std::max(current - maximum_delta, target);}
  return target;
}

}  // namespace nav2_regulated_modules
