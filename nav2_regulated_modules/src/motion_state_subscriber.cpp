#include "nav2_regulated_modules/motion_state_subscriber.hpp"

#include <cmath>
#include <functional>

#include "spdlog_wrapper.hpp"

namespace nav2_regulated_modules
{

MotionStateSubscriber::MotionStateSubscriber(nav2_util::LifecycleNode & node) {
  subscription_ = node.create_subscription<byd_custom_msgs::msg::MotionState>("/motion_state", rclcpp::SensorDataQoS(), std::bind(&MotionStateSubscriber::onMotionState, this, std::placeholders::_1));
}

void MotionStateSubscriber::onMotionState(const byd_custom_msgs::msg::MotionState::ConstSharedPtr message) {
  if (!std::isfinite(message->v_car) || !std::isfinite(message->w_car)) {
    LOG_WARN("忽略非法 MotionState：v_car={}，w_car={}", message->v_car, message->w_car);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_state_.linear_velocity = message->v_car;
    latest_state_.angular_velocity = message->w_car;
    latest_state_.receive_time = std::chrono::steady_clock::now();
    latest_state_.valid = true;
  }
  LOG_DEBUG("收到 MotionState：v_car={}，w_car={}，v_lift={}，lift_height={}，w_shelf={}，yaw_shelf={}", message->v_car, message->w_car, message->v_lift, message->lift_height, message->w_shelf, message->yaw_shelf);
}

MotionStateSnapshot MotionStateSubscriber::latestState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_state_;
}

void MotionStateSubscriber::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_state_ = MotionStateSnapshot{};
}

}  // namespace nav2_regulated_modules
