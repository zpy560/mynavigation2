#ifndef NAV2_REGULATED_MODULES__MOTION_STATE_SUBSCRIBER_HPP_
#define NAV2_REGULATED_MODULES__MOTION_STATE_SUBSCRIBER_HPP_

#include <chrono>
#include <mutex>

#include "byd_custom_msgs/msg/motion_state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_modules
{

struct MotionStateSnapshot
{
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  std::chrono::steady_clock::time_point receive_time{};
  bool valid{false};
};

class MotionStateSubscriber
{
public:
  explicit MotionStateSubscriber(nav2_util::LifecycleNode & node);
  MotionStateSnapshot latestState() const;
  void reset();

private:
  void onMotionState(const byd_custom_msgs::msg::MotionState::ConstSharedPtr message);

  mutable std::mutex mutex_;
  MotionStateSnapshot latest_state_;
  rclcpp::Subscription<byd_custom_msgs::msg::MotionState>::SharedPtr subscription_;
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__MOTION_STATE_SUBSCRIBER_HPP_
