#ifndef NAV2_REGULATED_MODULES__CHASSIS_CONTROL_SUBSCRIBER_HPP_
#define NAV2_REGULATED_MODULES__CHASSIS_CONTROL_SUBSCRIBER_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "byd_custom_msgs/msg/chassis_control.hpp"
#include "byd_custom_msgs/msg/control_res.hpp"
#include "nav2_regulated_modules/motion_state_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_modules
{

struct ChassisControlConfig
{
  std::string output_topic{"/control_to_uart"};
  double motion_state_timeout{0.2};
  double max_linear_velocity{0.52};
  double max_angular_velocity{2.0};
  double default_linear_acceleration{0.5};
  double default_angular_acceleration{1.0};
  double max_linear_acceleration{2.5};
  double max_angular_acceleration{3.2};
  double linear_stop_threshold{0.01};
  double angular_stop_threshold{0.05};
  double linear_kp{0.2};
  double linear_ki{0.0};
  double angular_kp{0.2};
  double angular_ki{0.0};
  double linear_integral_limit{0.2};
  double angular_integral_limit{0.5};
};

class ChassisControlSubscriber
{
public:
  ChassisControlSubscriber(nav2_util::LifecycleNode & node, MotionStateSubscriber & motion_state_subscriber, ChassisControlConfig config);
  void activate();
  void deactivate();
  void reset();

private:
  struct TargetCommand
  {
    double linear_velocity{0.0};
    double angular_velocity{0.0};
    double linear_acceleration{0.5};
    double angular_acceleration{1.0};
    uint8_t operation{0};
  };

  void onChassisControl(const byd_custom_msgs::msg::ChassisControl::ConstSharedPtr message);
  void processControlCommand();
  void clearControlStateLocked();
  bool requiresStopBeforeSwitchLocked(const TargetCommand & command) const;
  bool isStoppedLocked(const MotionStateSnapshot & state) const;
  void applyTargetLocked(const TargetCommand & command);
  void publishControl(double linear_velocity, double angular_velocity);
  void publishZero();
  static double approach(double current, double target, double maximum_delta);

  nav2_util::LifecycleNode & node_;
  MotionStateSubscriber & motion_state_subscriber_;
  ChassisControlConfig config_;
  std::mutex mutex_;
  TargetCommand target_command_;
  TargetCommand pending_command_;
  bool active_{false};
  bool has_command_{false};
  bool has_pending_command_{false};
  bool motion_state_fault_{false};
  bool publisher_conflict_{false};
  double reference_linear_velocity_{0.0};
  double reference_angular_velocity_{0.0};
  double output_linear_velocity_{0.0};
  double output_angular_velocity_{0.0};
  double linear_integral_{0.0};
  double angular_integral_{0.0};
  std::chrono::steady_clock::time_point last_control_time_{};

  rclcpp::Subscription<byd_custom_msgs::msg::ChassisControl>::SharedPtr subscription_;
  rclcpp::Publisher<byd_custom_msgs::msg::ControlRes>::SharedPtr publisher_;
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__CHASSIS_CONTROL_SUBSCRIBER_HPP_
