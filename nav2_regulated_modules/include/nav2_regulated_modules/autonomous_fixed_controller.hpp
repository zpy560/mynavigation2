#ifndef NAV2_REGULATED_MODULES__AUTONOMOUS_FIXED_CONTROLLER_HPP_
#define NAV2_REGULATED_MODULES__AUTONOMOUS_FIXED_CONTROLLER_HPP_

#include <optional>
#include <chrono>
#include "nav2_regulated_modules/fixed_path_controller.hpp"
#include "nav2_regulated_modules/detail/autonomous_fixed_path.hpp"
#include "nav2_regulated_modules/detail/autonomous_fixed_localization.hpp"
#include "nav2_regulated_modules/detail/autonomous_fixed_terminal_review.hpp"
#include "nav2_regulated_modules/detail/autonomous_fixed_terminal_trim.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "byd_custom_msgs/msg/motion_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_regulated_modules
{
// 单向子路径的控制包装；原 FixedPathController 的算法和参数命名空间保持独立。
class AutonomousFixedPathController : public nav2_core::Controller, public nav2_core::TerminalAwareController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed, const bool & percentage) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker) override;
  bool isTerminalStopLatched() override;
  bool isTerminalPositionAccurate() override;
  double getTerminalStableDuration() const override
  {
    // 本包装独立连续验收，禁用服务器 2 秒后宽松成功的分支。
    return 0.0;
  }

private:
  void status(const std::string & phase);
  bool pauseForLocalization(bool valid, const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured, double stamp, double steady_time);
  geometry_msgs::msg::Twist limitResumeCommand(const geometry_msgs::msg::Twist & command, double steady_time);
  void logTerminalReview(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured, double xy, double yaw, double xy_tolerance, bool stationary, double tf_stamp, double tf_age, double odom_age, double receive_age);
  bool trajectoryFree(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured, const geometry_msgs::msg::Twist & command);
  bool trimPoseAllowed(double x, double y) const;
  geometry_msgs::msg::Twist trimCommand(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured, double xy_tolerance);
  FixedPathController tracker_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::SubscriptionBase::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry odom_;
  std::chrono::steady_clock::time_point odom_received_;
  bool has_odom_ = false;
  std::mutex mutex_;
  nav_msgs::msg::Path path_;
  std::size_t cursor_ = 0;
  bool rotation_only_ = false;
  bool ready_ = false;
  bool tracker_loaded_ = false;
  bool terminal_ = false;
  detail::FixedTerminalReview terminal_review_;
  detail::FixedTrimState trim_;
  detail::FixedTrimLimits trim_limits_;
  int trim_direction_ = 0;
  bool trim_paused_ = false;
  std::optional<double> trim_confirm_stamp_;
  std::optional<double> trim_resume_since_;
  double trim_confirm_duration_ = 0.2;
  double trim_speed_limit_ = 0.02;
  double base_linear_speed_ = 0.5;
  std::optional<double> blocked_;
  std::optional<double> clear_;
  double period_ = 0.01;
  double settle_duration_ = 0.5;
  double brake_stable_duration_ = 0.2;
  double terminal_review_timeout_ = 2.0;
  double terminal_capture_margin_ = 0.02;
  double max_angular_ = 0.4;
  double max_accel_ = 0.8;
  double obstacle_timeout_ = 60.0;
  double localization_timeout_ = 0.3;
  double tracking_localization_timeout_ = 0.6;
  double terminal_localization_distance_ = 0.5;
  double short_pause_timeout_ = 0.5;
  double resume_stable_duration_ = 0.15;
  double resume_distance_ = 0.2;
  double resume_linear_acceleration_ = 0.5;
  bool localization_hard_wait_ = false;
  std::string localization_hard_reason_;
  detail::FixedLocalizationPauseTiming localization_pause_;
  std::optional<double> localization_fresh_since_;
  double localization_first_fresh_stamp_ = 0.0;
  double localization_latest_stamp_ = 0.0;
  unsigned int localization_fresh_frames_ = 0;
  std::optional<geometry_msgs::msg::PoseStamped> last_localized_pose_;
  double last_localized_time_ = 0.0;
  double last_localized_stamp_ = 0.0;
  bool resume_ramp_active_ = false;
  geometry_msgs::msg::Twist resume_command_;
  double resume_command_time_ = 0.0;
  std::string localization_reason_ = "NONE";
  std::string last_status_reason_;
  int64_t diagnostic_tf_stamp_ = 0;
  double diagnostic_tf_age_ = 1e9;
  double diagnostic_odom_age_ = 1e9;
  double diagnostic_receive_age_ = 1e9;
  double diagnostic_pause_duration_ = 0.0;
  double collision_log_time_ = -1.0;
  double health_log_time_ = -1.0;
  std::string collision_reason_;
  double last_status_time_ = -1.0;
  std::string last_phase_;
  std::string terminal_log_phase_;
  double terminal_log_time_ = -1.0;
};
}
#endif
