#ifndef NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_
#define NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "byd_custom_msgs/action/navigation_service.hpp"
#include "byd_custom_msgs/msg/navi_segment.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_regulated_modules/control_module.hpp"
#include "nav2_regulated_modules/navigation_state.hpp"
#include "nav2_regulated_modules/planning_module.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "spdlog_wrapper.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_regulated_modules
{

class RegulatedNavigator : public nav2_util::LifecycleNode
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using SmoothPath = nav2_msgs::action::SmoothPath;
  using FollowPath = nav2_msgs::action::FollowPath;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using NavigationService = byd_custom_msgs::action::NavigationService;
  using ClearCostmap = nav2_msgs::srv::ClearEntireCostmap;

  using ComputePoseHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using ComputePosesHandle = rclcpp_action::ClientGoalHandle<ComputePathThroughPoses>;
  using SmoothHandle = rclcpp_action::ClientGoalHandle<SmoothPath>;
  using FollowHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using NavigatePoseHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using NavigatePosesHandle = rclcpp_action::ServerGoalHandle<NavigateThroughPoses>;
  using NavigationServiceHandle = rclcpp_action::ServerGoalHandle<NavigationService>;

  explicit RegulatedNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  rclcpp_action::GoalResponse handlePoseGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::GoalResponse handlePosesGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigateThroughPoses::Goal> goal);
  rclcpp_action::CancelResponse handlePoseCancel(const std::shared_ptr<NavigatePoseHandle> goal);
  rclcpp_action::CancelResponse handlePosesCancel(const std::shared_ptr<NavigatePosesHandle> goal);
  void handlePoseAccepted(const std::shared_ptr<NavigatePoseHandle> goal);
  void handlePosesAccepted(const std::shared_ptr<NavigatePosesHandle> goal);
  void onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
  rclcpp_action::GoalResponse handleNavigationServiceGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigationService::Goal> goal);
  rclcpp_action::CancelResponse handleNavigationServiceCancel(const std::shared_ptr<NavigationServiceHandle> goal);
  void handleNavigationServiceAccepted(const std::shared_ptr<NavigationServiceHandle> goal);
  std::optional<nav_msgs::msg::Path> prepareFixedPath(const std::vector<byd_custom_msgs::msg::NaviSegment> & segments);
  void liner2to4point(const nav_msgs::msg::Path & input_path, nav_msgs::msg::Path & output_path);
  geometry_msgs::msg::PoseStamped bezier3(const nav_msgs::msg::Path & input_path, double t, bool use_result);
  void computeArcLengths(const std::vector<geometry_msgs::msg::PoseStamped> & poses, std::vector<double> & arc_lengths);
  double findTfromArcLength(const std::vector<double> & arc_lengths, const std::vector<double> & ts, double target_length);
  void generateBezierUniformPoints(const nav_msgs::msg::Path & input_path, double interval, nav_msgs::msg::Path & output_path);
  void publishFeedback();

  bool dependenciesReady();
  void startPlanning(bool replanning);
  bool isCurrentPlan(uint64_t generation, uint64_t sequence) const;
  void onPathReady(const nav_msgs::msg::Path & path);
  void handlePlanningFailure(const std::string & reason);

  void sendFollowPath(const nav_msgs::msg::Path & path);
  void publishFixedPath(const nav_msgs::msg::Path & path);
  bool isCurrentFollow(uint64_t generation, uint64_t sequence) const;
  void stopRobot();
  void onControllerVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity);
  void onSmoothedVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity);
  void onVelocityOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry);
  void logVelocityChain();

  void startRecovery(const std::string & reason);
  void resumeCurrentTask();
  void monitorTask();
  bool lookupCurrentPose(geometry_msgs::msg::PoseStamped & pose);
  void updatePassedGoals(const geometry_msgs::msg::PoseStamped & current_pose);

  void cancelSubGoals(bool invalidate_callbacks);
  void cancelTask(const std::string & reason);
  void preemptCurrentTask();
  void succeedTask();
  void failTask(const std::string & reason);
  void resetTask();

  bool configured_{false};
  bool active_{false};
  bool planning_active_{false};
  bool updating_path_{false};
  bool has_last_pose_{false};
  bool cancel_requested_{false};
  uint64_t task_generation_{0};
  uint64_t plan_sequence_{0};
  uint64_t follow_sequence_{0};
  NavigationTask task_;
  NavigationMode operation_mode_{NavigationMode::AUTONOMOUS};
  PlanningModule planning_module_;
  ControlModule control_module_;

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string goal_topic_;
  std::string navigation_service_action_;
  std::string fixed_path_visualization_topic_;
  std::string fixed_path_boundaries_topic_;
  std::string controller_cmd_vel_topic_;
  std::string smoothed_cmd_vel_topic_;
  std::string velocity_odom_topic_;
  double server_timeout_{5.0};
  double cancel_timeout_{2.0};
  double smoothing_duration_{2.0};
  double feedback_frequency_{5.0};
  double costmap_wait_duration_{0.8};
  double passed_goal_radius_{0.7};
  double localization_timeout_{0.3};
  double max_translation_jump_{0.3};
  double max_rotation_jump_{0.35};
  double progress_min_translation_{0.1};
  double localization_recovery_timeout_{10.0};
  double localization_stable_duration_{0.5};
  int max_recovery_rounds_{2};
  bool check_smoother_collisions_{true};
  double current_speed_{0.0};
  double fixed_path_step_{0.1};
  double fixed_path_boundary_half_width_{0.4};
  double velocity_log_frequency_{1.0};

  rclcpp::Time last_valid_tf_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time localization_lost_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time localization_stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_ready_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped last_pose_;
  nav_msgs::msg::Path pending_raw_path_;
  geometry_msgs::msg::Twist latest_controller_velocity_;
  geometry_msgs::msg::Twist latest_smoothed_velocity_;
  nav_msgs::msg::Odometry latest_velocity_odometry_;
  std::mutex velocity_mutex_;
  bool has_controller_velocity_{false};
  bool has_smoothed_velocity_{false};
  bool has_velocity_odometry_{false};

  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_pose_client_;
  rclcpp_action::Client<ComputePathThroughPoses>::SharedPtr compute_poses_client_;
  rclcpp_action::Client<SmoothPath>::SharedPtr smooth_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_pose_server_;
  rclcpp_action::Server<NavigateThroughPoses>::SharedPtr navigate_poses_server_;
  rclcpp_action::Server<NavigationService>::SharedPtr navigation_service_server_;
  rclcpp::Client<ClearCostmap>::SharedPtr clear_local_client_;
  rclcpp::Client<ClearCostmap>::SharedPtr clear_global_client_;

  ComputePoseHandle::SharedPtr active_compute_pose_goal_;
  ComputePosesHandle::SharedPtr active_compute_poses_goal_;
  SmoothHandle::SharedPtr active_smooth_goal_;
  FollowHandle::SharedPtr active_follow_goal_;
  std::shared_ptr<NavigatePoseHandle> active_pose_goal_;
  std::shared_ptr<NavigatePosesHandle> active_poses_goal_;
  std::shared_ptr<NavigationServiceHandle> active_navigation_service_goal_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr smoothed_velocity_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr fixed_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr fixed_path_boundaries_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::TimerBase::SharedPtr velocity_log_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_
