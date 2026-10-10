// 固定路径控制器接口。负责起点对齐、路径跟踪、终点位置停车及前进／后退方向相关的速度计算。

#ifndef NAV2_REGULATED_MODULES__FIXED_PATH_CONTROLLER_HPP_
#define NAV2_REGULATED_MODULES__FIXED_PATH_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_regulated_modules/detail/fixed_path_start_speed_guard.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/terminal_aware_controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_regulated_modules
{

class FixedPathController : public nav2_core::Controller, public nav2_core::TerminalAwareController
{
  public:
// 构造控制器；插件资源与路径状态在 configure 和 setPlan 阶段建立。
  FixedPathController() = default;
  // 说明 ~FixedPathController 对应的接口行为；调用方需遵守该类的任务状态约束。
  ~FixedPathController() override = default;

  // 读取参数并建立控制器运行所需的 ROS 接口或插件资源。
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  // 释放插件资源并复位控制器内部阶段与路径状态。
  void cleanup() override;
  // 记录插件激活事件；速度指令仍由 computeVelocityCommands 计算。
  void activate() override;
  // 将控制阶段设为停止并锁存停车状态，不在此函数直接发布速度。
  void deactivate() override;
  // 装入新的跟踪路径；重置此前路径索引、终点锁存及进度数据。
  void setPlan(const nav_msgs::msg::Path & path) override;
  // 根据当前位姿与路径阶段生成速度指令；各安全边界可使输出归零。
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker) override;
  // 更新控制器允许速度；区分百分比限制与绝对速度限制。
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  // 读取终点停车锁存状态；锁存后不再输出继续跟踪命令。
  bool isTerminalStopLatched() override;
  // 只检查终点位置精度，供任务成功判定使用。
  bool isTerminalPositionAccurate() override;

private:
  enum class Phase
  {
    ALIGN_START, TRACK_PATH, STOPPED
  };

  // 把输入位姿通过 TF 转到目标坐标系；转换失败时返回 false。
  bool transformPose(const std::string & frame, const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output) const;
  // 从当前路径搜索区间选出距机器人最近的轨迹点索引。
  std::size_t findNearestIndex(const geometry_msgs::msg::PoseStamped & robot_pose);
  // 从最近路径点累计至终点的剩余行程，单位米。
  double remainingDistance(std::size_t start_index, const geometry_msgs::msg::PoseStamped & robot_pose) const;
  // 按前视距离选择路径目标点，必要时对末段进行受限延长。
  geometry_msgs::msg::PoseStamped selectCarrot(std::size_t start_index, double lookahead_distance) const;
  // 构造全零 Twist，统一用于停车或无有效路径场景。
  geometry_msgs::msg::TwistStamped zeroCommand() const;
  // 将航向误差换算为受角速度与加速度约束的原地转向指令。
  geometry_msgs::msg::TwistStamped rotateCommand(double yaw_error, double yaw_tolerance, double max_angular_velocity, double max_angular_accel, double angular_kp, const geometry_msgs::msg::Twist & velocity) const;
  // 按设定频率记录终点位置和航向误差；不改变停车判定。
  void logGoalErrors(double position_error, double yaw_error, double goal_xy_tolerance, double terminal_projection, bool goal_plane_crossed, double remaining_distance, double current_linear_velocity, double target_linear_velocity, double stopping_distance, double terminal_lookahead_distance);
  // 返回当前控制阶段名称，供诊断日志辨识。
  static const char * phaseName(Phase phase);
  // 将角度规约到约定区间，避免跨越正负 π 时误判。
  static double normalizeAngle(double angle);
  // 从姿态四元数提取平面航向角，单位弧度。
  static double poseYaw(const geometry_msgs::msg::PoseStamped & pose);
  // 计算两帧位姿的平面距离，单位米。
  static double poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ { rclcpp::get_logger("FixedPathController") };
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;
  std::vector<detail::StartPathPoint> start_path_points_;
  detail::FixedPathStartSpeedGuard start_speed_guard_;
  std::mutex mutex_;
  Phase phase_ { Phase::ALIGN_START };
  std::size_t nearest_index_ { 0 };
  std::size_t goal_tangent_index_ { 0 };
  int direction_sign_ { 1 };
  int stable_cycles_ { 0 };
  bool start_strategy_evaluated_ { false };
  bool direct_start_tracking_ { false };
  bool goal_braking_active_ { false };
  bool terminal_stop_latched_ { false };
  bool terminal_position_accurate_ { false };
  double start_path_yaw_ { 0.0 };
  double goal_path_yaw_ { 0.0 };
  double terminal_tangent_x_ { 0.0 };
  double terminal_tangent_y_ { 0.0 };
  double base_linear_velocity_ { 1.5 };
  double speed_limit_ { 1.5 };
  double lookahead_dist_ { 0.45 };
  double min_lookahead_dist_ { 0.25 };
  double max_lookahead_dist_ { 0.75 };
  double lookahead_time_ { 1.5 };
  double start_position_tolerance_ { 1.20 };
  double direct_tracking_lateral_tolerance_ { 0.20 };
  double start_offset_speed_limit_ = 0.30;
  double start_speed_limit_distance_ = 0.50;
  bool start_speed_prebraking_ = false;
  double start_speed_release_yaw_tolerance_ = 0.3490658503988659;
  int start_speed_release_stable_cycles_ = 10;
  double direct_tracking_max_yaw_error_ { 0.2617993877991494 };
  double initial_yaw_tolerance_ { 0.05235987755982989 };
  double rotate_to_heading_angular_vel_ { 0.4 };
  double max_angular_accel_ { 0.8 };
  double rotate_to_heading_kp_ { 1.5 };
  double min_approach_linear_velocity_ { 0.005 };
  double approach_velocity_scaling_dist_ { 0.8 };
  double goal_linear_deceleration_ { 0.25 };
  double goal_final_approach_velocity_ { 0.01 };
  double goal_stop_entry_tolerance_ { 0.0 };
  double goal_braking_reaction_time_ { 0.1 };
  double goal_braking_distance_margin_ { 0.1 };
  bool dynamic_goal_braking_margin_enabled_ = false;
  double goal_braking_min_distance_margin_ = 0.03;
  double goal_braking_margin_transition_speed_ = 0.3;
  // 新制动策略默认关闭，避免未经验证的速度曲线直接进入实车速度链。
  bool adaptive_goal_braking_enabled_ { false };
  double adaptive_goal_max_deceleration_ { 1.0 };
  double adaptive_goal_jerk_limit_ { 2.0 };
  double adaptive_goal_approach_speed_ { 0.03 };
  double adaptive_goal_response_time_ { 0.5 };
  double adaptive_goal_distance_margin_ { 0.01 };
  double goal_terminal_lookahead_dist_ { 0.2 };
  double goal_terminal_lookahead_reference_speed_ { 0.75 };
  double goal_terminal_lookahead_speed_gain_ { 0.1 };
  double goal_terminal_lookahead_min_dist_ { 0.15 };
  double goal_terminal_lookahead_max_dist_ { 0.25 };
  double terminal_tracking_yaw_error_ { 0.0 };
  double last_braking_command_magnitude_ { 0.0 };
  double last_braking_deceleration_ { 0.0 };
  rclcpp::Time last_braking_update_time_ { 0, 0, RCL_ROS_TIME };
  double goal_error_log_frequency_ { 1.0 };
  double transform_tolerance_ { 0.2 };
  double control_duration_ { 0.01 };
  int alignment_stable_cycles_ { 1 };
  rclcpp::Time last_error_log_time_ { 0, 0, RCL_ROS_TIME };
  bool error_log_initialized_ { false };
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__FIXED_PATH_CONTROLLER_HPP_
