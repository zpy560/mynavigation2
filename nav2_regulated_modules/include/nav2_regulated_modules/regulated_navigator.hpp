// 无 BT 规控协调器的声明。入口 Action、规划／平滑／跟踪客户端、定位监控与底盘支路由生命周期统一管理。
// remote／autonomous／fixed_path 在启动时选择；各入口按模式校验任务，发送另一类 Action 不会自动切换模式。

#ifndef NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_
#define NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_

#include <cstdint>
#include <chrono>
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
#include "nav2_regulated_modules/chassis_control_subscriber.hpp"
#include "nav2_regulated_modules/control_module.hpp"
#include "nav2_regulated_modules/motion_state_subscriber.hpp"
#include "nav2_regulated_modules/navigation_state.hpp"
#include "nav2_regulated_modules/planning_module.hpp"
#include "nav2_regulated_modules/tf_diagnostics.hpp"
#include "nav2_regulated_modules/detail/autonomous_fixed_path.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/collision_state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
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

  // 仅声明运行参数；ROS 实体在 configure 建立，业务入口在 activate 后才允许接收。
  // 构造中的 false 等值是节点回退，不覆盖 Launch 从项目 YAML 注入的配置。
  explicit RegulatedNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 读取并校验配置，再创建客户端、Action 服务、订阅与定时器。
  // NavigationService 仅在 fixed_path 建立；标准自主入口虽已建立，接受回调仍检查运行模式。
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  // 激活固定路径可视化与底盘支路，重置 collision 接收历史，再开放业务入口。
  // 这只表示生命周期就绪，不生成任务，也不证明定位／轮速已经新鲜。
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  // 先关闭业务许可，再停用底盘定时器并请求取消导航任务。
  // 固定路径可视化停用、bond 销毁与实际机械停稳是不同事件。
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  // 释放配置阶段创建的 ROS 句柄、反馈缓存与 TF 对象。
  // cleanup 是资源清理，不是控制权移交或完整的下游停稳等待。
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  // 停用底盘支路并调用任务取消流程；是否发零速取决于当前任务状态。
  // 不能仅凭本回调返回 SUCCESS 推断 UART 和轮速已完成停车。
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  void beginFixedAutonomy();
  void monitorFixedAutonomy();
  void freezeAutonomousPath(const nav_msgs::msg::Path & path);
  void followFrozenSegment(bool resume);
  void finishFrozenSegment();
  void onFixedExecutionStatus(const std_msgs::msg::String::SharedPtr message);
  bool fixedVelocityStopped();
  nav_msgs::msg::Path frozen_path_;
  std::vector<detail::FixedSegment> frozen_segments_;
  std::size_t frozen_segment_ = 0;
  std::size_t frozen_cursor_ = 0;
  std::size_t frozen_dispatch_first_ = 0;
  uint64_t frozen_fingerprint_ = 0;
  bool frozen_plan_requested_ = false;
  bool frozen_follow_inflight_ = false;
  bool frozen_localization_wait_ = false;
  std::size_t frozen_outstanding_follows_ = 0;
  int64_t frozen_execution_id_ = 0;
  int64_t frozen_last_dispatch_id_ = 0;
  std::string frozen_phase_;
  double frozen_wait_since_ = 0.0;
  double frozen_status_since_ = 0.0;
  double frozen_obstacle_since_ = -1.0;
  std::optional<double> frozen_inhibited_since_;
  std::optional<double> frozen_stable_since_;
  std::optional<std::chrono::steady_clock::time_point> frozen_soft_pause_since_;
  std::string frozen_localization_reason_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr frozen_status_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr frozen_path_pub_;

  // 仅接受 autonomous 且激活的有效单点 Pose；behavior_tree 非空会拒绝。
  // 这里只校验请求，规划发生在接受后的处理函数，不通过此函数切换启动模式。
  rclcpp_action::GoalResponse handlePoseGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigateToPose::Goal> goal);
  // 逐点检查非空目标序列的有效性，并拒绝行为树 XML 请求。
  // 多点任务的目标数量不等于已经计算出的路径点数量。
  rclcpp_action::GoalResponse handlePosesGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigateThroughPoses::Goal> goal);
  // 仅当句柄属于当前单点任务时置取消标记，随后由监控周期执行取消流程。
  // 返回 ACCEPT 表示接受取消请求，不是下游 FollowPath 的终态或停稳确认。
  rclcpp_action::CancelResponse handlePoseCancel(const std::shared_ptr<NavigatePoseHandle> goal);
  // 仅为当前多点任务设置取消标记，避免旧客户端取消新任务。
  // 该回调本身不等待规划、平滑或控制子 Goal 结束。
  rclcpp_action::CancelResponse handlePosesCancel(const std::shared_ptr<NavigatePosesHandle> goal);
  // 先抢占现有任务，再递增 generation，保存目标并启动规划。
  // 旧子 Goal 以异步方式取消，旧回调靠代次过滤，不把句柄重置视为取消握手完成。
  void handlePoseAccepted(const std::shared_ptr<NavigatePoseHandle> goal);
  // 将多点目标复制到任务快照，末点作为最终目标，再发送多点规划请求。
  // 后续已通过目标可被移除，原始输入序列与当前待办序列应区分。
  void handlePosesAccepted(const std::shared_ptr<NavigatePosesHandle> goal);
  // 将合法 goal_pose 纳入同一自主任务状态机，但没有对应上层 Action 结果句柄。
  // 非 autonomous 或未激活时忽略；接管同样递增任务代次。
  void onTopicGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
  // 仅允许 fixed_path、节点激活、分段非空、全段同向且 max_speed 有限为正。
  // 几何连续性检查在接受后的 prepareFixedPath，故部分几何错误会以已接受后 ABORTED 结束。
  rclcpp_action::GoalResponse handleNavigationServiceGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigationService::Goal> goal);
  // 当前固定路径句柄才设置 cancel_requested，监控周期随后处理。
  // 返回 ACCEPT 不表示下游取消完成，finish=false 也不能替代轮速停稳证据。
  rclcpp_action::CancelResponse handleNavigationServiceCancel(const std::shared_ptr<NavigationServiceHandle> goal);
  // 构造几何路径、等待 FollowPath，再接管旧任务并计算总长与最小段限速。
  // 车头姿态由切线和方向生成，当前定位不被重置；超过全局速度上限时钳位，不因此自动拒绝 Goal。
  void handleNavigationServiceAccepted(const std::shared_ptr<NavigationServiceHandle> goal);
  // 检查有限坐标、非重合端点、相邻连接与段类型，再为直线／贝塞尔生成统一采样。
  // 路径位于 global_frame，输出 z=0；按整条方向编码车头姿态，不在这里进行全程碰撞可行性验证。
  std::optional<nav_msgs::msg::Path> prepareFixedPath(const std::vector<byd_custom_msgs::msg::NaviSegment> & segments);
  // 将两端点扩成共线的四控制点，两个内点分别位于线段的三分之一和三分之二。
  // 因此同一贝塞尔采样器也能处理直线，函数名沿用既有拼写。
  void liner2to4point(const nav_msgs::msg::Path & input_path, nav_msgs::msg::Path & output_path);
  // 以 t∈[0,1] 对四控制点求三次贝塞尔 XY 位置，z 固定为 0。
  // use_result=false 时返回单位姿态供弧长估计；true 时由一阶导数求切线航向。调用方保证四个控制点和参数有效。
  geometry_msgs::msg::PoseStamped bezier3(const nav_msgs::msg::Path & input_path, double t, bool use_result);
  // 按相邻离散点二维距离累计弧长，单位 m，首点累计值为 0。
  // 这是离散近似，不是三次曲线解析弧长。
  void computeArcLengths(const std::vector<geometry_msgs::msg::PoseStamped> & poses, std::vector<double> & arc_lengths);
  // 在累计弧长表中定位目标长度，再在线性邻区间反查 t。
  // 首尾边界钳位到参数范围，重复长度区间不能用作有效除数。
  double findTfromArcLength(const std::vector<double> & arc_lengths, const std::vector<double> & ts, double target_length);
  // 先密集采样估计累计弧长，再按 interval（m）反查 t 输出近似等距路径。
  // 保留精确末点并合并近重复邻点；等距指空间弧长，不是控制器时间间隔。
  void generateBezierUniformPoints(const nav_msgs::msg::Path & input_path, double interval, nav_msgs::msg::Path & output_path);
  // 查询当前位姿，为普通单点／多点 Action 计算时间、ETA 和剩余目标数。
  // 固定路径进度另在 FollowPath 反馈回调发布，不能把此定时器频率当作它的固定反馈频率。
  void publishFeedback();
  // 为固定路径发布 percentage=false 的正绝对限速，单位 m/s。
  // 倒车方向在 Path 姿态和控制器符号中表达，不能发布负 SpeedLimit 代替方向。
  void publishSpeedLimit();

  // 按 server_timeout 等待规划、可选平滑及 FollowPath 服务发现。
  // 服务可发现只证明通信入口存在，不证明地图、TF 或底盘反馈有效。
  bool dependenciesReady();
  // 仅在 autonomous、任务有效且没有在途规划时提交请求。
  // generation 标识任务，plan_sequence 标识该任务的规划轮次；两者均匹配才接受回调。
  void startPlanning(bool replanning);
  // 同时比较 generation 和规划 sequence，拒绝旧任务或旧规划轮次的异步结果。
  // 它不检查机器人位置或路径质量，只承担回调归属判断。
  bool isCurrentPlan(uint64_t generation, uint64_t sequence) const;
  // 保存原始规划路径，按 use_smoother 决定是否发 SmoothPath。
  // 平滑拒绝、失败或返回空路径时回退原始 Path；不把回退当成平滑已成功。
  void onPathReady(const nav_msgs::msg::Path & path);
  // 累计连续规划失败；允许的周期重规划失败可继续旧 FollowPath。
  // 不满足继续条件时进入恢复，而不是在本函数直接改变规划算法。
  void handlePlanningFailure(const std::string & reason);

  // 按任务类别选择控制器与 GoalChecker，再异步发送完整 Path。
  // FollowPath 的响应、反馈和结果均校验 generation／follow_sequence；固定路径绝对限速在接受后发布。
  void sendFollowPath(const nav_msgs::msg::Path & path);
  // 发布 Path 及左右边界、端点等 Marker，帮助观察固定路径几何。
  // 边界宽度是显示参数，不是碰撞检测区，也不限制实际行驶误差。
  void publishFixedPath(const nav_msgs::msg::Path & path);
  // 比较任务代次与跟踪轮次；同一任务换路径也会使旧 sequence 失效。
  // 不能只比较 generation，否则周期重规划的旧跟踪结果可能干扰当前路径。
  bool isCurrentFollow(uint64_t generation, uint64_t sequence) const;
  // 向速度链入口连续发布三帧零 Twist，增加停车请求被下游收到的机会。
  // 不检查发布确认或实际轮速；若没有发布器直接返回，不承诺已经机械停稳。
  void stopRobot();
  // 在 velocity_mutex 保护下缓存控制器最新 Twist 与曾接收标记。
  // 缓存用于日志，不代表与平滑器反馈来自严格同一控制周期。
  void onControllerVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity);
  // 记录平滑后 Twist 供诊断；不再对速度限幅或二次发布。
  // 旧快照可持续存在，消费者应另行理解采样新鲜度。
  void onSmoothedVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity);
  // 缓存速度反馈，供速度观测和 autonomous_fixed 连续停稳判断；不作为地图定位。
  void onVelocityOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry);
  void onVelocityMotionState(const byd_custom_msgs::msg::MotionState::SharedPtr motion);
  // 先在锁内复制三路快照，再在锁外写日志，减少回调持锁时间。
  // 没有任何样本时不打印；独立订阅的 latest 值不能当作同步传感器帧。
  void logVelocityChain();

  // 在恢复轮次允许时取消子 Goal、发送零速并请求清空代价地图。
  // 清图请求异步发送；等待时间到后再尝试恢复，不在此确认清图或实际停车完成。
  void startRecovery(const std::string & reason);
  // 固定路径复用保存的 active_path，自主任务重新进入规划。
  // 是否还属于当前有效任务由 task_ 与上层状态机决定；不从已结束任务恢复。
  void resumeCurrentTask();
  // 依次处理取消、清图等待、TF 丢失／恢复、可选跳变、进度及自主重规划。
  // collision 仅暂停固定路径无进展计时，不跳过前面的取消与定位处理。
  void monitorTask();
  // 自主模式同时检查 TF 源时间戳；固定路径保留最近变换查询与本机成功计时。
  // 新鲜定位仍不等于独立物理精度验收。
  bool lookupCurrentPose(geometry_msgs::msg::PoseStamped & pose);
  // 多点任务中按 passed_goal_radius（m）移除已接近的前置目标，至少保留末点。
  // 这是目标队列维护，不是 FollowPath 成功判定。
  void updatePassedGoals(const geometry_msgs::msg::PoseStamped & current_pose);

  // 对已持有句柄发送 async_cancel_goal，再重置本地句柄；可选递增轮次使旧回调失效。
  // 未等待取消响应／结果，不能称为下游全部终止；尚未拿到句柄的请求也不由这些分支直接取消。
  void cancelSubGoals(bool invalidate_callbacks);
  // 进入取消状态、废弃旧回调、请求停车，再按上层句柄状态返回 canceled 或 abort。
  // 只有客户端正在取消的句柄报告 canceled；一般内部取消走 abort，随后清空任务。
  void cancelTask(const std::string & reason);
  // 新任务接管前使旧规划／跟踪轮次失效，发出取消与停车请求并 abort 旧上层任务。
  // 该流程不等待机械停稳，不应在注释中描述为已经完成安全控制权握手。
  void preemptCurrentTask();
  // 向当前对应 Action 报告成功；固定路径另发 progress=1、finish=true 后清理任务。
  // 上层成功由下游结果触发，不直接证明外部精度验收、定位覆盖或真实底盘制动距离合格。
  void succeedTask();
  // 废弃子 Goal 并向现有上层句柄报告失败，再清理任务快照。
  // 此函数自身没有调用 stopRobot；停车效果不能从函数名或本地状态归零推断。
  void failTask(const std::string & reason);
  // 清除任务与诊断历史；固定路径结束时发布 0 SpeedLimit 清除该任务基准。
  // 0 在插件接口中的语义与 collision_task_speed 清理需区分，不代表独立停车命令。
  void resetTask();

  // 业务状态与生命周期不是两个独立控制源：active_ 是上层回调放行条件，任务阶段另由 task_ 表示。
  bool active_ { false };
  bool planning_active_ { false };
  bool updating_path_ { false };
  bool has_last_pose_ { false };
  bool cancel_requested_ { false };
  bool collision_state_enabled_ = false;
  nav2_util::CollisionState collision_state_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr collision_state_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr collision_params_handler_;
  bool collision_progress_paused_ = false;
  // 代次／轮次只用于当前异步回调归属，不是 ROS Action UUID，也不提供分布式取消确认。
  uint64_t task_generation_ { 0 };
  uint64_t plan_sequence_ { 0 };
  uint64_t follow_sequence_ { 0 };
  NavigationTask task_;
  NavigationMode operation_mode_ { NavigationMode::AUTONOMOUS };
  std::unique_ptr<MotionStateSubscriber> motion_state_subscriber_;
  std::unique_ptr<ChassisControlSubscriber> chassis_control_subscriber_;
  PlanningModule planning_module_;
  ControlModule control_module_;

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string goal_topic_;
  std::string navigation_service_action_;
  std::string fixed_path_visualization_topic_;
  std::string fixed_path_boundaries_topic_;
  std::string fixed_path_controller_id_;
  std::string fixed_path_goal_checker_id_;
  std::string controller_cmd_vel_topic_;
  std::string smoothed_cmd_vel_topic_;
  std::string velocity_odom_topic_;
  std::string speed_limit_topic_;
  // 下列超时／周期为 s；距离阈值为 m，角度跳变阈值为 rad，频率字段为 Hz。
  double server_timeout_ { 5.0 };
  double cancel_timeout_ { 2.0 };
  double smoothing_duration_ { 2.0 };
  double feedback_frequency_ { 5.0 };
  double costmap_wait_duration_ { 0.8 };
  double passed_goal_radius_ { 0.7 };
  double localization_timeout_ { 0.3 };
  bool enable_localization_jump_detection_ { false };
  double max_translation_jump_ { 0.3 };
  double max_rotation_jump_ { 0.35 };
  double progress_min_translation_ { 0.1 };
  double autonomous_progress_min_rotation_ = 0.2;
  double autonomous_goal_progress_distance_ = 0.01;
  std::optional<double> autonomous_progress_heading_error_;
  double autonomous_progress_goal_distance_ = 0.0;
  bool autonomous_progress_terminal_ = false;
  double autonomous_progress_reference_heading_ = 0.0;
  bool autonomous_terminal_progress_enabled_ = false;
  double autonomous_terminal_xy_tolerance_ = 0.03;
  double autonomous_terminal_distance_ = 0.15;
  double localization_recovery_timeout_ { 10.0 };
  double localization_stable_duration_ { 0.5 };
  int max_recovery_rounds_ { 2 };
  bool check_smoother_collisions_ { true };
  double current_speed_ { 0.0 };
  double fixed_path_step_ { 0.1 };
  double fixed_path_max_speed_ { 1.5 };
  double fixed_path_boundary_half_width_ { 0.4 };
  double velocity_log_frequency_ { 1.0 };

  rclcpp::Time last_valid_tf_time_ { 0, 0, RCL_ROS_TIME };
  rclcpp::Time localization_lost_time_ { 0, 0, RCL_ROS_TIME };
  rclcpp::Time localization_stable_since_ { 0, 0, RCL_ROS_TIME };
  rclcpp::Time recovery_ready_time_ { 0, 0, RCL_ROS_TIME };
  geometry_msgs::msg::PoseStamped last_pose_;
  // 平滑失败回退使用的原始规划路径；对应回调必须先通过当前规划代次检查。
  nav_msgs::msg::Path pending_raw_path_;
  geometry_msgs::msg::Twist latest_controller_velocity_;
  geometry_msgs::msg::Twist latest_smoothed_velocity_;
  nav_msgs::msg::Odometry latest_velocity_odometry_;
  // 仅保护诊断速度缓存，不是整个任务状态机的全局互斥锁。
  std::mutex velocity_mutex_;
  bool has_controller_velocity_ { false };
  bool has_smoothed_velocity_ { false };
  bool has_velocity_odometry_ { false };

  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_pose_client_;
  rclcpp_action::Client<ComputePathThroughPoses>::SharedPtr compute_poses_client_;
  rclcpp_action::Client<SmoothPath>::SharedPtr smooth_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_pose_server_;
  rclcpp_action::Server<NavigateThroughPoses>::SharedPtr navigate_poses_server_;
  rclcpp_action::Server<NavigationService>::SharedPtr navigation_service_server_;
  rclcpp::Client<ClearCostmap>::SharedPtr clear_local_client_;
  rclcpp::Client<ClearCostmap>::SharedPtr clear_global_client_;

  // 这些句柄只在接受响应到达后可用；尚在等待接受的请求与已经持有句柄的任务需区分。
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
  rclcpp::SubscriptionBase::SharedPtr velocity_odom_sub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr fixed_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr fixed_path_boundaries_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::TimerBase::SharedPtr velocity_log_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<TfDiagnostics> tf_diagnostics_;
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_
