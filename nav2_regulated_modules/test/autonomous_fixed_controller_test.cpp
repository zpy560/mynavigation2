// 真实插件＋空白 Costmap＋合成反馈；仅调用求速度接口，不发布运动命令。
#include "nav2_regulated_modules/autonomous_fixed_controller.hpp"
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include <iostream>
#include <thread>
#include <limits>

namespace
{
void require(const bool value, const char * message)
{
  if (!value)
  {
    throw std::runtime_error(message);
  }
}

geometry_msgs::msg::PoseStamped point(const double x, const double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}
}

int main(int argc, char ** argv)
{
  const bool motion_feedback = argc > 1 && std::string(argv[1]) == "--motion-state";
  const std::string feedback_topic = motion_feedback ? "/fixed_controller_test/motion_state" : "/fixed_controller_test/odom";
  rclcpp::init(argc, argv);
  auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("fixed_controller_test");
  parent->declare_parameter("controller_frequency", 100.0);
  parent->declare_parameter("odom_topic", feedback_topic);
  parent->declare_parameter("tracking.tracker.desired_linear_vel", 0.5);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fixed_test_costmap");
  const std::vector<rclcpp::Parameter> map_parameters =
  {
    rclcpp::Parameter("plugins", std::vector<std::string>()), rclcpp::Parameter("global_frame", "map"), rclcpp::Parameter("robot_base_frame", "base_link"), rclcpp::Parameter("width", 5), rclcpp::Parameter("height", 5), rclcpp::Parameter("resolution", 0.05), rclcpp::Parameter("origin_x", -2.5), rclcpp::Parameter("origin_y", -2.5), rclcpp::Parameter("track_unknown_space", false), rclcpp::Parameter("footprint", "[[0.2, 0.1], [0.2, -0.1], [-0.2, -0.1], [-0.2, 0.1]]")
  };
  map->set_parameters(map_parameters);
  require(map->on_configure(rclcpp_lifecycle::State()) == nav2_util::CallbackReturn::SUCCESS, "empty costmap configures");
  auto tf = map->getTfBuffer();
  nav2_regulated_modules::AutonomousFixedPathController controller;
  controller.configure(parent, "tracking", tf, map);
  controller.activate();
  nav2_controller::StoppedGoalChecker checker;
  parent->declare_parameter("arrival.xy_goal_tolerance", 0.03);
  parent->declare_parameter("arrival.yaw_goal_tolerance", 0.08);
  parent->declare_parameter("arrival.trans_stopped_velocity", 0.01);
  parent->declare_parameter("arrival.rot_stopped_velocity", 0.05);
  checker.initialize(parent, "arrival", map);
  nav2_controller::StoppedGoalChecker cusp_checker;
  parent->declare_parameter("cusp.xy_goal_tolerance", 0.01);
  parent->declare_parameter("cusp.yaw_goal_tolerance", 0.08726646259971647);
  parent->declare_parameter("cusp.trans_stopped_velocity", 0.01);
  parent->declare_parameter("cusp.rot_stopped_velocity", 0.05);
  cusp_checker.initialize(parent, "cusp", map);
  nav2_core::GoalChecker * active_checker = &checker;
  auto odom_pub = motion_feedback ? nullptr : parent->create_publisher<nav_msgs::msg::Odometry>(feedback_topic, 10);
  auto motion_pub = motion_feedback ? parent->create_publisher<byd_custom_msgs::msg::MotionState>(feedback_topic, 10) : nullptr;
  if (motion_pub)
  {
    motion_pub->on_activate();
  }
  else
  {
    odom_pub->on_activate();
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parent->get_node_base_interface());
  const auto compute = [&](const geometry_msgs::msg::PoseStamped & input, const bool feedback = true, const double linear = 0.0, const double angular = 0.0, const double source_age = 0.0, const double tf_age = 0.0)
  {
    geometry_msgs::msg::Twist measured;
    measured.linear.x = linear;
    measured.angular.z = angular;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.header.stamp = parent->now() - rclcpp::Duration::from_seconds(tf_age);
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = input.pose.position.x;
    transform.transform.rotation = input.pose.orientation;
    // 延迟输入不能被上一帧更新鲜的测试缓存遮蔽。
    tf->clear();
    tf->setTransform(transform, "isolated_test", false);
    if (feedback)
    {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = parent->now() - rclcpp::Duration::from_seconds(source_age);
      odom.twist.twist = measured;
      if (motion_pub)
      {
        byd_custom_msgs::msg::MotionState motion;
        motion.header = odom.header;
        motion.v_car = linear;
        motion.w_car = angular;
        motion_pub->publish(motion);
      }
      else
      {
        odom_pub->publish(odom);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(12));
    executor.spin_some();
    auto pose = input;
    pose.header.stamp = transform.header.stamp;
    return controller.computeVelocityCommands(pose, measured, active_checker);
  };
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = parent->now();
  path.poses =
  {
    point(0), point(0.2)
  };
  controller.setPlan(path);
  geometry_msgs::msg::TwistStamped command;
  for (int i = 0; i < 15; ++i)
  {
    command = compute(point(0));
  }
  require(command.twist.linear.x > 0.0, "forward path produces positive velocity");
  command = compute(point(0), true, 0.0, 0.0, 1.0);
  require(command.twist.linear.x == 0.0 && command.twist.angular.z == 0.0, "newly received feedback retains its stale source timestamp");
  for (int i = 0; i < 20; ++i)
  {
    command = compute(point(0));
  }
  require(command.twist.linear.x > 0.0, "fresh feedback resumes the same path");
  command = compute(point(0), true, std::numeric_limits<double>::quiet_NaN());
  require(command.twist.linear.x == 0.0 && command.twist.angular.z == 0.0, "invalid linear feedback commands zero");
  command = compute(point(0), true, 0.0, std::numeric_limits<double>::infinity());
  require(command.twist.linear.x == 0.0 && command.twist.angular.z == 0.0, "invalid angular feedback commands zero");
  path.poses.back() = point(-0.2);
  controller.setPlan(path);
  for (int i = 0; i < 15; ++i)
  {
    command = compute(point(0));
  }
  require(command.twist.linear.x < 0.0, "reverse path produces negative velocity without 180 degree rotation");
  command = compute(point(0), true, -0.01, -0.01);
  require(command.twist.linear.x < 0.0, "signed reverse and angular feedback remains valid");
  path.poses.back() = point(0.2);
  controller.setPlan(path);
  for (int i = 0; i < 65; ++i)
  {
    compute(point(0.2));
  }
  require(controller.isTerminalStopLatched() && controller.isTerminalPositionAccurate(), "strict continuous stopped arrival completes");
  bool rejected = false;
  try
  {
    compute(point(0.26));
  }
  catch (const std::exception &)
  {
    rejected = true;
  }
  require(rejected && !controller.isTerminalStopLatched(), "post-stop drift cannot become fallback success");
  active_checker = &cusp_checker;
  controller.setPlan(path);
  command = compute(point(0.1925), true, 0.01015);
  require(!controller.isTerminalStopLatched() && command.twist.linear.x == 0.0, "recorded cusp locking speed requests braking only");
  command = compute(point(0.188), true, 0.0101);
  require(!controller.isTerminalStopLatched() && command.twist.linear.x == 0.0, "12mm transient before stopped feedback does not throw");
  for (int i = 0; i < 75; ++i)
  {
    compute(point(0.194));
  }
  require(controller.isTerminalStopLatched(), "real wrapper recovers to unchanged 1cm cusp acceptance");
  controller.setPlan(path);
  compute(point(0.1925));
  const auto waiting_started = std::chrono::steady_clock::now();
  rejected = false;
  bool trimming = false;
  for (int i = 0; i < 600 && !rejected; ++i)
  {
    try
    {
      command = compute(point(0.185));
      trimming = trimming || command.twist.linear.x > 0.0;
      require(command.twist.linear.x >= 0.0 && command.twist.linear.x <= 0.02 && !controller.isTerminalStopLatched(), "persistent cusp error gets bounded original-direction trim without early success");
    }
    catch (const nav2_core::PlannerException &)
    {
      rejected = true;
    }
  }
  require(trimming && rejected && std::chrono::duration<double>(std::chrono::steady_clock::now() - waiting_started).count() >= 5.0, "unresponsive position fails after the finite trim deadline");
  path.poses.back() = point(-0.2);
  controller.setPlan(path);
  compute(point(-0.1958));
  bool reverse_trim = false;
  controller.setSpeedLimit(0.003, false);
  for (int i = 0; i < 65; ++i)
  {
    command = compute(point(-0.186096));
    reverse_trim = reverse_trim || command.twist.linear.x < 0.0;
    require(command.twist.linear.x <= 0.0 && command.twist.linear.x >= -0.003 && !controller.isTerminalStopLatched(), "recorded 13.904mm error can trim backward while respecting speed limits");
  }
  require(reverse_trim, "persistent reverse cusp offset must generate active correction");
  for (int i = 0; i < 80; ++i)
  {
    compute(point(-0.196));
  }
  require(controller.isTerminalStopLatched(), "trimmed position still needs braking and full strict acceptance");
  controller.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  path.poses =
  {
    point(0), point(0.5), point(1.0), point(1.5), point(2.0)
  };
  controller.setPlan(path);
  command = compute(point(0), true, 0.0, 0.0, 0.0, 0.4);
  require(command.twist.linear.x == 0.0 && command.twist.angular.z == 0.0, "delayed TF cannot start a new execution");
  for (int i = 0; i < 25; ++i)
  {
    command = compute(point(0));
  }
  require(command.twist.linear.x > 0.0, "fresh source frames recover the existing execution");
  for (int i = 0; i < 45; ++i)
  {
    command = compute(point(0), true, 0.1, 0.0, 0.0, 0.45);
    require(command.twist.linear.x > 0.0 && !controller.isTerminalStopLatched(), "ongoing tracking tolerates the recorded bounded TF delay");
  }
  command = compute(point(0), true, 0.0, 0.0, 1.0, 0.45);
  require(command.twist.linear.x == 0.0 && command.twist.angular.z == 0.0, "tracking TF tolerance never bypasses stale feedback");
  controller.setPlan(path);
  compute(point(0));
  command = compute(point(1.7), true, 0.0, 0.0, 0.0, 0.45);
  require(command.twist.linear.x == 0.0 && command.twist.angular.z == 0.0 && !controller.isTerminalStopLatched(), "terminal zone rejects delayed TF before acceptance");
  controller.setPlan(path);
  compute(point(0));
  command = compute(point(0), true, 0.0, 0.0, 0.0, 0.7);
  require(command.twist.linear.x == 0.0 && command.twist.angular.z == 0.0, "TF beyond the tracking limit still stops");
  controller.setPlan(path);
  compute(point(0));
  command = compute(point(0), true, 0.0, 0.0, 0.0, -0.2);
  require(command.twist.linear.x == 0.0 && command.twist.angular.z == 0.0, "future TF is never admitted by tracking tolerance");
  path.poses =
  {
    point(0), point(0.2)
  };
  active_checker = &checker;
  path.poses.back() = point(0.2, 1.0);
  controller.setPlan(path);
  for (int i = 0; i < 30; ++i)
  {
    command = compute(point(0.2));
  }
  require(command.twist.linear.x == 0.0 && command.twist.angular.z > 0.0 && !controller.isTerminalStopLatched(), "goal heading aligns after braking without early success");
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  command = compute(point(0.2), false);
  require(command.twist.linear.x == 0.0 && command.twist.angular.z == 0.0 && !controller.isTerminalStopLatched(), "stale feedback commands zero and invalidates arrival");
  path.poses.back() = point(0.2);
  controller.setPlan(path);
  unsigned int mx = 0;
  unsigned int my = 0;
  require(map->getCostmap()->worldToMap(0.2, 0.0, mx, my), "obstacle cell is inside costmap");
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  command = compute(point(0));
  require(command.twist.linear.x == 0.0 && command.twist.angular.z == 0.0, "footprint collision suppresses motion");
  controller.deactivate();
  controller.cleanup();
  executor.remove_node(parent->get_node_base_interface());
  map->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
  std::cout << (motion_feedback ? "MotionState" : "Odometry") << " controller forward/reverse/strict arrival/yaw/stale feedback/footprint checks passed; no motion command published\n";
}
