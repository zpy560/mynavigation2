// Copyright (c) 2022 Samsung Research
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

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_velocity_smoother/velocity_smoother.hpp"

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;
using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace nav2_velocity_smoother
{

VelocitySmoother::VelocitySmoother(const rclcpp::NodeOptions & options) : LifecycleNode("velocity_smoother", "", options), last_command_time_{0, 0, get_clock()->get_clock_type()} {
  // SpdlogWrapper::init("nav2_velocity_smoother", get_name());
  LOG_INFO("Creating velocity smoother");
  LOG_INFO("Velocity smoother limits cmd_vel_nav before publishing final cmd_vel");

}

VelocitySmoother::~VelocitySmoother() {
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
}

nav2_util::CallbackReturn VelocitySmoother::on_configure(const rclcpp_lifecycle::State &) {
  LOG_INFO("Configuring velocity smoother");
  LOG_INFO("Configuring velocity limits, feedback mode and command topics");
  auto node = shared_from_this();
  std::string feedback_type;
  double velocity_timeout_dbl;

  // Smoothing metadata
  declare_parameter_if_not_declared(node, "smoothing_frequency", rclcpp::ParameterValue(20.0));
  declare_parameter_if_not_declared(node, "feedback", rclcpp::ParameterValue(std::string("OPEN_LOOP")));
  declare_parameter_if_not_declared(node, "scale_velocities", rclcpp::ParameterValue(false));
  node->get_parameter("smoothing_frequency", smoothing_frequency_);
  node->get_parameter("feedback", feedback_type);
  node->get_parameter("scale_velocities", scale_velocities_);
  LOG_INFO("Velocity smoother metadata smoothing_frequency={}, feedback={}, scale_velocities={}", smoothing_frequency_, feedback_type.c_str(), scale_velocities_);

  // Kinematics
  declare_parameter_if_not_declared(node, "max_velocity", rclcpp::ParameterValue(std::vector<double>{0.50, 0.0, 2.5}));
  declare_parameter_if_not_declared(node, "min_velocity", rclcpp::ParameterValue(std::vector<double>{-0.50, 0.0, -2.5}));
  declare_parameter_if_not_declared(node, "max_accel", rclcpp::ParameterValue(std::vector<double>{2.5, 0.0, 3.2}));
  declare_parameter_if_not_declared(node, "max_decel", rclcpp::ParameterValue(std::vector<double>{-2.5, 0.0, -3.2}));
  node->get_parameter("max_velocity", max_velocities_);
  node->get_parameter("min_velocity", min_velocities_);
  node->get_parameter("max_accel", max_accels_);
  node->get_parameter("max_decel", max_decels_);

  for (unsigned int i = 0; i != 3; i++) {
    if (max_decels_[i] > 0.0) {
      throw std::runtime_error("Positive values set of deceleration! These should be negative to slow down!");
    }
    if (max_accels_[i] < 0.0) {
      throw std::runtime_error("Negative values set of acceleration! These should be positive to speed up!");
    }
    if (min_velocities_[i] > max_velocities_[i]) {
      throw std::runtime_error("Min velocities are higher than max velocities!");
    }
  }

  // Get feature parameters
  declare_parameter_if_not_declared(node, "odom_topic", rclcpp::ParameterValue("odom"));
  declare_parameter_if_not_declared(node, "odom_duration", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, "deadband_velocity", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
  declare_parameter_if_not_declared(node, "velocity_timeout", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, "speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
  node->get_parameter("odom_topic", odom_topic_);
  node->get_parameter("odom_duration", odom_duration_);
  node->get_parameter("deadband_velocity", deadband_velocities_);
  node->get_parameter("velocity_timeout", velocity_timeout_dbl);
  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  velocity_timeout_ = rclcpp::Duration::from_seconds(velocity_timeout_dbl);
  LOG_INFO("Velocity smoother feature parameters odom_topic={}, odom_duration={}, velocity_timeout={}", odom_topic_.c_str(), odom_duration_, velocity_timeout_dbl);

  if (max_velocities_.size() != 3 || min_velocities_.size() != 3 || max_accels_.size() != 3 || max_decels_.size() != 3 || deadband_velocities_.size() != 3)
  {
    throw std::runtime_error("Invalid setting of kinematic and/or deadband limits!" " All limits must be size of 3 representing (x, y, theta).");
  }
  target_maxvx_ = max_velocities_[0];
  target_minvx_ = min_velocities_[0];

  // Get control type
  if (feedback_type == "OPEN_LOOP") {
    open_loop_ = true;
    LOG_INFO("Velocity smoother uses OPEN_LOOP feedback from last published command");
  } else if (feedback_type == "CLOSED_LOOP") {
    open_loop_ = false;
    odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(node, odom_duration_, odom_topic_);
    LOG_INFO("Velocity smoother uses CLOSED_LOOP feedback from odometry topic {}", odom_topic_.c_str());
  } else {
    throw std::runtime_error("Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.");
  }

  // Setup inputs / outputs
  smoothed_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 1);
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1), std::bind(&VelocitySmoother::inputCommandCallback, this, std::placeholders::_1));
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(speed_limit_topic, rclcpp::QoS(10), std::bind(&VelocitySmoother::speedLimitCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VelocitySmoother::on_activate(const rclcpp_lifecycle::State &) {
  LOG_INFO("Activating");
  LOG_INFO("Activating smoothed cmd_vel publisher and smoothing timer");
  smoothed_cmd_pub_->on_activate();
  double timer_duration_ms = 1000.0 / smoothing_frequency_;
  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(timer_duration_ms)), std::bind(&VelocitySmoother::smootherTimer, this));

  dyn_params_handler_ = this->add_on_set_parameters_callback(std::bind(&VelocitySmoother::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VelocitySmoother::on_deactivate(const rclcpp_lifecycle::State &) {
  LOG_INFO("Deactivating");
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  smoothed_cmd_pub_->on_deactivate();
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VelocitySmoother::on_cleanup(const rclcpp_lifecycle::State &) {
  LOG_INFO("Cleaning up");
  smoothed_cmd_pub_.reset();
  odom_smoother_.reset();
  cmd_sub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VelocitySmoother::on_shutdown(const rclcpp_lifecycle::State &) {
  LOG_INFO("Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void VelocitySmoother::inputCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(*msg)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  command_ = msg;
  last_command_time_ = now();
  LOG_INFO("Received raw cmd_vel linear=({:.3f}, {:.3f}) angular_z={:.3f}", msg->linear.x, msg->linear.y, msg->angular.z);
}

double VelocitySmoother::findEtaConstraint(const double v_curr, const double v_cmd, const double accel, const double decel) {
  // Exploiting vector scaling properties
  double dv = v_cmd - v_curr;

  double v_component_max;
  double v_component_min;

  // Accelerating if magnitude of v_cmd is above magnitude of v_curr
  // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
  // Decelerating otherwise
  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    v_component_max = accel / smoothing_frequency_;
    v_component_min = -accel / smoothing_frequency_;
  } else {
    v_component_max = -decel / smoothing_frequency_;
    v_component_min = decel / smoothing_frequency_;
  }

  if (dv > v_component_max) {
    return v_component_max / dv;
  }

  if (dv < v_component_min) {
    return v_component_min / dv;
  }

  return -1.0;
}

double VelocitySmoother::applyConstraints(const double v_curr, const double v_cmd, const double accel, const double decel, const double eta) {
  double dv = v_cmd - v_curr;

  double v_component_max;
  double v_component_min;

  // Accelerating if magnitude of v_cmd is above magnitude of v_curr
  // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
  // Decelerating otherwise
  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    v_component_max = accel / smoothing_frequency_;
    v_component_min = -accel / smoothing_frequency_;
  } else {
    v_component_max = -decel / smoothing_frequency_;
    v_component_min = decel / smoothing_frequency_;
  }

  return v_curr + std::clamp(eta * dv, v_component_min, v_component_max);
}

void VelocitySmoother::smootherTimer() {
  // Wait until the first command is received
  if (!command_) {
    return;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  // Check for velocity timeout. If nothing received, publish zeros to apply deceleration
  if (now() - last_command_time_ > velocity_timeout_) {
    if (last_cmd_ == geometry_msgs::msg::Twist() || stopped_) {
      stopped_ = true;
      return;
    }
    *command_ = geometry_msgs::msg::Twist();
  }

  stopped_ = false;

  // Get current velocity based on feedback type
  geometry_msgs::msg::Twist current_;
  if (open_loop_) {
    current_ = last_cmd_;
  } else {
    current_ = odom_smoother_->getTwist();
  }
  if (limitv2target) {
    // 当前速度不在 [target_minvx_, target_maxvx_] 区间内：以加速度/减速度步长逐步逼近目标限幅，避免超调
    if (current_.linear.x < target_minvx_ || current_.linear.x > target_maxvx_) {
      max_velocities_[0] = applyConstraints(max_velocities_[0], target_maxvx_, max_accels_[0], max_decels_[0], 1.0);
      min_velocities_[0] = applyConstraints(min_velocities_[0], target_minvx_, max_accels_[0], max_decels_[0], 1.0);
    } else {
      // 当前速度已在目标区间内：直接到位
      max_velocities_[0] = target_maxvx_;
      min_velocities_[0] = target_minvx_;
    }

    // 限幅已移动到目标则结束过渡
    if (std::fabs(max_velocities_[0] - target_maxvx_) < 1e-3 &&
      std::fabs(min_velocities_[0] - target_minvx_) < 1e-3)
    {
      max_velocities_[0] = target_maxvx_;
      min_velocities_[0] = target_minvx_;
      limitv2target = false;
    }
  }

  // Apply absolute velocity restrictions to the command
  command_->linear.x = std::clamp(command_->linear.x, min_velocities_[0], max_velocities_[0]);
  command_->linear.y = std::clamp(command_->linear.y, min_velocities_[1], max_velocities_[1]);
  command_->angular.z = std::clamp(command_->angular.z, min_velocities_[2], max_velocities_[2]);

  // Find if any component is not within the acceleration constraints. If so, store the most
  // significant scale factor to apply to the vector <dvx, dvy, dvw>, eta, to reduce all axes
  // proportionally to follow the same direction, within change of velocity bounds.
  // In case eta reduces another axis out of its own limit, apply accel constraint to guarantee
  // output is within limits, even if it deviates from requested command slightly.
  double eta = 1.0;
  if (scale_velocities_) {
    double curr_eta = -1.0;

    curr_eta = findEtaConstraint(current_.linear.x, command_->linear.x, max_accels_[0], max_decels_[0]);
    if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
      eta = curr_eta;
    }

    curr_eta = findEtaConstraint(current_.linear.y, command_->linear.y, max_accels_[1], max_decels_[1]);
    if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
      eta = curr_eta;
    }

    curr_eta = findEtaConstraint(current_.angular.z, command_->angular.z, max_accels_[2], max_decels_[2]);
    if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
      eta = curr_eta;
    }
  }

  cmd_vel->linear.x = applyConstraints(current_.linear.x, command_->linear.x, max_accels_[0], max_decels_[0], eta);
  cmd_vel->linear.y = applyConstraints(current_.linear.y, command_->linear.y, max_accels_[1], max_decels_[1], eta);
  cmd_vel->angular.z = applyConstraints(current_.angular.z, command_->angular.z, max_accels_[2], max_decels_[2], eta);
  last_cmd_ = *cmd_vel;

  cmd_vel->linear.x = std::clamp(cmd_vel->linear.x, min_velocities_[0], max_velocities_[0]);
  cmd_vel->linear.y = std::clamp(cmd_vel->linear.y, min_velocities_[1], max_velocities_[1]);
  cmd_vel->angular.z = std::clamp(cmd_vel->angular.z, min_velocities_[2], max_velocities_[2]);
  // Apply deadband restrictions & publish
  cmd_vel->linear.x = fabs(cmd_vel->linear.x) < deadband_velocities_[0] ? 0.0 : cmd_vel->linear.x;
  cmd_vel->linear.y = fabs(cmd_vel->linear.y) < deadband_velocities_[1] ? 0.0 : cmd_vel->linear.y;
  cmd_vel->angular.z = fabs(cmd_vel->angular.z) < deadband_velocities_[2] ? 0.0 : cmd_vel->angular.z;
  // LOG_INFO("command: linear.x:{}, linear.y:{}, angular.z:{}| current:{}, {}, {} | cmd_vel:{}, {}, {}", 
  //   command_->linear.x, command_->linear.y, command_->angular.z,
  //   current_.linear.x,  current_.linear.y,  current_.angular.z,
  //   cmd_vel->linear.x,  cmd_vel->linear.y,  cmd_vel->angular.z
  // );
  // LOG_INFO(" min_velocities_[0]={}, max_velocities_[0]={} ", min_velocities_[0], max_velocities_[0] );

  smoothed_cmd_pub_->publish(std::move(cmd_vel));
}

void VelocitySmoother::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
  if (msg->percentage) {
      RCLCPP_ERROR(this->get_logger(), "Percentage mode is not supported");
      return;
  }

  double speed = msg->speed_limit;
  RCLCPP_INFO(this->get_logger(), "set speed = {%0.3f}",speed);
  target_maxvx_ = std::max(0.0, speed);   // 正数保留，负数归零
  target_minvx_ = std::min(0.0, speed);   // 负数保留，正数归零
  limitv2target = true;
}

rcl_interfaces::msg::SetParametersResult VelocitySmoother::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "smoothing_frequency") {
        smoothing_frequency_ = parameter.as_double();
        LOG_INFO("Updating smoothing_frequency to {}", smoothing_frequency_);
        if (timer_) {
          timer_->cancel();
          timer_.reset();
        }

        double timer_duration_ms = 1000.0 / smoothing_frequency_;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(timer_duration_ms)), std::bind(&VelocitySmoother::smootherTimer, this));
      } else if (name == "velocity_timeout") {
        velocity_timeout_ = rclcpp::Duration::from_seconds(parameter.as_double());
        LOG_INFO("Updating velocity_timeout to {}", parameter.as_double());
      } else if (name == "odom_duration") {
        odom_duration_ = parameter.as_double();
        LOG_INFO("Updating odom_duration to {}", odom_duration_);
        odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(shared_from_this(), odom_duration_, odom_topic_);
      }
    } else if (type == ParameterType::PARAMETER_DOUBLE_ARRAY) {
      if (parameter.as_double_array().size() != 3) {
        RCLCPP_WARN(get_logger(), "Invalid size of parameter %s. Must be size 3", name.c_str());
        result.successful = false;
        break;
      }

      if (name == "max_velocity") {
        max_velocities_ = parameter.as_double_array();
        LOG_INFO("Updating max_velocity limits");
      } else if (name == "min_velocity") {
        min_velocities_ = parameter.as_double_array();
        LOG_INFO("Updating min_velocity limits");
      } else if (name == "max_accel") {
        for (unsigned int i = 0; i != 3; i++) {
          if (parameter.as_double_array()[i] < 0.0) {
            RCLCPP_WARN(get_logger(), "Negative values set of acceleration! These should be positive to speed up!");
            result.successful = false;
          }
        }
        max_accels_ = parameter.as_double_array();
        LOG_INFO("Updating max_accel limits");
      } else if (name == "max_decel") {
        for (unsigned int i = 0; i != 3; i++) {
          if (parameter.as_double_array()[i] > 0.0) {
            RCLCPP_WARN(get_logger(), "Positive values set of deceleration! These should be negative to slow down!");
            result.successful = false;
          }
        }
        max_decels_ = parameter.as_double_array();
        LOG_INFO("Updating max_decel limits");
      } else if (name == "deadband_velocity") {
        deadband_velocities_ = parameter.as_double_array();
        LOG_INFO("Updating deadband_velocity limits");
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "feedback") {
        if (parameter.as_string() == "OPEN_LOOP") {
          open_loop_ = true;
          odom_smoother_.reset();
          LOG_INFO("Switching velocity smoother feedback to OPEN_LOOP");
        } else if (parameter.as_string() == "CLOSED_LOOP") {
          open_loop_ = false;
          odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(shared_from_this(), odom_duration_, odom_topic_);
          LOG_INFO("Switching velocity smoother feedback to CLOSED_LOOP");
        } else {
          RCLCPP_WARN(get_logger(), "Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.");
          result.successful = false;
          break;
        }
      } else if (name == "odom_topic") {
        odom_topic_ = parameter.as_string();
        LOG_INFO("Updating odom_topic to {}", odom_topic_.c_str());
        odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(shared_from_this(), odom_duration_, odom_topic_);
      }
    }
  }

  return result;
}

}  // namespace nav2_velocity_smoother

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_velocity_smoother::VelocitySmoother)
