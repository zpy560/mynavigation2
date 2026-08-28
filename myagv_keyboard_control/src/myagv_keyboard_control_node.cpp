#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "byd_custom_msgs/msg/control_res.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class TerminalMode
{
public:
  explicit TerminalMode(const std::string & input_device) {
    input_fd_ = open(input_device.c_str(), O_RDONLY | O_NOCTTY);
    if (input_fd_ == -1 && isatty(STDIN_FILENO)) {
      input_fd_ = dup(STDIN_FILENO);
    }
    if (input_fd_ == -1) {
      throw std::runtime_error("failed to open input device '" + input_device + "' and stdin is not a terminal");
    }

    if (tcgetattr(input_fd_, &original_termios_) == -1) {
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to read terminal attributes");
    }

    original_flags_ = fcntl(input_fd_, F_GETFL, 0);
    if (original_flags_ == -1) {
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to read terminal flags");
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(input_fd_, TCSANOW, &raw) == -1) {
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to configure terminal attributes");
    }
    termios_changed_ = true;

    if (fcntl(input_fd_, F_SETFL, original_flags_ | O_NONBLOCK) == -1) {
      restore();
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to configure non-blocking terminal input");
    }
    flags_changed_ = true;
  }

  ~TerminalMode() {
    restore();
    if (input_fd_ != -1) {
      close(input_fd_);
      input_fd_ = -1;
    }
  }

  TerminalMode(const TerminalMode &) = delete;
  TerminalMode & operator=(const TerminalMode &) = delete;

  int inputFd() const {
    return input_fd_;
  }

private:
  void restore() {
    if (flags_changed_) {
      (void)fcntl(input_fd_, F_SETFL, original_flags_);
      flags_changed_ = false;
    }
    if (termios_changed_) {
      (void)tcsetattr(input_fd_, TCSANOW, &original_termios_);
      termios_changed_ = false;
    }
  }

  int input_fd_{-1};
  termios original_termios_{};
  int original_flags_{0};
  bool termios_changed_{false};
  bool flags_changed_{false};
};

class SCurvePlanner {
public:
    SCurvePlanner(double dt, double sm, 
                  double norm_accel_amax, double norm_accel_jmax, 
                  double norm_decel_amax, double norm_decel_jmax)
                //   double emrg_amax, double emrg_jmax)
        : dt_(dt), sm_(sm), 
          am_(norm_accel_amax), jm_(norm_accel_jmax),
          norm_accel_amax_(norm_accel_amax), norm_accel_jmax_(norm_accel_jmax), 
          norm_decel_amax_(norm_decel_amax), norm_decel_jmax_(norm_decel_jmax), 
          emrg_amax_(8.0), emrg_jmax_(8.0),
          sc_(0.0), ac_(0.0), jc_(0.0), d_(0.0) {}

    void reset(){
        sc_ = 0.0;
        ac_ = 0.0;
        jc_ = 0.0;
        d_  = 0.0;
    }

    void setDirection(double d, bool emrgstate = false) {
        d_ = std::max(-1.0, std::min(1.0, d));
        if (sc_ * d_ < 0.0 && std::abs(sc_) > 1e-6) {
            d_ = 0.0; 
        }
        if(prestate_emrg == true && emrgstate == false){
            am_ = norm_accel_amax_;   
            jm_ = norm_accel_jmax_;   
            prestate_emrg = false;
        }
        if(prestate_emrg == false && emrgstate == true){
            am_ = emrg_amax_;  
            jm_ = emrg_jmax_;   
            prestate_emrg = true;
        }
    }

    double update() {
        double vg = d_ * sm_;

        if (std::abs(sc_ - vg) <= 1e-6 ) {
            sc_ = vg;
            ac_ = 0.0;
            jc_ = 0.0;
            return sc_;
        }
        
        if (std::abs(ac_) < 1e-6) {
            bool is_speeding_up = (sc_ * vg >= 0.0) && (std::abs(sc_) < std::abs(vg));
            
            if (is_speeding_up) {
                am_ = norm_accel_amax_;
                jm_ = norm_accel_jmax_;
            } else {
                am_ = norm_decel_amax_;
                jm_ = norm_decel_jmax_;
            }
        } else {
            if (sc_ * ac_ >= 0.0) {
                am_ = norm_accel_amax_;
                jm_ = norm_accel_jmax_;
            } else {
                am_ = norm_decel_amax_;
                jm_ = norm_decel_jmax_;
            }
        }

        double vel_err = vg - sc_;
        double dr = 0;
        if( vel_err < -1e-6 ) dr = -1.0;
        if( vel_err >  1e-6 ) dr =  1.0;

        
        if ( dr*ac_>=0.0 && std::abs(vel_err)-0.5*std::abs(ac_*ac_/jm_) <= 1e-4) {
            if( std::abs(dr) > 1e-6){
              jc_ =  -dr*jm_;
            } else {
              if (ac_ > 0.0) jc_ = -jm_;  
              if (ac_ < 0.0) jc_ =  jm_;  
            }
        } else {
            if( std::abs(dr) > 1e-6){
              jc_ =  dr*jm_;
            }
            else{
              if (ac_ > 0.0) jc_ =  jm_;
              if (ac_ < 0.0) jc_ = -jm_;
            }
            
        }

        double tmp = ac_;
        ac_ += jc_ * dt_;
        
        ac_ = std::max(-am_, std::min(am_, ac_));
        if( dr*ac_>=0.0 && dr*jc_<=0.0 )
        {
          if( jc_<0.0 ) {
            ac_ = std::max(ac_, 0.0);
          }
          if( jc_>0.0 ) {
            ac_ = std::min(ac_, 0.0);
          }
        }

        sc_ += 0.5*(tmp + ac_)*dt_ ;

        if (dr >= 1e-6) {
            sc_ = std::min(sc_, vg);
        } else if (dr <= -1e-6) {
            sc_ = std::max(sc_, vg);
        }
        else{
          sc_ = vg;
        }
        sc_ = std::max(-sm_, std::min(sm_, sc_));
        return sc_;
    }

    double getVelocity() const { return sc_; }
    double getAcceleration() const { return ac_; }
    double getJerk() const { return jc_; }

private:
    
    bool prestate_emrg = false;
    double dt_;
    double sm_;
    double am_;
    double jm_;

    double norm_accel_amax_;
    double norm_accel_jmax_;
    double norm_decel_amax_;
    double norm_decel_jmax_;
    double emrg_amax_;
    double emrg_jmax_;

    double sc_;
    double ac_;
    double jc_;
    double d_;
};

enum class Motion
{
  STOP,
  FORWARD,
  REVERSE,
  LEFT,
  RIGHT
};

constexpr double kVelocityEpsilon = 1e-9;

double approachVelocity(const double current, const double target, const double acceleration_limit, const double deceleration_limit, const double dt) {
  if (std::abs(target - current) <= kVelocityEpsilon) {
    return target;
  }

  const bool changing_direction = current * target < 0.0;
  const bool increasing_magnitude = std::abs(target) > std::abs(current);
  const double rate_limit = changing_direction || !increasing_magnitude ? deceleration_limit : acceleration_limit;
  const double max_delta = rate_limit * dt;
  if (target > current) {
    return std::min(current + max_delta, target);
  }
  return std::max(current - max_delta, target);
}

}  // namespace

class MyAgvKeyboardControl : public rclcpp::Node
{
public:
  MyAgvKeyboardControl() : Node("myagv_keyboard_control") {
    const auto input_device = declare_parameter<std::string>("input_device", "/dev/tty");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/control_to_uart");
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    linear_speed_ = declare_parameter<double>("linear_speed", 0.2);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.5);
    linear_accel_limit_ = declare_parameter<double>("linear_accel_limit", 0.4);
    linear_decel_limit_ = declare_parameter<double>("linear_decel_limit", 0.8);
    angular_accel_limit_ = declare_parameter<double>("angular_accel_limit", 1.0);
    angular_decel_limit_ = declare_parameter<double>("angular_decel_limit", 2.0);
    linear_accel_jerk_limit_ = declare_parameter<double>("linear_accel_jerk_limit", 0.4);
    linear_decel_jerk_limit_ = declare_parameter<double>("linear_decel_jerk_limit", 0.8);
    angular_accel_jerk_limit_ = declare_parameter<double>("angular_accel_jerk_limit", 1.0);
    angular_decel_jerk_limit_ = declare_parameter<double>("angular_decel_jerk_limit", 2.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);

    validateParameters();

    double dt = 1.0/publish_rate_;
    linear_planner_ = std::make_unique<SCurvePlanner>(dt, 
      linear_speed_, linear_accel_limit_, linear_accel_jerk_limit_, linear_decel_limit_, linear_decel_jerk_limit_);
    angular_planner_ = std::make_unique<SCurvePlanner>(dt, 
      angular_speed_, angular_accel_limit_, angular_accel_jerk_limit_, angular_decel_limit_, angular_decel_jerk_limit_);

    terminal_mode_ = std::make_unique<TerminalMode>(input_device);
    publisher_ = create_publisher<byd_custom_msgs::msg::ControlRes>(output_topic, 10);
    last_direction_key_ = std::chrono::steady_clock::now();
    last_update_time_ = last_direction_key_;

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = create_wall_timer(period, std::bind(&MyAgvKeyboardControl::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Publishing byd_custom_msgs/msg/ControlRes to %s at %.1f Hz", output_topic.c_str(), publish_rate_);
    RCLCPP_INFO(get_logger(), "Controls: W/Up forward, S/Down reverse, A/Left turn left, D/Right turn right, " "Space/X stop, Q quit");
    RCLCPP_INFO(get_logger(), "Speeds: linear %.3f m/s, angular %.3f rad/s, command timeout %.3f s", linear_speed_, angular_speed_, command_timeout_);
    RCLCPP_INFO(get_logger(), "Rate limits: linear accel/decel %.3f/%.3f m/s^2, angular accel/decel " "%.3f/%.3f rad/s^2", linear_accel_limit_, linear_decel_limit_, angular_accel_limit_, angular_decel_limit_);
  }

  void stop() {
    hardStop();
    publishCommand();
  }

private:
  void validateParameters() const {
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be greater than zero");
    }
    if (!std::isfinite(linear_speed_) || linear_speed_ < 0.0) {
      throw std::invalid_argument("linear_speed must be finite and non-negative");
    }
    if (!std::isfinite(angular_speed_) || angular_speed_ < 0.0) {
      throw std::invalid_argument("angular_speed must be finite and non-negative");
    }
    if (!std::isfinite(linear_accel_limit_) || linear_accel_limit_ <= 0.0) {
      throw std::invalid_argument("linear_accel_limit must be finite and greater than zero");
    }
    if (!std::isfinite(linear_decel_limit_) || linear_decel_limit_ <= 0.0) {
      throw std::invalid_argument("linear_decel_limit must be finite and greater than zero");
    }
    if (!std::isfinite(angular_accel_limit_) || angular_accel_limit_ <= 0.0) {
      throw std::invalid_argument("angular_accel_limit must be finite and greater than zero");
    }
    if (!std::isfinite(angular_decel_limit_) || angular_decel_limit_ <= 0.0) {
      throw std::invalid_argument("angular_decel_limit must be finite and greater than zero");
    }
    if (!std::isfinite(linear_accel_jerk_limit_) || linear_accel_jerk_limit_ <= 0.0) {
      throw std::invalid_argument("linear_accel_jerk_limit must be finite and greater than zero");
    }
    if (!std::isfinite(linear_decel_jerk_limit_) || linear_decel_jerk_limit_ <= 0.0) {
      throw std::invalid_argument("linear_decel_jerk_limit must be finite and greater than zero");
    }
    if (!std::isfinite(angular_accel_jerk_limit_) || angular_accel_jerk_limit_ <= 0.0) {
      throw std::invalid_argument("angular_accel_jerk_limit must be finite and greater than zero");
    }
    if (!std::isfinite(angular_decel_jerk_limit_) || angular_decel_jerk_limit_ <= 0.0) {
      throw std::invalid_argument("angular_decel_jerk_limit must be finite and greater than zero");
    }
    if (!std::isfinite(command_timeout_) || command_timeout_ < 0.0) {
      throw std::invalid_argument("command_timeout must be finite and non-negative");
    }
  }

  void timerCallback() {
    readKeyboard();
    const auto now = std::chrono::steady_clock::now();
    applyCommandTimeout(now);
    
    // RCLCPP_INFO(get_logger(), "线性速度规划.");
    // const double elapsed = std::chrono::duration<double>(now - last_update_time_).count();
    // last_update_time_ = now;
    // const double dt = std::clamp(elapsed, 0.0, 2.0 / publish_rate_);
    // RCLCPP_INFO(get_logger(), "msg print time: dt=%.3f .", dt);
    // updateSmoothedCommand(dt);             

    // RCLCPP_INFO(get_logger(), "S型曲线速度规划.");            
    current_v_ = linear_planner_->update();   
    current_w_ = angular_planner_->update();  

    publishCommand();
  }

  void readKeyboard() {
    char key = 0;
    while (true) {
      const ssize_t bytes_read = read(terminal_mode_->inputFd(), &key, 1);
      if (bytes_read == 1) {
        processKey(key);
        continue;
      }
      if (bytes_read == -1 && errno == EINTR) {
        continue;
      }
      if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read keyboard input: %s", strerror(errno));
      }
      break;
    }
  }

  void processKey(char key) {
    if (escape_state_ == 1) {
      escape_state_ = key == '[' ? 2 : 0;
      return;
    }

    if (escape_state_ == 2) {
      escape_state_ = 0;
      switch (key) {
        case 'A':
          commandMotion(Motion::FORWARD);
          return;
        case 'B':
          commandMotion(Motion::REVERSE);
          return;
        case 'C':
          commandMotion(Motion::RIGHT);
          return;
        case 'D':
          commandMotion(Motion::LEFT);
          return;
        default:
          return;
      }
    }

    if (key == '\x1b') {
      escape_state_ = 1;
      return;
    }

    const char normalized = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    switch (normalized) {
      case 'w':
        commandMotion(Motion::FORWARD);
        break;
      case 's':
        commandMotion(Motion::REVERSE);
        break;
      case 'a':
        commandMotion(Motion::LEFT);
        break;
      case 'd':
        commandMotion(Motion::RIGHT);
        break;
      case 'x':
      case ' ':
        setMotion(Motion::STOP, true);
        break;
      case 'q':
        hardStop();
        publishCommand();
        RCLCPP_INFO(get_logger(), "Quit requested; published stop command");
        rclcpp::shutdown();
        break;
      default:
        break;
    }
  }

  void commandMotion(Motion motion) {
    last_direction_key_ = std::chrono::steady_clock::now();
    setMotion(motion, true);
  }

  void setMotion(Motion motion, bool log_change) {
    if (motion_ == motion) {
      return;
    }

    motion_ = motion;
    switch (motion_) {
      case Motion::FORWARD:
        target_v_ = linear_speed_;
        target_w_ = 0.0;
        linear_planner_->setDirection(1.0);
        angular_planner_->setDirection(0.0);
        break;
      case Motion::REVERSE:
        target_v_ = -linear_speed_;
        target_w_ = 0.0;
        linear_planner_->setDirection(-1.0);
        angular_planner_->setDirection(0.0);
        break;
      case Motion::LEFT:
        target_v_ = 0.0;
        target_w_ = angular_speed_;
        linear_planner_->setDirection(0.0);
        angular_planner_->setDirection(1.0);
        break;
      case Motion::RIGHT:
        target_v_ = 0.0;
        target_w_ = -angular_speed_;
        linear_planner_->setDirection(0.0);
        angular_planner_->setDirection(-1.0);
        break;
      case Motion::STOP:
        target_v_ = 0.0;
        target_w_ = 0.0;
        linear_planner_->setDirection(0.0);
        angular_planner_->setDirection(0.0);
        break;
    }

    if (log_change) {
      RCLCPP_INFO(get_logger(), "Target changed: v=%.3f, w=%.3f", target_v_, target_w_);
    }
  }

  void hardStop() {
    motion_ = Motion::STOP;
    target_v_ = 0.0;
    target_w_ = 0.0;
    current_v_ = 0.0;
    current_w_ = 0.0;
  }

  bool transitionRequiresStop() const {
    const bool linear_sign_change = current_v_ * target_v_ < 0.0;
    const bool angular_sign_change = current_w_ * target_w_ < 0.0;
    const bool linear_to_angular = std::abs(current_v_) > kVelocityEpsilon && std::abs(target_w_) > kVelocityEpsilon;
    const bool angular_to_linear = std::abs(current_w_) > kVelocityEpsilon && std::abs(target_v_) > kVelocityEpsilon;
    return linear_sign_change || angular_sign_change || linear_to_angular || angular_to_linear;
  }

  void updateSmoothedCommand(const double dt) {
    double effective_target_v = target_v_;
    double effective_target_w = target_w_;
    if (transitionRequiresStop()) {
      effective_target_v = 0.0;
      effective_target_w = 0.0;
    }

    current_v_ = approachVelocity(current_v_, effective_target_v, linear_accel_limit_, linear_decel_limit_, dt);
    current_w_ = approachVelocity(current_w_, effective_target_w, angular_accel_limit_, angular_decel_limit_, dt);
  }

  void applyCommandTimeout(const std::chrono::steady_clock::time_point now) {
    if (motion_ == Motion::STOP || command_timeout_ == 0.0) {
      return;
    }

    const auto elapsed = std::chrono::duration<double>(now - last_direction_key_).count();
    if (elapsed > command_timeout_) {
      setMotion(Motion::STOP, false);
      RCLCPP_WARN(get_logger(), "Keyboard command timed out; smoothly decelerating chassis");
    }
  }

  void publishCommand() {
    byd_custom_msgs::msg::ControlRes msg;
    msg.v = current_v_;
    msg.w = current_w_;
    msg.v_lift = 0.0;
    msg.w_rotation = 0.0;
    publisher_->publish(msg);
    // RCLCPP_INFO(get_logger(), "msg.v: v=%.3f, msg.w: w=%.3f", current_v_, current_w_);
    // RCLCPP_INFO(get_logger(), "current : v=%.3f, acc=%.3f, jerk=%0.3f", linear_planner_->getVelocity(), linear_planner_->getAcceleration(), linear_planner_->getJerk());
  }

  std::unique_ptr<TerminalMode> terminal_mode_;
  rclcpp::Publisher<byd_custom_msgs::msg::ControlRes>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double publish_rate_{50.0};
  double linear_speed_{0.2};
  double angular_speed_{0.5};
  double linear_accel_limit_{0.4};
  double linear_decel_limit_{0.8};
  double angular_accel_limit_{1.0};
  double angular_decel_limit_{2.0};
  double linear_accel_jerk_limit_{0.4};
  double linear_decel_jerk_limit_{0.8};
  double angular_accel_jerk_limit_{1.0};
  double angular_decel_jerk_limit_{2.0};
  double command_timeout_{0.5};

  std::unique_ptr<SCurvePlanner> linear_planner_;
  std::unique_ptr<SCurvePlanner> angular_planner_;

  double target_v_{0.0};
  double target_w_{0.0};
  double current_v_{0.0};
  double current_w_{0.0};
  Motion motion_{Motion::STOP};
  int escape_state_{0};
  std::chrono::steady_clock::time_point last_direction_key_{};
  std::chrono::steady_clock::time_point last_update_time_{};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<MyAgvKeyboardControl>();
    rclcpp::spin(node);
    RCLCPP_WARN(node->get_logger(), "Node shutting down, publishing final STOP command...");
    node->stop();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("myagv_keyboard_control"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
