#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode()
  : Node("keyboard_teleop_node")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint1", "joint2", "joint3", "joint4"});
    joint_states_topic_ = declare_parameter<std::string>(
      "joint_states_topic", "cone/joint_states");
    joint_commands_topic_ = declare_parameter<std::string>(
      "joint_commands_topic", "cone/joint_commands");
    step_rad_ = declare_parameter<double>("step_rad", 0.03);
    poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 50.0);
    joint_min_ = declare_parameter<std::vector<double>>(
      "joint_min", {-1.5708, -1.5708, -1.5708, -3.1416});
    joint_max_ = declare_parameter<std::vector<double>>(
      "joint_max", { 1.5708,  1.5708,  1.5708,  3.1416});

    if (joint_names_.size() != 4) {
      throw std::runtime_error("keyboard_teleop expects exactly 4 joint names");
    }
    if (joint_min_.size() != joint_names_.size() || joint_max_.size() != joint_names_.size()) {
      throw std::runtime_error("joint_min/joint_max must have the same size as joint_names");
    }
    if (step_rad_ <= 0.0) {
      throw std::runtime_error("step_rad must be > 0");
    }
    if (poll_rate_hz_ <= 0.0) {
      throw std::runtime_error("poll_rate_hz must be > 0");
    }

    measured_position_.assign(joint_names_.size(), 0.0);
    target_position_.assign(joint_names_.size(), 0.0);
    joint_received_.assign(joint_names_.size(), false);

    command_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_commands_topic_, 10);
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, 10,
      std::bind(&KeyboardTeleopNode::jointStateCallback, this, std::placeholders::_1));

    setupTerminal();

    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    key_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&KeyboardTeleopNode::pollKeyboard, this));

    printHelp();
    RCLCPP_INFO(
      get_logger(),
      "Waiting for all 4 joints on /%s before accepting commands...",
      joint_states_topic_.c_str());
  }

  ~KeyboardTeleopNode() override
  {
    restoreTerminal();
  }

private:
  void setupTerminal()
  {
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_WARN(
        get_logger(),
        "stdin is not a TTY. Run this node directly from an interactive terminal for keyboard input.");
      return;
    }

    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
      RCLCPP_WARN(get_logger(), "tcgetattr() failed; keyboard input disabled");
      return;
    }

    struct termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      RCLCPP_WARN(get_logger(), "tcsetattr() failed; keyboard input disabled");
      return;
    }

    original_stdin_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_stdin_flags_ >= 0) {
      fcntl(STDIN_FILENO, F_SETFL, original_stdin_flags_ | O_NONBLOCK);
    }
    terminal_configured_ = true;
  }

  void restoreTerminal()
  {
    if (!terminal_configured_) {
      return;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    if (original_stdin_flags_ >= 0) {
      fcntl(STDIN_FILENO, F_SETFL, original_stdin_flags_);
    }
    terminal_configured_ = false;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k) {
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (msg->name[k] == joint_names_[i]) {
          measured_position_[i] = msg->position[k];
          joint_received_[i] = true;
          break;
        }
      }
    }

    if (!command_initialized_ && allJointsReceived()) {
      // The Dynamixel driver defines the startup pose as 0 rad. Still initialize from
      // measured feedback so this node remains safe if that behavior changes later.
      target_position_ = measured_position_;
      command_initialized_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Joint states received. Teleop ready. Initial target = [%.3f, %.3f, %.3f, %.3f] rad",
        target_position_[0], target_position_[1], target_position_[2], target_position_[3]);
    }
  }

  bool allJointsReceived() const
  {
    return std::all_of(joint_received_.begin(), joint_received_.end(), [](bool v) {return v;});
  }

  void pollKeyboard()
  {
    if (!terminal_configured_) {
      return;
    }

    char key = 0;
    while (read(STDIN_FILENO, &key, 1) > 0) {
      handleKey(key);
    }
  }

  void handleKey(char key)
  {
    if (key == 'x' || key == 'X' || key == 27) {
      if (command_initialized_) {
        holdMeasuredPosition();
      }
      RCLCPP_INFO(get_logger(), "Exiting keyboard teleop");
      restoreTerminal();
      rclcpp::shutdown();
      return;
    }

    if (key == '?') {
      printHelp();
      return;
    }

    if (!command_initialized_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for /%s joint feedback; command ignored",
        joint_states_topic_.c_str());
      return;
    }

    bool changed = false;
    switch (key) {
      case 'q': case 'Q': changed = jogJoint(0, +step_rad_); break;
      case 'a': case 'A': changed = jogJoint(0, -step_rad_); break;
      case 'w': case 'W': changed = jogJoint(1, +step_rad_); break;
      case 's': case 'S': changed = jogJoint(1, -step_rad_); break;
      case 'e': case 'E': changed = jogJoint(2, +step_rad_); break;
      case 'd': case 'D': changed = jogJoint(2, -step_rad_); break;
      case 'r': case 'R': changed = jogJoint(3, +step_rad_); break;
      case 'f': case 'F': changed = jogJoint(3, -step_rad_); break;

      case 'h': case 'H':
        holdMeasuredPosition();
        return;

      case '0':
        target_position_.assign(joint_names_.size(), 0.0);
        clampAllTargets();
        publishCommand();
        RCLCPP_INFO(get_logger(), "Commanding startup-zero pose [0, 0, 0, 0]");
        return;

      case '[':
        step_rad_ = std::max(0.001, step_rad_ / 2.0);
        RCLCPP_INFO(get_logger(), "step_rad = %.4f rad (%.2f deg)", step_rad_, step_rad_ * 180.0 / kPi);
        return;

      case ']':
        step_rad_ = std::min(0.5, step_rad_ * 2.0);
        RCLCPP_INFO(get_logger(), "step_rad = %.4f rad (%.2f deg)", step_rad_, step_rad_ * 180.0 / kPi);
        return;

      case 'p': case 'P':
        printState();
        return;

      default:
        return;
    }

    if (changed) {
      publishCommand();
    }
  }

  bool jogJoint(size_t index, double delta)
  {
    const double before = target_position_[index];
    const double requested = before + delta;
    target_position_[index] = std::clamp(requested, joint_min_[index], joint_max_[index]);

    if (target_position_[index] != requested) {
      RCLCPP_WARN(
        get_logger(), "%s target limited to %.3f rad",
        joint_names_[index].c_str(), target_position_[index]);
    }

    RCLCPP_INFO(
      get_logger(), "%s target: %.3f -> %.3f rad",
      joint_names_[index].c_str(), before, target_position_[index]);
    return target_position_[index] != before;
  }

  void holdMeasuredPosition()
  {
    target_position_ = measured_position_;
    clampAllTargets();
    publishCommand();
    RCLCPP_INFO(
      get_logger(), "Hold measured pose: [%.3f, %.3f, %.3f, %.3f] rad",
      target_position_[0], target_position_[1], target_position_[2], target_position_[3]);
  }

  void clampAllTargets()
  {
    for (size_t i = 0; i < target_position_.size(); ++i) {
      target_position_[i] = std::clamp(target_position_[i], joint_min_[i], joint_max_[i]);
    }
  }

  void publishCommand()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position = target_position_;
    command_pub_->publish(msg);
  }

  void printState() const
  {
    RCLCPP_INFO(
      get_logger(),
      "meas=[%.3f %.3f %.3f %.3f], target=[%.3f %.3f %.3f %.3f] rad",
      measured_position_[0], measured_position_[1], measured_position_[2], measured_position_[3],
      target_position_[0], target_position_[1], target_position_[2], target_position_[3]);
  }

  void printHelp() const
  {
    RCLCPP_INFO(
      get_logger(),
      "\n"
      "================ 4-DoF Arm Keyboard Teleop ================\n"
      " joint1 : q (+) / a (-)\n"
      " joint2 : w (+) / s (-)\n"
      " joint3 : e (+) / d (-)\n"
      " joint4 : r (+) / f (-)\n"
      " h      : hold current measured pose\n"
      " 0      : move to startup-zero pose [0,0,0,0]\n"
      " [ / ]  : decrease / increase jog step\n"
      " p      : print measured + target positions\n"
      " ?      : print this help\n"
      " x/ESC  : hold current pose and exit\n"
      "===========================================================\n");
  }

  std::vector<std::string> joint_names_;
  std::string joint_states_topic_;
  std::string joint_commands_topic_;
  std::vector<double> joint_min_;
  std::vector<double> joint_max_;
  std::vector<double> measured_position_;
  std::vector<double> target_position_;
  std::vector<bool> joint_received_;

  double step_rad_{0.03};
  double poll_rate_hz_{50.0};
  bool command_initialized_{false};

  struct termios original_termios_ {};
  int original_stdin_flags_{-1};
  bool terminal_configured_{false};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr key_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("keyboard_teleop_node"), "Initialization failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
