#include "robot_dog_laser_designator/keyboard_teleop_node.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>

namespace robot_dog_laser_designator
{

KeyboardTeleopNode::KeyboardTeleopNode()
: Node("robot_dog_teleop_node")
{
  declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  declare_parameter<std::string>("posture_cmd_topic", "/posture_cmd");
  declare_parameter<double>("linear_speed", linear_speed_);
  declare_parameter<double>("angular_speed", angular_speed_);
  declare_parameter<double>("linear_step", linear_step_);
  declare_parameter<double>("angular_step", angular_step_);
  declare_parameter<double>("max_linear_speed", max_linear_speed_);
  declare_parameter<double>("max_angular_speed", max_angular_speed_);

  linear_speed_ = get_parameter("linear_speed").as_double();
  angular_speed_ = get_parameter("angular_speed").as_double();
  linear_step_ = get_parameter("linear_step").as_double();
  angular_step_ = get_parameter("angular_step").as_double();
  max_linear_speed_ = get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = get_parameter("max_angular_speed").as_double();

  const std::string cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
  const std::string posture_cmd_topic = get_parameter("posture_cmd_topic").as_string();

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));
  posture_pub_ = create_publisher<std_msgs::msg::String>(posture_cmd_topic, rclcpp::QoS(10));

  setupTerminal();
  printHelp();
}

KeyboardTeleopNode::~KeyboardTeleopNode()
{
  // Always leave the user's shell in a sane state, even on Ctrl-C.
  restoreTerminal();
  std::printf("\n");
}

void KeyboardTeleopNode::setupTerminal()
{
  if (!isatty(STDIN_FILENO)) {
    RCLCPP_WARN(
      get_logger(),
      "stdin is not a terminal -- keyboard input will not work (run this node "
      "interactively, not piped/redirected).");
    return;
  }
  tcgetattr(STDIN_FILENO, &original_termios_);
  struct termios raw = original_termios_;
  // Disable line buffering (ICANON) and local echo so single keystrokes are
  // delivered immediately without the user seeing them typed back.
  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  terminal_modified_ = true;
}

void KeyboardTeleopNode::restoreTerminal()
{
  if (terminal_modified_) {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    terminal_modified_ = false;
  }
}

int KeyboardTeleopNode::readKey(int timeout_ms)
{
  struct pollfd pfd{STDIN_FILENO, POLLIN, 0};
  const int ret = poll(&pfd, 1, timeout_ms);
  if (ret > 0 && (pfd.revents & POLLIN)) {
    char c = 0;
    if (::read(STDIN_FILENO, &c, 1) == 1) {
      return static_cast<unsigned char>(c);
    }
  }
  return -1;
}

void KeyboardTeleopNode::handleKey(int key)
{
  constexpr int kCtrlC = 3;
  switch (key) {
    case 'w': case 'W':
      current_twist_.linear.x = linear_speed_;
      current_twist_.linear.y = 0.0;
      current_twist_.angular.z = 0.0;
      publishTwist();
      break;
    case 's': case 'S':
      current_twist_.linear.x = -linear_speed_;
      current_twist_.linear.y = 0.0;
      current_twist_.angular.z = 0.0;
      publishTwist();
      break;
    case 'a': case 'A':
      current_twist_.linear.x = 0.0;
      current_twist_.linear.y = linear_speed_;
      current_twist_.angular.z = 0.0;
      publishTwist();
      break;
    case 'd': case 'D':
      current_twist_.linear.x = 0.0;
      current_twist_.linear.y = -linear_speed_;
      current_twist_.angular.z = 0.0;
      publishTwist();
      break;
    case 'q': case 'Q':
      current_twist_.linear.x = 0.0;
      current_twist_.linear.y = 0.0;
      current_twist_.angular.z = angular_speed_;
      publishTwist();
      break;
    case 'e': case 'E':
      current_twist_.linear.x = 0.0;
      current_twist_.linear.y = 0.0;
      current_twist_.angular.z = -angular_speed_;
      publishTwist();
      break;
    case 'x': case 'X': case ' ':
      current_twist_ = geometry_msgs::msg::Twist();
      publishTwist();
      break;
    case 'z': case 'Z':
      publishPosture("sit");
      break;
    case 'c': case 'C':
      publishPosture("stand");
      break;
    case 'r': case 'R':
      publishPosture("wave");
      break;
    case 'f': case 'F':
      publishPosture("play_bow");
      break;
    case 'g': case 'G':
      publishPosture("beg");
      break;
    case 't': case 'T':
      publishPosture("shake");
      break;
    case 'v': case 'V':
      linear_speed_ = std::min(max_linear_speed_, linear_speed_ + linear_step_);
      angular_speed_ = std::min(max_angular_speed_, angular_speed_ + angular_step_);
      printStatus();
      break;
    case 'b': case 'B':
      linear_speed_ = std::max(0.0, linear_speed_ - linear_step_);
      angular_speed_ = std::max(0.0, angular_speed_ - angular_step_);
      printStatus();
      break;
    case kCtrlC:
      rclcpp::shutdown();
      break;
    default:
      break;
  }
}

void KeyboardTeleopNode::publishTwist()
{
  cmd_vel_pub_->publish(current_twist_);
  printStatus();
}

void KeyboardTeleopNode::publishPosture(const std::string & posture)
{
  current_posture_ = posture;
  // Sitting and walking are mutually exclusive: zero cmd_vel whenever the
  // posture changes so a leftover latched twist can't fight the posture
  // blend in RobotDogControllerNode.
  current_twist_ = geometry_msgs::msg::Twist();
  cmd_vel_pub_->publish(current_twist_);

  std_msgs::msg::String msg;
  msg.data = posture;
  posture_pub_->publish(msg);
  printStatus();
}

void KeyboardTeleopNode::printHelp() const
{
  std::printf(
    "\n"
    "robot_dog_laser_designator klavye teleop\n"
    "----------------------------\n"
    "Hareket:\n"
    "  w / s     : ileri / geri\n"
    "  a / d     : sola / sağa kayma (strafe)\n"
    "  q / e     : sola / sağa dönüş (yaw)\n"
    "  x / BOŞLUK: dur\n"
    "Duruş:\n"
    "  z         : otur (sit)\n"
    "  c         : kalk / ayakta dur (stand)\n"
    "Gösteri:\n"
    "  r         : selam (wave)\n"
    "  f         : play bow\n"
    "  g         : dilenci (beg)\n"
    "  t         : body shake\n"
    "Hız:\n"
    "  v / b     : hızı artır / azalt\n"
    "Çıkış: Ctrl-C\n"
    "\n");
}

void KeyboardTeleopNode::printStatus() const
{
  std::printf(
    "\r[hız: lin=%.2f m/s ang=%.2f rad/s | vx=%.2f vy=%.2f wz=%.2f | duruş=%s]   ",
    linear_speed_, angular_speed_,
    current_twist_.linear.x, current_twist_.linear.y, current_twist_.angular.z,
    current_posture_.c_str());
  std::fflush(stdout);
}

void KeyboardTeleopNode::run()
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->get_node_base_interface());

  while (rclcpp::ok()) {
    const int key = readKey(/*timeout_ms=*/50);
    if (key >= 0) {
      handleKey(key);
    }
    executor.spin_some();
  }
}

}  // namespace robot_dog_laser_designator
