#ifndef ROBOT_DOG_LASER_DESIGNATOR__KEYBOARD_TELEOP_NODE_HPP_
#define ROBOT_DOG_LASER_DESIGNATOR__KEYBOARD_TELEOP_NODE_HPP_

#include <termios.h>

#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_dog_laser_designator
{

/// Purpose-built keyboard teleop for robot_dog_laser_designator: everything
/// teleop_twist_keyboard offers (WASD + turning, latched cmd_vel, live
/// speed adjustment) plus posture keys published as std_msgs/String on
/// posture_cmd_topic -- sit/stand plus showpiece tricks (wave, play_bow,
/// beg, shake) for RobotDogControllerNode's posture / TrickPlayer path.
///
/// Puts the controlling terminal into raw, non-canonical, non-blocking
/// mode (termios) so single keystrokes are read immediately without
/// waiting for Enter, then restores the original terminal settings on
/// destruction (including on Ctrl-C, via a signal handler) so the shell
/// is left in a usable state afterwards.
class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode();
  ~KeyboardTeleopNode() override;

  /// Blocks, polling the keyboard and driving rclcpp's executor, until
  /// rclcpp::ok() goes false (Ctrl-C).
  void run();

private:
  void setupTerminal();
  void restoreTerminal();

  /// Returns the next byte typed by the user, or -1 if none is available
  /// within timeout_ms (non-blocking read via poll()).
  int readKey(int timeout_ms);

  void handleKey(int key);
  void publishTwist();
  void publishPosture(const std::string & posture);
  void printHelp() const;
  void printStatus() const;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr posture_pub_;

  struct termios original_termios_{};
  bool terminal_modified_{false};

  double linear_speed_{0.4};    ///< m/s, current forward/strafe speed setting
  double angular_speed_{0.6};   ///< rad/s, current turn speed setting
  double linear_step_{0.05};    ///< m/s, +/- key increment
  double angular_step_{0.1};    ///< rad/s, +/- key increment
  double max_linear_speed_{0.55};
  double max_angular_speed_{1.2};

  geometry_msgs::msg::Twist current_twist_;
  std::string current_posture_{"stand"};
};

}  // namespace robot_dog_laser_designator

#endif  // ROBOT_DOG_LASER_DESIGNATOR__KEYBOARD_TELEOP_NODE_HPP_
