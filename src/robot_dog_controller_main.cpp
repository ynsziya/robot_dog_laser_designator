#include <rclcpp/rclcpp.hpp>

#include "robot_dog_laser_designator/robot_dog_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_dog_laser_designator::RobotDogControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
