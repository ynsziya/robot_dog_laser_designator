#ifndef ROBOT_DOG_LASER_DESIGNATOR__LEG_ODOMETRY_NODE_HPP_
#define ROBOT_DOG_LASER_DESIGNATOR__LEG_ODOMETRY_NODE_HPP_

#include <array>
#include <mutex>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robot_dog_laser_designator/leg_kinematics.hpp"
#include "robot_dog_laser_designator/robot_dog_model.hpp"

namespace robot_dog_laser_designator
{

class LegOdometryNode : public rclcpp::Node
{
public:
  LegOdometryNode();

private:
  void declareParameters();
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void contactCallback(LegId leg, const ros_gz_interfaces::msg::Contacts::SharedPtr msg);
  void updateLoop();

  // --- ROS ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::array<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr, 4>
    contact_subs_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  // --- Kinematik (controller ile aynı çekirdek) ---
  RobotDogModel model_;
  std::array<LegKinematics, 4> leg_kinematics_;

  // --- Callback'lerden gelen son veri (timer okur → mutex) ---
  std::mutex state_mutex_;
  std::array<LegJointAngles, 4> latest_angles_{};
  bool have_joint_state_{false};
  // Gazebo contact sensors only publish while touching; once messages stop
  // the foot has lifted. We treat contact as "fresh" only within this window.
  std::array<rclcpp::Time, 4> last_sphere_contact_time_;
  std::array<bool, 4> have_sphere_contact_stamp_{};
  double contact_timeout_sec_{0.05};
  double latest_yaw_{0.0};
  bool have_imu_{false};

  rclcpp::Time last_update_time_;
  bool have_last_update_time_{false};

  std::array<Vec3, 4> prev_foot_pos_{};
  std::array<bool, 4> have_prev_foot_{};
  std::array<bool, 4> prev_in_contact_{};

  // --- Entegre edilmiş dünya konumu ---
  double x_{0.0};
  double y_{0.0};

  double update_frequency_hz_{100.0};

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};

  // --- Son hesaplanan gövde-frame hız (m/s), sadece updateLoop yazar ---
  double vx_{0.0};
  double vy_{0.0};
};

}  // namespace robot_dog_laser_designator

#endif