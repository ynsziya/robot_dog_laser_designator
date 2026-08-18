#include <algorithm>
#include <cmath>
#include <cstring>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{

/// Gazebo gpu_lidar clouds contain NaN/Inf no-returns and have no per-point
/// time. FAST-LIO then either builds a corrupt map or sets lidar_end_time ==
/// lidar_beg_time (curvature 0) and reports "No Effective Points!".
/// This node strips invalid returns and republishes a Velodyne-style cloud
/// with time=scan_period so every point is treated as a snapshot at scan end.
class FastLioBridgeNode : public rclcpp::Node
{
public:
  FastLioBridgeNode()
  : Node("fast_lio_bridge_node")
  {
    const auto imu_in = declare_parameter<std::string>("imu_input_topic", "/imu");
    const auto imu_out = declare_parameter<std::string>("imu_output_topic", "/imu_fastlio");
    const auto cloud_in = declare_parameter<std::string>("cloud_input_topic", "/scan/points");
    const auto cloud_out = declare_parameter<std::string>("cloud_output_topic", "/fast_lio/points");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "camera_init");
    child_frame_ = declare_parameter<std::string>("child_frame", "body");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/Odometry");
    const double tf_rate = declare_parameter<double>("tf_rate_hz", 50.0);
    min_range_ = declare_parameter<double>("min_range", 0.5);
    max_range_ = declare_parameter<double>("max_range", 29.0);
    scan_period_ = declare_parameter<double>("scan_period", 0.1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rclcpp::QoS reliable(50);
    reliable.reliable();

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_out, reliable);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_in, rclcpp::SensorDataQoS(),
      std::bind(&FastLioBridgeNode::imuCallback, this, std::placeholders::_1));

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_out, rclcpp::SensorDataQoS());
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_in, rclcpp::SensorDataQoS(),
      std::bind(&FastLioBridgeNode::cloudCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, reliable,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        last_odom_ = msg;
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, tf_rate));
    tf_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FastLioBridgeNode::publishTf, this));

    RCLCPP_INFO(
      get_logger(),
      "FAST-LIO bridge: IMU %s -> %s; cloud %s -> %s; TF %s -> %s",
      imu_in.c_str(), imu_out.c_str(), cloud_in.c_str(), cloud_out.c_str(),
      parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  static constexpr double kGravity = 9.80665;

  void imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    auto out = *msg;
    auto & acc = out.linear_acceleration;
    double n = std::hypot(acc.x, acc.y, acc.z);

    // Gazebo can publish ~0 (or a huge spike) for the first physics ticks.
    // FAST-LIO then estimates gravity from noise and the pose integrates
    // upward, taking the registered cloud with it.
    if (n < 5.0) {
      acc.z += kGravity;
      n = std::hypot(acc.x, acc.y, acc.z);
    }
    if (n < 5.0 || n > 20.0) {
      return;
    }
    imu_pub_->publish(out);
  }

  struct OutPoint
  {
    float x;
    float y;
    float z;
    float intensity;
    float time;
    uint16_t ring;
    uint16_t pad{0};
  };

  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (msg->width == 0 || msg->data.empty()) {
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    const bool has_intensity = std::any_of(
      msg->fields.begin(), msg->fields.end(),
      [](const auto & f) {
        return f.name == "intensity" &&
          f.datatype == sensor_msgs::msg::PointField::FLOAT32;
      });

    std::vector<OutPoint> kept;
    kept.reserve(msg->width * std::max<uint32_t>(msg->height, 1));

    if (has_intensity) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
        maybeKeep(*iter_x, *iter_y, *iter_z, *iter_i, kept);
      }
    } else {
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        maybeKeep(*iter_x, *iter_y, *iter_z, 0.0f, kept);
      }
    }

    if (kept.size() < 16) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "FAST-LIO cloud converter kept only %zu points (need finite returns)",
        kept.size());
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.width = static_cast<uint32_t>(kept.size());
    out.is_dense = true;
    out.is_bigendian = false;
    out.fields.resize(6);
    auto set_field = [](sensor_msgs::msg::PointField & f, const char * name,
                        uint32_t offset, uint8_t dtype, uint32_t count) {
      f.name = name;
      f.offset = offset;
      f.datatype = dtype;
      f.count = count;
    };
    set_field(out.fields[0], "x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
    set_field(out.fields[1], "y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
    set_field(out.fields[2], "z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
    set_field(out.fields[3], "intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1);
    set_field(out.fields[4], "time", 16, sensor_msgs::msg::PointField::FLOAT32, 1);
    set_field(out.fields[5], "ring", 20, sensor_msgs::msg::PointField::UINT16, 1);
    out.point_step = sizeof(OutPoint);
    out.row_step = out.point_step * out.width;
    out.data.resize(out.row_step);
    std::memcpy(out.data.data(), kept.data(), out.data.size());
    cloud_pub_->publish(out);
  }

  void maybeKeep(
    float x, float y, float z, float intensity, std::vector<OutPoint> & kept) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return;
    }
    const float r2 = x * x + y * y + z * z;
    if (r2 < min_range_ * min_range_ || r2 > max_range_ * max_range_) {
      return;
    }
    OutPoint pt{};
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.intensity = std::isfinite(intensity) ? intensity : 0.0f;
    // Snapshot lidar: all points share the scan-end offset (seconds).
    pt.time = static_cast<float>(scan_period_);
    pt.ring = 0;
    kept.push_back(pt);
  }

  void publishTf()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id = child_frame_;
    // Stamp with current sim time (not the lagged odom stamp). RViz's
    // message_filter looks up TF at the sensor message time; using a stale
    // odom stamp makes /scan/points and /camera/points flicker Error/Ok.
    tf.header.stamp = now();

    if (last_odom_) {
      tf.transform.translation.x = last_odom_->pose.pose.position.x;
      tf.transform.translation.y = last_odom_->pose.pose.position.y;
      tf.transform.translation.z = last_odom_->pose.pose.position.z;
      tf.transform.rotation = last_odom_->pose.pose.orientation;
    } else {
      tf.transform.rotation.w = 1.0;
    }

    tf_broadcaster_->sendTransform(tf);
  }

  std::string parent_frame_;
  std::string child_frame_;
  double min_range_{0.5};
  double max_range_{29.0};
  double scan_period_{0.1};
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastLioBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
