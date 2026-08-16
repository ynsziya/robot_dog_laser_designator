#include "robot_dog_laser_designator/leg_odometry_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <string>

namespace robot_dog_laser_designator 
{

    namespace 
    {
        /// Substring identifying a foot sphere collision in the names Gazebo
        /// reports (e.g. "bosdyn_spot::fl_knee::fl_knee_fixed_joint_lump__
        /// fl_foot_contact_sphere_collision_1").
        constexpr const char * kFootSphereMarker = "foot_contact_sphere";

        bool findJointPosition(
            const sensor_msgs::msg::JointState & msg,
            const std::string & name,
            double & out) 
        {
            for (std::size_t i = 0; i < msg.name.size(); ++i) {
                if (msg.name[i] == name) {
                    if (i >= msg.position.size()) {
                        return false;
                    }
                    out = msg.position[i];
                    return true;
                }
            }

            return false;
        }
    }   // namespace

    LegOdometryNode::LegOdometryNode() : Node("leg_odometry_node"), 
        leg_kinematics_{
            LegKinematics(model_.geometry(), model_.mount(LegId::FL).is_left),
            LegKinematics(model_.geometry(), model_.mount(LegId::FR).is_left),
            LegKinematics(model_.geometry(), model_.mount(LegId::RL).is_left),
            LegKinematics(model_.geometry(), model_.mount(LegId::RR).is_left)}
    {

        declareParameters();

        const std::string joint_states_topic = get_parameter("joint_states_topic").as_string();
        const std::string imu_topic = get_parameter("imu_topic").as_string();
        const std::string odom_topic = get_parameter("odom_topic").as_string();
        update_frequency_hz_ = get_parameter("update_frequency_hz").as_double();
        contact_timeout_sec_ = get_parameter("contact_timeout_sec").as_double();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic, rclcpp::QoS(10),
            std::bind(&LegOdometryNode::jointStateCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(), 
            std::bind(&LegOdometryNode::imuCallback, this, std::placeholders::_1));

        for  (LegId leg : kAllLegs) {
            const auto idx = static_cast<std::size_t>(leg);
            const std::string topic = "/foot_contact/" + toString(leg);
            contact_subs_[idx] = create_subscription<ros_gz_interfaces::msg::Contacts>(
                topic, rclcpp::SensorDataQoS(),
                [this, leg](const ros_gz_interfaces::msg::Contacts::SharedPtr msg) {
                    contactCallback(leg, msg);
                });
        }

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(10));

        last_update_time_ = now();

        const auto period = std::chrono::duration<double>(1.0 / update_frequency_hz_);
        update_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&LegOdometryNode::updateLoop, this));

        RCLCPP_INFO(
            get_logger(),
            "leg_odometry_node started: joint_states='%s' imu='%s' odom='%s' freq=%.1f Hz",
            joint_states_topic.c_str(), imu_topic.c_str(), odom_topic.c_str(),
            update_frequency_hz_);
    }

    void LegOdometryNode::declareParameters()
    {
        declare_parameter<std::string>("joint_states_topic", "/joint_states");
        declare_parameter<std::string>("imu_topic", "/imu");
        declare_parameter<std::string>("odom_topic", "/leg_odom");
        declare_parameter<double>("update_frequency_hz", update_frequency_hz_);
        declare_parameter<double>("contact_timeout_sec", contact_timeout_sec_);
        declare_parameter<std::string>("odom_frame", odom_frame_);
        declare_parameter<std::string>("base_frame", base_frame_);
    }

    void LegOdometryNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
        std::array<LegJointAngles, 4> angles;
        for (LegId leg : kAllLegs) {
            const auto idx = static_cast<std::size_t>(leg);
            if(!findJointPosition(*msg, model_.jointNameHipRoll(leg), angles[idx].hip_roll) ||
                !findJointPosition(*msg, model_.jointNameHipPitch(leg), angles[idx].hip_pitch) ||
                !findJointPosition(*msg, model_.jointNameKnee(leg), angles[idx].knee)) 
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "joint_states missing one or more leg joints, ignoring this message");
                  return;
            }
        }

        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_angles_ = angles;
        have_joint_state_ = true;
    }

    void LegOdometryNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const double x = msg->orientation.x;
        const double y = msg->orientation.y;
        const double z = msg->orientation.z;
        const double w = msg->orientation.w;

        // Yaw from quaternion (REP-103: X forward, Y left, Z up)
        const double siny_cosp = 2.0 * (w * z + x * y);
        const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        const double yaw = std::atan2(siny_cosp, cosy_cosp);

        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_yaw_ = yaw;
        have_imu_ = true;
    }

    void LegOdometryNode::contactCallback(LegId leg, const ros_gz_interfaces::msg::Contacts::SharedPtr msg) 
    {
        // Only the foot sphere counts as stance. The shin mesh grazes the
        // ground during swing, and treating that as stance makes the swing
        // legs (moving forward) cancel the stance legs (moving backward) in
        // the velocity average, leaving the odometry stuck near zero.
        //
        // Gazebo contact sensors also only publish WHILE touching — they do
        // not send empty messages on lift-off. So we stamp the last sphere
        // hit here; updateLoop expires stale stamps as "not in contact".
        bool sphere_hit = false;
        for (const auto & contact : msg->contacts) {
            if (contact.collision1.name.find(kFootSphereMarker) != std::string::npos ||
                contact.collision2.name.find(kFootSphereMarker) != std::string::npos)
            {
                sphere_hit = true;
                break;
            }
        }
        if (!sphere_hit) {
            return;
        }

        const auto idx = static_cast<std::size_t>(leg);
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_sphere_contact_time_[idx] = now();
        have_sphere_contact_stamp_[idx] = true;
    }

    void LegOdometryNode::updateLoop() 
    {
        // 1) Callback state'ini kopyala, kilidi hemen bırak
        std::array<LegJointAngles, 4> angles;
        std::array<bool, 4> in_contact{};
        double yaw = 0.0;
        bool have_joints = false;
        bool have_imu = false;
        const rclcpp::Time now_time = now();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            angles = latest_angles_;
            yaw = latest_yaw_;
            have_joints = have_joint_state_;
            have_imu = have_imu_;
            for (std::size_t i = 0; i < 4; ++i) {
                if (!have_sphere_contact_stamp_[i]) {
                    in_contact[i] = false;
                    continue;
                }
                const double age = (now_time - last_sphere_contact_time_[i]).seconds();
                in_contact[i] = age >= 0.0 && age <= contact_timeout_sec_;
            }
        }


        if (!have_joints) {
            return;
        }

        // 2) Her bacak için gövde-frame ayak pozisyonu (FK)
        std::array<Vec3, 4> foot_pos{};
        for (LegId leg : kAllLegs) {
            const auto idx = static_cast<std::size_t>(leg);
            const LegMount & mount = model_.mount(leg);
            const Vec3 local = leg_kinematics_[idx].solveFk(angles[idx]);
            foot_pos[idx] = Vec3{
                mount.origin_in_base.x + local.x,
                mount.origin_in_base.y + local.y,
                mount.origin_in_base.z + local.z};
        }

        // 3) dt: iki updateLoop çağrısı arasındaki süre
        double dt = 1.0 / update_frequency_hz_;
        if (have_last_update_time_) {
            const double measured_dt = (now_time - last_update_time_).seconds();
            if (measured_dt > 1e-6) {
                dt = measured_dt;
            }
        }
        last_update_time_ = now_time;
        have_last_update_time_ = true;


        // 4) Stance ayaklardan gövde hızı (önce hız, sonra prev güncelle)
        double sum_vx = 0.0;
        double sum_vy = 0.0;
        int stance_count = 0;

        for (LegId leg : kAllLegs) {
            const auto idx = static_cast<std::size_t>(leg);

            if (!in_contact[idx]) {
                have_prev_foot_[idx] = false;
                prev_in_contact_[idx] = false;
                continue;
            }

            const bool newly_in_contact = !prev_in_contact_[idx];
            if (newly_in_contact) {
                have_prev_foot_[idx] = false;
            }

            // Geçerli prev varsa bu ayaktan hız tahmini
            if (have_prev_foot_[idx] && dt > 1e-6) {
                const double dx = foot_pos[idx].x - prev_foot_pos_[idx].x;
                const double dy = foot_pos[idx].y - prev_foot_pos_[idx].y;
                // Ayak gövdeye göre +x kaydıysa gövde dünyada -x gitmiştir
                sum_vx += -dx / dt;
                sum_vy += -dy / dt;
                ++stance_count;
            }

            // Hız hesabından SONRA prev'i güncelle
            prev_foot_pos_[idx] = foot_pos[idx];
            have_prev_foot_[idx] = true;
            prev_in_contact_[idx] = true;

        }

        if (stance_count > 0) {
            vx_ = sum_vx / static_cast<double>(stance_count);
            vy_ = sum_vy / static_cast<double>(stance_count);
        } else {
            // Hiç stance ayak yok (veya hepsi yeni değdi): hız bilinmiyor → 0
            vx_ = 0.0;
            vy_ = 0.0;
        }

        // 5) Gövde hızını dünya çerçevesine çevir, konumu biriktir
        const double yaw_use = have_imu ? yaw : 0.0;
        const double c = std::cos(yaw_use);
        const double s = std::sin(yaw_use);

        const double vx_world = vx_ * c - vy_ * s;
        const double vy_world = vx_ * s + vy_ * c;
        
        x_ += vx_world * dt;
        y_ += vy_world * dt;

        // 6) nav_msgs/Odometry yayınla (TF yok; EKF yayınlayacak)
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now_time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // yaw → quaternion (sadece Z rotasyonu)
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = std::sin(yaw_use * 0.5);
        odom.pose.pose.orientation.w = std::cos(yaw_use * 0.5);

        // Hızlar child (base) frame'de: gövdeye göre vx_, vy_
        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.z = 0.0;   // yaw hızı IMU gyro'dan EKF'ye gidecek

        // Covariance (6x6 satır-major): x,y,z, roll,pitch,yaw
        // Büyük değer = "bu kaynağa az güven" → EKF IMU ile birleştirir
        for (int i = 0; i < 36; ++i) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }

        odom.pose.covariance[0] = 0.05;   // x
        odom.pose.covariance[7] = 0.05;   // y
        odom.pose.covariance[35] = 0.1;   // yaw (asıl kaynak IMU; burası zayıf)
        odom.twist.covariance[0] = 0.02;  // vx
        odom.twist.covariance[7] = 0.02;  // vy

        odom_pub_->publish(odom);

        
    }



}   // namespace