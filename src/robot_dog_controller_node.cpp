#include "robot_dog_laser_designator/robot_dog_controller_node.hpp"

#include <algorithm>
#include <cmath>

namespace robot_dog_laser_designator
{

namespace
{
inline double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

GaitType gaitTypeFromString(const std::string & name, const rclcpp::Logger & logger)
{
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  if (lower == "trot") {return GaitType::TROT;}
  if (lower == "walk") {return GaitType::WALK;}
  if (lower == "pace") {return GaitType::PACE;}
  if (lower == "bound") {return GaitType::BOUND;}
  RCLCPP_WARN(logger, "Unknown gait_type '%s', falling back to 'trot'", name.c_str());
  return GaitType::TROT;
}
}  // namespace

RobotDogControllerNode::RobotDogControllerNode()
: Node("robot_dog_controller_node"),
  leg_kinematics_{
    LegKinematics(model_.geometry(), model_.mount(LegId::FL).is_left),
    LegKinematics(model_.geometry(), model_.mount(LegId::FR).is_left),
    LegKinematics(model_.geometry(), model_.mount(LegId::RL).is_left),
    LegKinematics(model_.geometry(), model_.mount(LegId::RR).is_left)},
  gait_(GaitType::TROT, /*step_frequency_hz=*/0.0, /*duty_factor=*/0.5)
{
  declareParameters();

  // Neutral foot targets: forward kinematics of the exact pose baked into
  // spot_zero.urdf's <ros2_control> initial_value fields, so cmd_vel == 0
  // reproduces exactly the pose the robot already spawns in (see
  // RobotDogModel::standingPoseAngles() for the rationale).
  const LegJointAngles standing = RobotDogModel::standingPoseAngles();
  for (LegId leg : kAllLegs) {
    const auto idx = static_cast<std::size_t>(leg);
    neutral_foot_position_[idx] = leg_kinematics_[idx].solveFk(standing);
  }

  const std::string cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
  const std::string imu_topic = get_parameter("imu_topic").as_string();
  const std::string joint_commands_topic = get_parameter("joint_commands_topic").as_string();
  const std::string posture_cmd_topic = get_parameter("posture_cmd_topic").as_string();

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(10),
    std::bind(&RobotDogControllerNode::cmdVelCallback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    std::bind(&RobotDogControllerNode::imuCallback, this, std::placeholders::_1));

  posture_sub_ = create_subscription<std_msgs::msg::String>(
    posture_cmd_topic, rclcpp::QoS(10),
    std::bind(&RobotDogControllerNode::postureCallback, this, std::placeholders::_1));

  joint_command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    joint_commands_topic, rclcpp::QoS(10));

  const auto now_time = now();
  last_cmd_time_ = now_time;
  last_loop_time_ = now_time;

  const auto period = std::chrono::duration<double>(1.0 / control_frequency_hz_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&RobotDogControllerNode::controlLoop, this));

  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&RobotDogControllerNode::onParametersSet, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "robot_dog_controller_node started: cmd_vel='%s' imu='%s' commands='%s' "
    "posture='%s' control_freq=%.1f Hz gait='%s'",
    cmd_vel_topic.c_str(), imu_topic.c_str(), joint_commands_topic.c_str(),
    posture_cmd_topic.c_str(), control_frequency_hz_,
    get_parameter("gait_type").as_string().c_str());
}

void RobotDogControllerNode::declareParameters()
{
  // --- Topics -----------------------------------------------------------
  declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  declare_parameter<std::string>("imu_topic", "/imu");
  declare_parameter<std::string>("joint_commands_topic", "/leg_position_controller/commands");
  declare_parameter<std::string>("posture_cmd_topic", "/posture_cmd");

  // --- Control loop -------------------------------------------------------
  declare_parameter<double>("control_frequency_hz", control_frequency_hz_);
  declare_parameter<double>("cmd_vel_timeout_sec", cmd_vel_timeout_sec_);

  // --- Teleop shaping -----------------------------------------------------
  declare_parameter<double>("linear_deadband", linear_deadband_);
  declare_parameter<double>("angular_deadband", angular_deadband_);
  declare_parameter<double>("max_linear_speed", max_linear_speed_);
  declare_parameter<double>("max_angular_speed", max_angular_speed_);
  declare_parameter<double>("max_stride", max_stride_);

  // --- Gait ---------------------------------------------------------------
  declare_parameter<std::string>("gait_type", "trot");
  declare_parameter<double>("duty_factor", gait_.dutyFactor());
  declare_parameter<double>("min_step_frequency_hz", min_step_frequency_hz_);
  declare_parameter<double>("max_step_frequency_hz", max_step_frequency_hz_);

  // --- Swing trajectory shape ---------------------------------------------
  declare_parameter<double>("step_height", trajectory_.params().step_height);
  declare_parameter<double>("control_point_fraction", trajectory_.params().control_point_fraction);

  // --- Body pose trim / IMU leveling --------------------------------------
  declare_parameter<bool>("use_imu_feedback", use_imu_feedback_);
  declare_parameter<double>("body_height_trim", body_pose_.params().body_height_trim);
  declare_parameter<double>("pitch_compensation_gain", body_pose_.params().pitch_compensation_gain);
  declare_parameter<double>("roll_compensation_gain", body_pose_.params().roll_compensation_gain);
  declare_parameter<double>("max_trim_z", body_pose_.params().max_trim_z);

  // --- Sit / stand posture -------------------------------------------------
  // Fixed joint-space target the robot blends into (in joint space, not via
  // IK) when "sit" is received on posture_cmd_topic; blends back out to
  // whatever the walking pipeline outputs when "stand" is received.
  declare_parameter<double>("sit_hip_roll", sit_angles_.hip_roll);
  declare_parameter<double>("sit_hip_pitch", sit_angles_.hip_pitch);
  declare_parameter<double>("sit_knee", sit_angles_.knee);
  declare_parameter<double>("sit_transition_duration_sec", sit_transition_duration_sec_);

  // Apply the just-declared values into the core objects / cached fields.
  control_frequency_hz_ = get_parameter("control_frequency_hz").as_double();
  cmd_vel_timeout_sec_ = get_parameter("cmd_vel_timeout_sec").as_double();
  linear_deadband_ = get_parameter("linear_deadband").as_double();
  angular_deadband_ = get_parameter("angular_deadband").as_double();
  max_linear_speed_ = get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = get_parameter("max_angular_speed").as_double();
  max_stride_ = get_parameter("max_stride").as_double();
  min_step_frequency_hz_ = get_parameter("min_step_frequency_hz").as_double();
  max_step_frequency_hz_ = get_parameter("max_step_frequency_hz").as_double();
  use_imu_feedback_ = get_parameter("use_imu_feedback").as_bool();

  applyGaitTypeParam(get_parameter("gait_type").as_string());
  gait_.setDutyFactor(get_parameter("duty_factor").as_double());

  TrajectoryGenerator::Params traj_params;
  traj_params.step_height = get_parameter("step_height").as_double();
  traj_params.control_point_fraction = get_parameter("control_point_fraction").as_double();
  trajectory_.setParams(traj_params);

  BodyPoseController::Params body_params;
  body_params.body_height_trim = get_parameter("body_height_trim").as_double();
  body_params.pitch_compensation_gain = get_parameter("pitch_compensation_gain").as_double();
  body_params.roll_compensation_gain = get_parameter("roll_compensation_gain").as_double();
  body_params.max_trim_z = get_parameter("max_trim_z").as_double();
  body_pose_.setParams(body_params);

  sit_angles_.hip_roll = get_parameter("sit_hip_roll").as_double();
  sit_angles_.hip_pitch = get_parameter("sit_hip_pitch").as_double();
  sit_angles_.knee = get_parameter("sit_knee").as_double();
  sit_transition_duration_sec_ = get_parameter("sit_transition_duration_sec").as_double();
}

void RobotDogControllerNode::applyGaitTypeParam(const std::string & name)
{
  gait_ = GaitEngine(gaitTypeFromString(name, get_logger()), gait_.stepFrequency(), gait_.dutyFactor());
}

rcl_interfaces::msg::SetParametersResult RobotDogControllerNode::onParametersSet(
  const std::vector<rclcpp::Parameter> & params)
{
  // Live-tunable subset -- topics / control_frequency_hz require a restart
  // since they're only used once, at construction time, to create
  // subscriptions/publishers/timers.
  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "gait_type") {
      applyGaitTypeParam(p.as_string());
    } else if (name == "duty_factor") {
      gait_.setDutyFactor(p.as_double());
    } else if (name == "min_step_frequency_hz") {
      min_step_frequency_hz_ = p.as_double();
    } else if (name == "max_step_frequency_hz") {
      max_step_frequency_hz_ = p.as_double();
    } else if (name == "linear_deadband") {
      linear_deadband_ = p.as_double();
    } else if (name == "angular_deadband") {
      angular_deadband_ = p.as_double();
    } else if (name == "max_linear_speed") {
      max_linear_speed_ = p.as_double();
    } else if (name == "max_angular_speed") {
      max_angular_speed_ = p.as_double();
    } else if (name == "max_stride") {
      max_stride_ = p.as_double();
    } else if (name == "cmd_vel_timeout_sec") {
      cmd_vel_timeout_sec_ = p.as_double();
    } else if (name == "use_imu_feedback") {
      use_imu_feedback_ = p.as_bool();
    } else if (name == "step_height" || name == "control_point_fraction") {
      TrajectoryGenerator::Params tp = trajectory_.params();
      if (name == "step_height") {tp.step_height = p.as_double();}
      if (name == "control_point_fraction") {tp.control_point_fraction = p.as_double();}
      trajectory_.setParams(tp);
    } else if (
      name == "body_height_trim" || name == "pitch_compensation_gain" ||
      name == "roll_compensation_gain" || name == "max_trim_z")
    {
      BodyPoseController::Params bp = body_pose_.params();
      if (name == "body_height_trim") {bp.body_height_trim = p.as_double();}
      if (name == "pitch_compensation_gain") {bp.pitch_compensation_gain = p.as_double();}
      if (name == "roll_compensation_gain") {bp.roll_compensation_gain = p.as_double();}
      if (name == "max_trim_z") {bp.max_trim_z = p.as_double();}
      body_pose_.setParams(bp);
    } else if (name == "sit_hip_roll") {
      sit_angles_.hip_roll = p.as_double();
    } else if (name == "sit_hip_pitch") {
      sit_angles_.hip_pitch = p.as_double();
    } else if (name == "sit_knee") {
      sit_angles_.knee = p.as_double();
    } else if (name == "sit_transition_duration_sec") {
      sit_transition_duration_sec_ = p.as_double();
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void RobotDogControllerNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_cmd_ = *msg;
  last_cmd_time_ = now();
  have_cmd_ = true;
}

void RobotDogControllerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const double x = msg->orientation.x;
  const double y = msg->orientation.y;
  const double z = msg->orientation.z;
  const double w = msg->orientation.w;

  // Roll/pitch from quaternion (standard formulas, REP-103 ENU convention:
  // X forward, Y left, Z up). Yaw is not needed here since only body
  // tilt -- not heading -- feeds into BodyPoseController.
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  const double pitch =
    (std::fabs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_roll_ = roll;
  latest_pitch_ = pitch;
  have_imu_ = true;
}

void RobotDogControllerNode::postureCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string cmd = msg->data;
  std::transform(cmd.begin(), cmd.end(), cmd.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });

  const TrickId trick = TrickPlayer::idFromString(cmd);
  if (trick != TrickId::None) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Tricks and sit are mutually exclusive: drop sit and queue the show.
    sit_target_ = 0.0;
    cancel_trick_ = false;
    pending_trick_ = trick;
    RCLCPP_INFO(get_logger(), "posture_cmd trick '%s'", TrickPlayer::toString(trick));
    return;
  }

  double target;
  if (cmd == "sit") {
    target = 1.0;
  } else if (cmd == "stand" || cmd == "up" || cmd == "stand_up") {
    target = 0.0;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Unknown posture_cmd '%s' (expected sit|stand|wave|play_bow|beg|shake), ignoring",
      msg->data.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  sit_target_ = target;
  // sit/stand preempt any running choreography.
  pending_trick_ = TrickId::None;
  cancel_trick_ = true;
}

void RobotDogControllerNode::controlLoop()
{
  const auto current_time = now();

  double dt = 1.0 / control_frequency_hz_;
  if (have_last_loop_time_) {
    const double measured_dt = (current_time - last_loop_time_).seconds();
    if (measured_dt > 0.0) {
      dt = measured_dt;
    }
  }
  last_loop_time_ = current_time;
  have_last_loop_time_ = true;

  geometry_msgs::msg::Twist cmd;
  bool cmd_is_stale = true;
  double roll = 0.0;
  double pitch = 0.0;
  bool imu_ok = false;
  double sit_target = 0.0;
  TrickId pending_trick = TrickId::None;
  bool cancel_trick = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cmd = latest_cmd_;
    if (have_cmd_) {
      // timeout <= 0: latch last cmd_vel until a new message arrives
      // (teleop_twist_keyboard one-shot keys). timeout > 0: hold-to-drive.
      if (cmd_vel_timeout_sec_ <= 0.0) {
        cmd_is_stale = false;
      } else {
        cmd_is_stale =
          (current_time - last_cmd_time_).seconds() > cmd_vel_timeout_sec_;
      }
    }
    roll = latest_roll_;
    pitch = latest_pitch_;
    imu_ok = have_imu_;
    sit_target = sit_target_;
    pending_trick = pending_trick_;
    pending_trick_ = TrickId::None;
    cancel_trick = cancel_trick_;
    cancel_trick_ = false;
  }

  if (cancel_trick) {
    trick_player_.cancel();
  }
  if (pending_trick != TrickId::None) {
    // Starting a trick clears any residual sit blend so the two posture
    // systems don't stack weights on the same joints.
    sit_blend_ = 0.0;
    sit_target = 0.0;
    trick_player_.start(pending_trick);
  }
  trick_player_.update(dt);

  // Advance the sit/stand joint-space blend toward its target at a fixed
  // rate (1 / sit_transition_duration_sec_ per second) so "sit" and "stand"
  // are smooth, time-based motions rather than instant pose snaps.
  // Skip while a trick owns the joints (sit was already forced to 0 above).
  if (!trick_player_.active()) {
    const double sit_rate = (sit_transition_duration_sec_ > 1e-6) ?
      1.0 / sit_transition_duration_sec_ : 1e9;
    if (sit_blend_ < sit_target) {
      sit_blend_ = std::min(sit_target, sit_blend_ + sit_rate * dt);
    } else if (sit_blend_ > sit_target) {
      sit_blend_ = std::max(sit_target, sit_blend_ - sit_rate * dt);
    }
  }
  // The robot cannot walk while sitting, mid sit-transition, or mid-trick:
  // force cmd_vel to zero so gait and posture never fight.
  const bool posture_locked =
    trick_player_.active() || (sit_target > 0.5) || (sit_blend_ > 1e-3);

  // Safety (only when timeout > 0): no recent teleop command -> stand still
  // rather than keep executing a possibly-stale velocity command.
  if (cmd_is_stale || posture_locked) {
    cmd = geometry_msgs::msg::Twist();
  }

  double vx = clampd(cmd.linear.x, -max_linear_speed_, max_linear_speed_);
  double vy = clampd(cmd.linear.y, -max_linear_speed_, max_linear_speed_);
  double wz = clampd(cmd.angular.z, -max_angular_speed_, max_angular_speed_);

  if (std::fabs(vx) < linear_deadband_) {vx = 0.0;}
  if (std::fabs(vy) < linear_deadband_) {vy = 0.0;}
  if (std::fabs(wz) < angular_deadband_) {wz = 0.0;}

  const bool standing_still = (vx == 0.0 && vy == 0.0 && wz == 0.0);

  double step_frequency = 0.0;
  if (!standing_still) {
    const double speed_ratio = clampd(
      std::max(
        std::hypot(vx, vy) / std::max(max_linear_speed_, 1e-6),
        std::fabs(wz) / std::max(max_angular_speed_, 1e-6)),
      0.0, 1.0);
    step_frequency = min_step_frequency_hz_ +
      speed_ratio * (max_step_frequency_hz_ - min_step_frequency_hz_);
  }

  // setStepFrequency() before update(): GaitEngine freezes its internal
  // clock whenever step_frequency == 0 (isStanding()), and resumes from
  // wherever it left off once it becomes nonzero again -- no foot pops.
  gait_.setStepFrequency(step_frequency);
  gait_.update(dt);

  const double eff_roll = (use_imu_feedback_ && imu_ok) ? roll : 0.0;
  const double eff_pitch = (use_imu_feedback_ && imu_ok) ? pitch : 0.0;

  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(12);
  std::size_t idx = 0;

  for (LegId leg : kAllLegs) {
    const auto leg_idx = static_cast<std::size_t>(leg);
    const LegMount & mount = model_.mount(leg);

    // Per-leg velocity = commanded body velocity + tangential velocity
    // from angular.z at this leg's mount point (v = v_body + omega x r).
    // This is what makes turning look natural: outer legs take a longer
    // stride than inner legs, instead of every leg doing an identical
    // stride offset by some artificial yaw term.
    const double leg_vx = vx - wz * mount.origin_in_base.y;
    const double leg_vy = vy + wz * mount.origin_in_base.x;

    double stride_x = 0.0;
    double stride_y = 0.0;
    if (step_frequency > 1e-6) {
      stride_x = leg_vx / step_frequency;
      stride_y = leg_vy / step_frequency;
    }

    // Safety clamp: guards against an unreachable IK target if a large
    // cmd_vel is combined with a very low step frequency.
    const double stride_mag = std::hypot(stride_x, stride_y);
    if (stride_mag > max_stride_ && stride_mag > 1e-9) {
      const double scale = max_stride_ / stride_mag;
      stride_x *= scale;
      stride_y *= scale;
    }

    const LegPhaseState phase = gait_.legPhaseState(leg);
    const Vec3 offset = trajectory_.computeFootOffset(phase, stride_x, stride_y);
    const Vec3 trim = body_pose_.computeTrim(mount.origin_in_base, eff_roll, eff_pitch);

    const Vec3 target{
      neutral_foot_position_[leg_idx].x + offset.x + trim.x,
      neutral_foot_position_[leg_idx].y + offset.y + trim.y,
      neutral_foot_position_[leg_idx].z + offset.z + trim.z,
    };

    LegJointAngles angles;
    const bool reachable = leg_kinematics_[leg_idx].solveIk(target, angles);
    if (!reachable) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s leg target unreachable, using clamped IK solution", toString(leg).c_str());
    }

    // Joint-space posture overlay. Tricks take priority over sit: both blend
    // in joint space (not via IK) so deep / asymmetric poses stay reachable
    // and every actuator moves monotonically between targets.
    if (trick_player_.active()) {
      const double w = trick_player_.blendWeight();
      const LegJointAngles & trick = trick_player_.targetPose().legs[leg_idx];
      msg.data[idx++] = (1.0 - w) * angles.hip_roll + w * trick.hip_roll;
      msg.data[idx++] = (1.0 - w) * angles.hip_pitch + w * trick.hip_pitch;
      msg.data[idx++] = (1.0 - w) * angles.knee + w * trick.knee;
    } else {
      const double w = sit_blend_;
      msg.data[idx++] = (1.0 - w) * angles.hip_roll + w * sit_angles_.hip_roll;
      msg.data[idx++] = (1.0 - w) * angles.hip_pitch + w * sit_angles_.hip_pitch;
      msg.data[idx++] = (1.0 - w) * angles.knee + w * sit_angles_.knee;
    }
  }

  joint_command_pub_->publish(msg);
}

}  // namespace robot_dog_laser_designator
