#ifndef ROBOT_DOG_LASER_DESIGNATOR__ROBOT_DOG_MODEL_HPP_
#define ROBOT_DOG_LASER_DESIGNATOR__ROBOT_DOG_MODEL_HPP_

#include <array>
#include <string>

namespace robot_dog_laser_designator
{

/// Identifies each of the 4 legs. Index order MUST match joint_names.yaml
/// grouping (fl, fr, rl, rr) so that flattened joint command arrays line up
/// with spot_controllers.yaml's `leg_position_controller.joints` order.
enum class LegId : std::size_t
{
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3
};

constexpr std::array<LegId, 4> kAllLegs = {LegId::FL, LegId::FR, LegId::RL, LegId::RR};

std::string toString(LegId leg);

/// True for legs on the left side of the robot (+Y), false for right side (-Y).
bool isLeftLeg(LegId leg);

/// True for front legs (+X), false for rear legs (-X).
bool isFrontLeg(LegId leg);

/// A 3D point / vector in a Cartesian frame (meters).
struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// The 3 actuated joint angles of one leg, in radians.
/// Order matches the physical chain: hip_roll (abduction) -> hip_pitch -> knee.
/// NOTE: in this robot's URDF the joint literally named "*_hip_yaw_joint"
/// actually rotates about Y (i.e. it is a hip PITCH joint, not yaw). We keep
/// our own naming (hip_pitch) internally and map to the URDF joint names only
/// at the publishing boundary (see RobotDogModel::jointName()).
struct LegJointAngles
{
  double hip_roll{0.0};
  double hip_pitch{0.0};
  double knee{0.0};
};

/// Static geometric parameters shared by every leg (robot is left-right and
/// front-rear symmetric). All values derived from spot_zero.urdf + STL
/// bounding-box analysis (see conversation notes). Units: meters / radians.
struct LegGeometry
{
  /// Constant offset from hip_roll axis to hip_pitch axis along body-X.
  /// Does NOT participate in abduction trig (it lies along the roll axis).
  double hip_offset_x{0.050000};

  /// Abduction link length: hip_roll axis to hip_pitch axis, component
  /// perpendicular to the roll axis (in the Y-Z plane at zero abduction).
  double hip_offset_y{0.070000};

  /// Thigh length: hip_pitch axis to knee axis (3D norm from URDF).
  double thigh_length{0.4055};

  /// Shin length: knee axis to foot tip (derived from STL bounding box,
  /// since the URDF has no explicit foot link).
  double shin_length{0.4110};

  /// Joint limits [lower, upper] in radians, straight from spot_zero.urdf.
  struct Limits
  {
    double lower;
    double upper;
  };
  Limits hip_roll_limits{-0.239, 0.500};
  Limits hip_pitch_limits{-3.14, 3.14};
  Limits knee_limits{-2.631, -0.662};
};

/// Per-leg mounting position: hip_roll joint origin relative to base_link,
/// straight from spot_zero.urdf (see joint origins for *_hip_roll_joint).
struct LegMount
{
  LegId id;
  Vec3 origin_in_base;   ///< hip_roll joint origin, expressed in base_link frame
  bool is_left;
  bool is_front;
};

/// Holds the full kinematic description of the quadruped and provides
/// lookups needed by GaitEngine / LegKinematics / the controller node.
class RobotDogModel
{
public:
  RobotDogModel();

  const LegGeometry & geometry() const { return geometry_; }
  const LegMount & mount(LegId leg) const { return mounts_[static_cast<std::size_t>(leg)]; }

  /// URDF joint name for a given leg + joint slot, matching joint_names.yaml
  /// / spot_controllers.yaml exactly (uses "hip_yaw" naming even though it is
  /// physically a pitch joint, since that's the name in the URDF/controller).
  std::string jointNameHipRoll(LegId leg) const;
  std::string jointNameHipPitch(LegId leg) const;
  std::string jointNameKnee(LegId leg) const;

  /// Returns the 12 joint names in the exact order expected by
  /// leg_position_controller (spot_controllers.yaml), i.e.
  /// [fl_hip_roll, fl_hip_yaw, fl_knee, fr_..., rl_..., rr_...]
  std::array<std::string, 12> orderedJointNames() const;

  /// Body height (base_footprint -> base_link at neutral standing pose),
  /// from spot_zero.urdf base_footprint_joint origin.
  double nominalStandingHeight() const { return 0.7229; }

  /// The "safe" standing pose baked into spot_zero.urdf's <ros2_control>
  /// block as every joint's initial_value (see the comment there: "zero
  /// pose is outside knee limits"). We reuse this exact pose as the basis
  /// for each leg's neutral foot target (via forward kinematics) so that
  /// commanding zero velocity reproduces precisely the configuration the
  /// robot already spawns in, instead of an independently-guessed pose.
  static LegJointAngles standingPoseAngles() { return LegJointAngles{0.0, 0.764, -1.646}; }

private:
  LegGeometry geometry_;
  std::array<LegMount, 4> mounts_;
};

}  // namespace robot_dog_laser_designator

#endif  // ROBOT_DOG_LASER_DESIGNATOR__ROBOT_DOG_MODEL_HPP_
