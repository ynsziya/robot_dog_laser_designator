#ifndef ROBOT_DOG_LASER_DESIGNATOR__BODY_POSE_CONTROLLER_HPP_
#define ROBOT_DOG_LASER_DESIGNATOR__BODY_POSE_CONTROLLER_HPP_

#include "robot_dog_laser_designator/robot_dog_model.hpp"

namespace robot_dog_laser_designator
{

/// Produces a small per-leg foot-height trim used to (a) raise/lower the
/// robot's standing height and (b) optionally counteract measured body
/// roll/pitch (from an IMU) by shifting each foot's target height according
/// to its lever arm from the body center -- the standard linearized
/// "body leveling" trick used by most quadruped stacks (CHAMP included):
/// tilting the body is, to first order, equivalent to moving each foot up
/// or down in proportion to its distance from the body's rotation center.
///
/// This is intentionally a small, stateless, easily-testable class -- NOT a
/// full balance controller (no integral term, no per-axis independent
/// control loop). With the default zero gains it only applies
/// `body_height_trim`, so it is safe to wire into the controller node
/// before IMU-based tilt compensation has been tuned; enable the
/// compensation gains later once you've verified sign conventions in
/// simulation (they depend on the IMU's mounting orientation and on your
/// convention for roll/pitch sign, so treat the defaults as a starting
/// point, not a guaranteed-correct value).
class BodyPoseController
{
public:
  struct Params
  {
    /// Uniform vertical shift applied to every leg's neutral foot target,
    /// in meters. Positive = feet pushed further down = body stands taller
    /// (more extended legs); negative = crouched stance.
    double body_height_trim{0.0};

    /// Gain converting measured pitch (rad) x this leg's X lever arm (m,
    /// distance from body center along forward axis) into a foot-height
    /// correction (m). 0 = tilt compensation disabled.
    double pitch_compensation_gain{0.0};

    /// Gain converting measured roll (rad) x this leg's Y lever arm (m,
    /// distance from body center along the left axis) into a foot-height
    /// correction (m). 0 = tilt compensation disabled.
    double roll_compensation_gain{0.0};

    /// Safety clamp on the total |z| correction this class will ever
    /// produce, in meters. Prevents a bad gain or a noisy/glitchy IMU
    /// reading from commanding an unreachable IK target.
    double max_trim_z{0.05};
  };

  BodyPoseController();
  explicit BodyPoseController(const Params & params);

  void setParams(const Params & params) { params_ = params; }
  const Params & params() const { return params_; }

  /// mount_origin_in_base: this leg's hip mount offset from the body
  /// center, i.e. RobotDogModel::mount(leg).origin_in_base.
  /// measured_roll / measured_pitch: current body tilt in radians. Pass
  /// 0.0 for both when no IMU feedback is being used (or hasn't been
  /// enabled yet) -- the trim then reduces to just the height offset.
  /// Returns a Vec3 offset (in the leg-local frame) to add on top of the
  /// leg's neutral + gait foot target, before running IK.
  Vec3 computeTrim(const Vec3 & mount_origin_in_base, double measured_roll, double measured_pitch) const;

private:
  Params params_;
};

}  // namespace robot_dog_laser_designator

#endif  // ROBOT_DOG_LASER_DESIGNATOR__BODY_POSE_CONTROLLER_HPP_
