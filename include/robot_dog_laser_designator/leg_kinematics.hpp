#ifndef ROBOT_DOG_LASER_DESIGNATOR__LEG_KINEMATICS_HPP_
#define ROBOT_DOG_LASER_DESIGNATOR__LEG_KINEMATICS_HPP_

#include "robot_dog_laser_designator/robot_dog_model.hpp"

namespace robot_dog_laser_designator
{

/// Analytic inverse/forward kinematics for a single 3-DOF leg
/// (hip_roll -> hip_pitch -> knee), planar-leg simplification.
///
/// Frame convention (all in the LEG's local frame, i.e. relative to that
/// leg's hip_roll joint origin):
///   x: forward   y: left (toward hip_pitch axis for a left leg)   z: up
///
/// For right-side legs the geometry is mirrored: the hip_pitch axis sits at
/// -hip_offset_y instead of +hip_offset_y. Pass `is_left_leg` accordingly
/// (see RobotDogModel::mount(leg).is_left) -- this class does not know about
/// LegId, it only deals in geometry, so it stays reusable/testable in
/// isolation.
class LegKinematics
{
public:
  LegKinematics(const LegGeometry & geometry, bool is_left_leg);

  /// Solves joint angles that place the foot at `target` (in the leg's local
  /// frame, relative to the hip_roll joint). Returns false if the target is
  /// out of reach (distance > L2+L3 or < |L2-L3|), in which case `out` is
  /// still populated with the closest reachable approximation (clamped).
  bool solveIk(const Vec3 & target, LegJointAngles & out) const;

  /// Forward kinematics: joint angles -> foot position in leg-local frame.
  /// Used for self-consistency testing and for e.g. drawing a swing-phase
  /// trajectory preview.
  Vec3 solveFk(const LegJointAngles & angles) const;

private:
  LegGeometry geometry_;
  bool is_left_leg_;
  double L1_;  ///< signed abduction link length (+ for left, - for right)
};

}  // namespace robot_dog_laser_designator

#endif  // ROBOT_DOG_LASER_DESIGNATOR__LEG_KINEMATICS_HPP_
