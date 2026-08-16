#ifndef ROBOT_DOG_LASER_DESIGNATOR__TRAJECTORY_GENERATOR_HPP_
#define ROBOT_DOG_LASER_DESIGNATOR__TRAJECTORY_GENERATOR_HPP_

#include "robot_dog_laser_designator/gait_engine.hpp"
#include "robot_dog_laser_designator/robot_dog_model.hpp"

namespace robot_dog_laser_designator
{

/// Turns a leg's phase state (from GaitEngine) into a concrete foot
/// position OFFSET (relative to that leg's neutral standing foot position),
/// in the leg's local frame (x forward, y left, z up).
///
/// - STANCE: straight line, foot moves from +stride/2 to -stride/2 as the
///   body travels over it (this is what actually propels the robot).
/// - SWING: cubic Bezier arc that lifts the foot, carries it forward, and
///   sets it back down -- shaped to have a fast, clean liftoff/touchdown
///   (matches the qualitative shape CHAMP and most trot controllers use).
class TrajectoryGenerator
{
public:
  struct Params
  {
    /// Peak foot clearance height during swing, meters.
    double step_height{0.06};
    /// Where along the swing horizontal span (0-1) the Bezier control
    /// points sit; smaller = punchier liftoff/touchdown, larger = floatier.
    double control_point_fraction{0.2};
  };

  TrajectoryGenerator();
  explicit TrajectoryGenerator(const Params & params);

  /// stride_x / stride_y: total peak-to-peak horizontal foot travel for
  /// this leg over one full gait cycle (meters), i.e. how far the foot
  /// swings forward during swing phase / walks backward during stance.
  /// Typically stride = commanded_velocity * (1 / step_frequency).
  /// Positive stride_x = robot moving forward, positive stride_y = moving
  /// left (strafing) or turning-induced lateral component.
  Vec3 computeFootOffset(const LegPhaseState & phase, double stride_x, double stride_y) const;

  void setParams(const Params & params) { params_ = params; }
  const Params & params() const { return params_; }

private:
  Vec3 computeStanceOffset(double progress, double stride_x, double stride_y) const;
  Vec3 computeSwingOffset(double progress, double stride_x, double stride_y) const;

  Params params_;
};

}  // namespace robot_dog_laser_designator

#endif  // ROBOT_DOG_LASER_DESIGNATOR__TRAJECTORY_GENERATOR_HPP_
