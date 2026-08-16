#include "robot_dog_laser_designator/trajectory_generator.hpp"

#include "robot_dog_laser_designator/bezier_curve.hpp"

namespace robot_dog_laser_designator
{

TrajectoryGenerator::TrajectoryGenerator()
: params_(Params())
{
}

TrajectoryGenerator::TrajectoryGenerator(const Params & params)
: params_(params)
{
}

Vec3 TrajectoryGenerator::computeFootOffset(
  const LegPhaseState & phase, double stride_x, double stride_y) const
{
  if (phase.type == GaitPhaseType::STANCE) {
    return computeStanceOffset(phase.progress, stride_x, stride_y);
  }
  return computeSwingOffset(phase.progress, stride_x, stride_y);
}

Vec3 TrajectoryGenerator::computeStanceOffset(
  double progress, double stride_x, double stride_y) const
{
  // Foot travels linearly from +stride/2 (just touched down, ahead of body)
  // to -stride/2 (about to lift off, behind body). This is what pushes the
  // body forward -- z stays at 0 (foot flat on the ground the whole time).
  const double t = 0.5 - progress;  // +0.5 -> -0.5
  return Vec3{stride_x * t, stride_y * t, 0.0};
}

Vec3 TrajectoryGenerator::computeSwingOffset(
  double progress, double stride_x, double stride_y) const
{
  const Vec3 start{-0.5 * stride_x, -0.5 * stride_y, 0.0};
  const Vec3 end{0.5 * stride_x, 0.5 * stride_y, 0.0};

  // A cubic Bezier with both middle control points raised to the same
  // height only reaches 0.75x that height at its true peak (t=0.5) -- a
  // well-known property of the Bernstein basis. Scale by 4/3 so the
  // resulting curve's actual apex equals params_.step_height exactly.
  const double control_z = params_.step_height * (4.0 / 3.0);
  const double f = params_.control_point_fraction;
  const Vec3 p1{
    start.x + (end.x - start.x) * f,
    start.y + (end.y - start.y) * f,
    control_z
  };
  const Vec3 p2{
    start.x + (end.x - start.x) * (1.0 - f),
    start.y + (end.y - start.y) * (1.0 - f),
    control_z
  };

  CubicBezier curve(start, p1, p2, end);
  return curve.evaluate(progress);
}

}  // namespace robot_dog_laser_designator
