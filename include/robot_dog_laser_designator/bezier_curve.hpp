#ifndef ROBOT_DOG_LASER_DESIGNATOR__BEZIER_CURVE_HPP_
#define ROBOT_DOG_LASER_DESIGNATOR__BEZIER_CURVE_HPP_

#include <array>

#include "robot_dog_laser_designator/robot_dog_model.hpp"

namespace robot_dog_laser_designator
{

/// A cubic Bezier curve (4 control points) over Vec3. Used by
/// TrajectoryGenerator to shape the swing-phase foot path so it lifts off
/// and touches down smoothly instead of with velocity discontinuities
/// (which is what causes "jerky"/unnatural looking swing motion).
class CubicBezier
{
public:
  CubicBezier() = default;
  CubicBezier(const Vec3 & p0, const Vec3 & p1, const Vec3 & p2, const Vec3 & p3)
  : points_{p0, p1, p2, p3}
  {}

  /// Evaluate the curve at parameter t in [0, 1].
  Vec3 evaluate(double t) const
  {
    const double u = 1.0 - t;
    const double b0 = u * u * u;
    const double b1 = 3.0 * u * u * t;
    const double b2 = 3.0 * u * t * t;
    const double b3 = t * t * t;
    return Vec3{
      b0 * points_[0].x + b1 * points_[1].x + b2 * points_[2].x + b3 * points_[3].x,
      b0 * points_[0].y + b1 * points_[1].y + b2 * points_[2].y + b3 * points_[3].y,
      b0 * points_[0].z + b1 * points_[1].z + b2 * points_[2].z + b3 * points_[3].z,
    };
  }

private:
  std::array<Vec3, 4> points_{};
};

}  // namespace robot_dog_laser_designator

#endif  // ROBOT_DOG_LASER_DESIGNATOR__BEZIER_CURVE_HPP_
