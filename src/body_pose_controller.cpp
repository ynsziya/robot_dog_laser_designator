#include "robot_dog_laser_designator/body_pose_controller.hpp"

#include <algorithm>

namespace robot_dog_laser_designator
{

BodyPoseController::BodyPoseController()
: params_(Params())
{
}

BodyPoseController::BodyPoseController(const Params & params)
: params_(params)
{
}

Vec3 BodyPoseController::computeTrim(
  const Vec3 & mount_origin_in_base, double measured_roll, double measured_pitch) const
{
  double z = params_.body_height_trim;
  z += params_.pitch_compensation_gain * mount_origin_in_base.x * measured_pitch;
  z += params_.roll_compensation_gain * mount_origin_in_base.y * measured_roll;
  z = std::max(-params_.max_trim_z, std::min(params_.max_trim_z, z));
  return Vec3{0.0, 0.0, z};
}

}  // namespace robot_dog_laser_designator
