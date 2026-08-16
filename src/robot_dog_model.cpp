#include "robot_dog_laser_designator/robot_dog_model.hpp"

namespace robot_dog_laser_designator
{

std::string toString(LegId leg)
{
  switch (leg) {
    case LegId::FL: return "fl";
    case LegId::FR: return "fr";
    case LegId::RL: return "rl";
    case LegId::RR: return "rr";
  }
  return "unknown";
}

bool isLeftLeg(LegId leg)
{
  return leg == LegId::FL || leg == LegId::RL;
}

bool isFrontLeg(LegId leg)
{
  return leg == LegId::FL || leg == LegId::FR;
}

RobotDogModel::RobotDogModel()
{
  // hip_roll_joint origins relative to base_link, read directly from
  // spot_zero.urdf. These define where each leg attaches to the body.
  mounts_[static_cast<std::size_t>(LegId::FL)] =
    LegMount{LegId::FL, Vec3{0.335676, 0.085451, 0.086617}, true, true};
  mounts_[static_cast<std::size_t>(LegId::FR)] =
    LegMount{LegId::FR, Vec3{0.335676, -0.014549, 0.086617}, false, true};
  mounts_[static_cast<std::size_t>(LegId::RL)] =
    LegMount{LegId::RL, Vec3{-0.224324, 0.085451, 0.086617}, true, false};
  mounts_[static_cast<std::size_t>(LegId::RR)] =
    LegMount{LegId::RR, Vec3{-0.224324, -0.014549, 0.086617}, false, false};

  // NOTE: FL/RL y=0.085451 vs FR/RR y=-0.014549 look asymmetric at first
  // glance -- that's because base_link's own origin (in spot_zero.urdf) is
  // itself offset from the robot's true centerline (see <link name="base_link">
  // <inertial origin y=0.035451>). The left/right symmetry is preserved
  // around that offset centerline, not around base_link y=0. We keep the
  // raw URDF values here rather than "fixing" them, so foot targets computed
  // in base_link frame land correctly without needing a separate correction.
}

std::string RobotDogModel::jointNameHipRoll(LegId leg) const
{
  return toString(leg) + "_hip_roll_joint";
}

std::string RobotDogModel::jointNameHipPitch(LegId leg) const
{
  // Physically a pitch joint (rotates about Y), but named "hip_yaw" in the
  // URDF / joint_names.yaml / spot_controllers.yaml. We must match that
  // exact name when publishing commands.
  return toString(leg) + "_hip_yaw_joint";
}

std::string RobotDogModel::jointNameKnee(LegId leg) const
{
  return toString(leg) + "_knee_joint";
}

std::array<std::string, 12> RobotDogModel::orderedJointNames() const
{
  std::array<std::string, 12> names;
  std::size_t idx = 0;
  for (LegId leg : kAllLegs) {
    names[idx++] = jointNameHipRoll(leg);
    names[idx++] = jointNameHipPitch(leg);
    names[idx++] = jointNameKnee(leg);
  }
  return names;
}

}  // namespace robot_dog_laser_designator
