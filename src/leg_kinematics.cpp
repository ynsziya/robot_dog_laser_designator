#include "robot_dog_laser_designator/leg_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace robot_dog_laser_designator
{

namespace
{
inline double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}
}  // namespace

LegKinematics::LegKinematics(const LegGeometry & geometry, bool is_left_leg)
: geometry_(geometry), is_left_leg_(is_left_leg)
{
  // Signed abduction offset: positive for left legs (hip_pitch axis is at
  // +Y relative to hip_roll axis), negative for right legs (-Y).
  L1_ = is_left_leg_ ? geometry_.hip_offset_y : -geometry_.hip_offset_y;
}

bool LegKinematics::solveIk(const Vec3 & target, LegJointAngles & out) const
{
  const double L1 = L1_;
  const double L2 = geometry_.thigh_length;
  const double L3 = geometry_.shin_length;

  bool reachable = true;

  // --- Step 1: hip_roll (abduction) -----------------------------------
  // Distance from the hip_roll axis to the target, in the Y-Z plane.
  const double Dyz_sq = target.y * target.y + target.z * target.z;
  double Dyz = std::sqrt(Dyz_sq);

  // Guard against target being closer than |L1| (would make the sqrt below
  // imaginary) -- clamp Dyz up to |L1| in that degenerate case.
  if (Dyz < std::fabs(L1)) {
    Dyz = std::fabs(L1);
    reachable = false;
  }

  // pz: the planar (post-abduction) leg's z-reach, in the tilted leg plane.
  // Negative root chosen: the leg extends "outward/downward" away from the
  // body, which is the only physically valid configuration for this robot
  // (hip_roll limits are small, -0.239..0.5 rad, so the leg never folds
  // back across the body).
  const double pz = -std::sqrt(std::max(0.0, Dyz_sq - L1 * L1));

  const double hip_roll = std::atan2(target.z, target.y) - std::atan2(pz, L1);

  // --- Step 2 & 3: planar 2-link IK (hip_pitch, knee) ------------------
  const double px = target.x - geometry_.hip_offset_x;
  const double Lr_sq = px * px + pz * pz;
  double Lr = std::sqrt(Lr_sq);

  const double max_reach = L2 + L3;
  const double min_reach = std::fabs(L2 - L3);
  if (Lr > max_reach) {
    reachable = false;
    Lr = max_reach;
  } else if (Lr < min_reach) {
    reachable = false;
    Lr = min_reach;
  }
  const double Lr_sq_clamped = Lr * Lr;

  // Law of cosines for the knee. Negative branch chosen to match this
  // robot's URDF convention (knee joint limits are entirely negative,
  // 0 = straight leg which is outside the mechanical range).
  const double cos_knee = clamp(
    (Lr_sq_clamped - L2 * L2 - L3 * L3) / (2.0 * L2 * L3), -1.0, 1.0);
  const double knee = -std::acos(cos_knee);

  const double A = L2 + L3 * std::cos(knee);
  const double B = L3 * std::sin(knee);
  const double denom = A * A + B * B;  // == Lr_sq_clamped, guarded > 0 by min_reach>0 in practice
  double hip_pitch = 0.0;
  if (denom > 1e-12) {
    const double sin_t2 = (B * pz - A * px) / denom;
    const double cos_t2 = -(A * pz + B * px) / denom;
    hip_pitch = std::atan2(sin_t2, cos_t2);
  }

  out.hip_roll = hip_roll;
  out.hip_pitch = hip_pitch;
  out.knee = knee;
  return reachable;
}

Vec3 LegKinematics::solveFk(const LegJointAngles & angles) const
{
  const double L1 = L1_;
  const double L2 = geometry_.thigh_length;
  const double L3 = geometry_.shin_length;
  const double t1 = angles.hip_roll;
  const double t2 = angles.hip_pitch;
  const double t3 = angles.knee;

  // Foot position relative to hip_pitch joint, in the leg's own (pre-tilt)
  // X-Z plane. direction(theta) = (-sin(theta), -cos(theta)) i.e. theta=0
  // points straight down (-Z), matching this URDF's convention.
  const double px = -L2 * std::sin(t2) - L3 * std::sin(t2 + t3);
  const double pz = -L2 * std::cos(t2) - L3 * std::cos(t2 + t3);

  Vec3 foot;
  foot.x = geometry_.hip_offset_x + px;
  foot.y = L1 * std::cos(t1) - pz * std::sin(t1);
  foot.z = L1 * std::sin(t1) + pz * std::cos(t1);
  return foot;
}

}  // namespace robot_dog_laser_designator
