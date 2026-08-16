#include <cmath>
#include <cstdio>
#include <random>

#include "../include/robot_dog_laser_designator/leg_kinematics.hpp"

using namespace robot_dog_laser_designator;

int main()
{
  LegGeometry geo;  // defaults from URDF/STL analysis
  double max_err = 0.0;
  double max_err_x=0, max_err_y=0, max_err_z=0;
  int fails = 0, total = 0;

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> roll_dist(geo.hip_roll_limits.lower, geo.hip_roll_limits.upper);
  std::uniform_real_distribution<double> pitch_dist(-1.2, 1.2); // realistic walking range
  std::uniform_real_distribution<double> knee_dist(geo.knee_limits.lower, geo.knee_limits.upper);

  for (bool is_left : {true, false}) {
    LegKinematics kin(geo, is_left);
    for (int i = 0; i < 20000; ++i) {
      LegJointAngles q;
      q.hip_roll = roll_dist(rng);
      q.hip_pitch = pitch_dist(rng);
      q.knee = knee_dist(rng);

      Vec3 foot = kin.solveFk(q);

      LegJointAngles q_ik;
      bool ok = kin.solveIk(foot, q_ik);
      total++;
      if (!ok) { fails++; continue; }

      Vec3 foot_check = kin.solveFk(q_ik);
      double ex = std::fabs(foot_check.x - foot.x);
      double ey = std::fabs(foot_check.y - foot.y);
      double ez = std::fabs(foot_check.z - foot.z);
      double err = std::sqrt(ex*ex+ey*ey+ez*ez);
      if (err > max_err) { max_err = err; max_err_x=ex; max_err_y=ey; max_err_z=ez; }
    }
  }

  std::printf("Tested %d configs, %d marked unreachable (expected, joint-limit edge cases)\n", total, fails);
  std::printf("Max Cartesian round-trip error: %.8f m (x=%.8f y=%.8f z=%.8f)\n",
              max_err, max_err_x, max_err_y, max_err_z);

  // Also print a concrete example: neutral standing pose foot target.
  LegKinematics kin_fl(geo, true);
  Vec3 stand_target{0.0, geo.hip_offset_y, -0.6}; // ~0.6m below hip, roughly straight down
  LegJointAngles q_stand;
  bool reach = kin_fl.solveIk(stand_target, q_stand);
  std::printf("\nExample: FL foot target (0, %.3f, -0.6) reachable=%d\n", geo.hip_offset_y, reach);
  std::printf("  hip_roll=%.4f hip_pitch=%.4f knee=%.4f (rad)\n",
              q_stand.hip_roll, q_stand.hip_pitch, q_stand.knee);
  Vec3 fk_back = kin_fl.solveFk(q_stand);
  std::printf("  FK check -> (%.4f, %.4f, %.4f)\n", fk_back.x, fk_back.y, fk_back.z);

  return (max_err > 1e-6) ? 1 : 0;
}
