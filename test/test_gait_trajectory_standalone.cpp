#include <cmath>
#include <cstdio>

#include "../include/robot_dog_laser_designator/gait_engine.hpp"
#include "../include/robot_dog_laser_designator/trajectory_generator.hpp"

using namespace robot_dog_laser_designator;

int main()
{
  // --- Test 1: trot phase offsets are correct (diagonal pairs in sync) ---
  {
    GaitEngine gait(GaitType::TROT, /*freq=*/1.0, /*duty=*/0.5);
    int mismatches = 0;
    for (int i = 0; i < 100; ++i) {
      gait.update(0.01);
      auto fl = gait.legPhaseState(LegId::FL);
      auto rr = gait.legPhaseState(LegId::RR);
      auto fr = gait.legPhaseState(LegId::FR);
      auto rl = gait.legPhaseState(LegId::RL);
      if (fl.type != rr.type || std::fabs(fl.progress - rr.progress) > 1e-9) mismatches++;
      if (fr.type != rl.type || std::fabs(fr.progress - rl.progress) > 1e-9) mismatches++;
      if (fl.type == fr.type) mismatches++;  // diagonal pairs must be OPPOSITE phase
    }
    std::printf("[Test 1] trot diagonal-pair sync mismatches: %d (expect 0)\n", mismatches);
  }

  // --- Test 2: swing/stance offset continuity across the full cycle -----
  {
    GaitEngine gait(GaitType::TROT, /*freq=*/2.0, /*duty=*/0.5);
    TrajectoryGenerator traj;
    const double stride_x = 0.15, stride_y = 0.0;
    const double dt = 0.002;
    Vec3 prev{};
    bool have_prev = false;
    double max_jump = 0.0;
    double min_z = 1e9, max_z = -1e9;

    for (int i = 0; i < 2000; ++i) {
      gait.update(dt);
      auto state = gait.legPhaseState(LegId::FL);
      Vec3 offset = traj.computeFootOffset(state, stride_x, stride_y);
      min_z = std::min(min_z, offset.z);
      max_z = std::max(max_z, offset.z);
      if (have_prev) {
        double dx = offset.x - prev.x, dy = offset.y - prev.y, dz = offset.z - prev.z;
        double jump = std::sqrt(dx*dx+dy*dy+dz*dz);
        max_jump = std::max(max_jump, jump);
      }
      prev = offset;
      have_prev = true;
    }
    std::printf("[Test 2] max per-step foot offset jump: %.6f m (dt=%.3fs, freq=2Hz -> should be small/smooth)\n", max_jump, dt);
    std::printf("[Test 2] z range during cycle: [%.4f, %.4f] (min should be ~0 during stance, max ~step_height=0.06)\n", min_z, max_z);
  }

  // --- Test 3: standing still (freq=0) keeps foot at neutral, no drift ---
  {
    GaitEngine gait(GaitType::TROT, /*freq=*/0.0, /*duty=*/0.5);
    TrajectoryGenerator traj;
    for (int i = 0; i < 500; ++i) {
      gait.update(0.01);
    }
    // NOTE: stride is derived from cmd_vel upstream (in the controller node),
    // so standing still means the CALLER passes stride=0, not that
    // GaitEngine/TrajectoryGenerator infer it. This test reflects that.
    auto state = gait.legPhaseState(LegId::FL);
    Vec3 offset = traj.computeFootOffset(state, /*stride_x=*/0.0, /*stride_y=*/0.0);
    std::printf("[Test 3] standing-still offset: (%.4f, %.4f, %.4f) (expect all 0)\n",
                offset.x, offset.y, offset.z);
  }

  return 0;
}
