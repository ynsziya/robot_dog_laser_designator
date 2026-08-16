#ifndef ROBOT_DOG_LASER_DESIGNATOR__GAIT_ENGINE_HPP_
#define ROBOT_DOG_LASER_DESIGNATOR__GAIT_ENGINE_HPP_

#include <array>

#include "robot_dog_laser_designator/robot_dog_model.hpp"

namespace robot_dog_laser_designator
{

enum class GaitPhaseType
{
  STANCE,  ///< foot on the ground, pushing the body forward
  SWING    ///< foot in the air, moving toward the next foothold
};

enum class GaitType
{
  TROT,   ///< diagonal pairs move together (FL+RR, FR+RL) - default, most stable
  WALK,   ///< one leg swings at a time (slowest, most statically stable)
  PACE,   ///< same-side pairs move together (FL+RL, FR+RR)
  BOUND   ///< front pair / rear pair move together
};

/// State of a single leg at a given instant within the gait cycle.
struct LegPhaseState
{
  GaitPhaseType type{GaitPhaseType::STANCE};
  /// Normalized progress within the CURRENT phase (swing or stance), [0, 1).
  /// 0 = just entered this phase, approaching 1 = about to switch phase.
  double progress{0.0};
};

/// Generates per-leg swing/stance phase state for a cyclic gait, driven by
/// a single global gait-cycle clock. This is intentionally NOT a full CPG
/// (central pattern generator) -- it's the simpler deterministic
/// phase-offset approach used by CHAMP and most teleop-driven quadруped
/// controllers, which is far easier to tune and debug than CPG/RL and is
/// perfectly natural-looking for trot/walk gaits. A CPG-based engine can be
/// swapped in later behind this same interface if you want to experiment
/// with it (e.g. for running gaits).
class GaitEngine
{
public:
  /// step_frequency_hz: full gait cycles per second (higher = faster legs).
  /// duty_factor: fraction of each cycle spent in stance, (0, 1).
  ///   0.5 = classic trot (equal swing/stance).
  ///   >0.5 = more feet on the ground at once (more stable, slower looking).
  GaitEngine(GaitType type, double step_frequency_hz, double duty_factor);

  /// Advance the internal gait clock by dt seconds. Call this once per
  /// control loop iteration (e.g. at 100-200 Hz) BEFORE querying leg states.
  void update(double dt);

  /// Per-leg phase/state at the current gait clock time.
  LegPhaseState legPhaseState(LegId leg) const;

  /// Change step frequency on the fly (e.g. scaled by commanded speed).
  /// Does not reset the current phase, so it stays continuous (no foot pops).
  void setStepFrequency(double step_frequency_hz);
  double stepFrequency() const { return step_frequency_hz_; }

  void setDutyFactor(double duty_factor);
  double dutyFactor() const { return duty_factor_; }

  /// Resets the gait clock to 0 (all legs' phase offsets applied fresh).
  /// Use when starting to walk from standstill so the gait always begins
  /// from a clean, predictable configuration.
  void reset();

  /// True if the robot is currently commanded to stand still (frequency==0).
  /// When standing, all legs are forced to STANCE regardless of clock phase.
  bool isStanding() const { return step_frequency_hz_ <= 1e-6; }

private:
  double legPhaseOffset(LegId leg) const;

  GaitType type_;
  double step_frequency_hz_;
  double duty_factor_;
  double global_phase_{0.0};  ///< in [0, 1), one full gait cycle
};

}  // namespace robot_dog_laser_designator

#endif  // ROBOT_DOG_LASER_DESIGNATOR__GAIT_ENGINE_HPP_
