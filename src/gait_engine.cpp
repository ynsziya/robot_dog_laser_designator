#include "robot_dog_laser_designator/gait_engine.hpp"

#include <cmath>

namespace robot_dog_laser_designator
{

GaitEngine::GaitEngine(GaitType type, double step_frequency_hz, double duty_factor)
: type_(type), step_frequency_hz_(step_frequency_hz), duty_factor_(duty_factor)
{
}

void GaitEngine::update(double dt)
{
  if (isStanding()) {
    return;  // clock frozen while standing so we resume smoothly from wherever we stopped
  }
  global_phase_ += step_frequency_hz_ * dt;
  global_phase_ = std::fmod(global_phase_, 1.0);
  if (global_phase_ < 0.0) {
    global_phase_ += 1.0;
  }
}

double GaitEngine::legPhaseOffset(LegId leg) const
{
  switch (type_) {
    case GaitType::TROT:
      // Diagonal pairs in phase: (FL, RR) and (FR, RL) offset by half a cycle.
      switch (leg) {
        case LegId::FL: return 0.0;
        case LegId::RR: return 0.0;
        case LegId::FR: return 0.5;
        case LegId::RL: return 0.5;
      }
      break;
    case GaitType::WALK:
      // Sequential single-leg swing, most statically stable: FL -> RL -> FR -> RR.
      switch (leg) {
        case LegId::FL: return 0.0;
        case LegId::RL: return 0.25;
        case LegId::FR: return 0.5;
        case LegId::RR: return 0.75;
      }
      break;
    case GaitType::PACE:
      // Same-side pairs together (less stable than trot, faster on hard flat ground).
      switch (leg) {
        case LegId::FL: return 0.0;
        case LegId::RL: return 0.0;
        case LegId::FR: return 0.5;
        case LegId::RR: return 0.5;
      }
      break;
    case GaitType::BOUND:
      // Front pair / rear pair together (galloping-style, higher speed / less stable).
      switch (leg) {
        case LegId::FL: return 0.0;
        case LegId::FR: return 0.0;
        case LegId::RL: return 0.5;
        case LegId::RR: return 0.5;
      }
      break;
  }
  return 0.0;
}

LegPhaseState GaitEngine::legPhaseState(LegId leg) const
{
  if (isStanding()) {
    return LegPhaseState{GaitPhaseType::STANCE, 0.0};
  }

  double local_phase = global_phase_ + legPhaseOffset(leg);
  local_phase = std::fmod(local_phase, 1.0);
  if (local_phase < 0.0) {
    local_phase += 1.0;
  }

  LegPhaseState state;
  if (local_phase < duty_factor_) {
    state.type = GaitPhaseType::STANCE;
    state.progress = local_phase / duty_factor_;
  } else {
    state.type = GaitPhaseType::SWING;
    state.progress = (local_phase - duty_factor_) / (1.0 - duty_factor_);
  }
  return state;
}

void GaitEngine::setStepFrequency(double step_frequency_hz)
{
  step_frequency_hz_ = step_frequency_hz;
}

void GaitEngine::setDutyFactor(double duty_factor)
{
  duty_factor_ = duty_factor;
}

void GaitEngine::reset()
{
  global_phase_ = 0.0;
}

}  // namespace robot_dog_laser_designator
