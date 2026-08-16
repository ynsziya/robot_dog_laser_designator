#include "robot_dog_laser_designator/trick_player.hpp"

#include <algorithm>
#include <cctype>

namespace robot_dog_laser_designator
{

namespace
{
constexpr double kEps = 1e-9;

// Joint limits (LegGeometry): hip_roll [-0.239, 0.500], knee [-2.631, -0.662].
// Keep showpiece angles comfortably inside those bounds.
const LegJointAngles kStand{0.0, 0.764, -1.646};
}  // namespace

BodyJointPose TrickPlayer::standingPose()
{
  return poseFromLegs(kStand, kStand, kStand, kStand);
}

BodyJointPose TrickPlayer::poseFromLegs(
  const LegJointAngles & fl,
  const LegJointAngles & fr,
  const LegJointAngles & rl,
  const LegJointAngles & rr)
{
  BodyJointPose pose;
  pose.legs[static_cast<std::size_t>(LegId::FL)] = fl;
  pose.legs[static_cast<std::size_t>(LegId::FR)] = fr;
  pose.legs[static_cast<std::size_t>(LegId::RL)] = rl;
  pose.legs[static_cast<std::size_t>(LegId::RR)] = rr;
  return pose;
}

LegJointAngles TrickPlayer::lerp(const LegJointAngles & a, const LegJointAngles & b, double t)
{
  const double u = std::clamp(t, 0.0, 1.0);
  return LegJointAngles{
    a.hip_roll + u * (b.hip_roll - a.hip_roll),
    a.hip_pitch + u * (b.hip_pitch - a.hip_pitch),
    a.knee + u * (b.knee - a.knee),
  };
}

BodyJointPose TrickPlayer::lerp(const BodyJointPose & a, const BodyJointPose & b, double t)
{
  BodyJointPose out;
  for (std::size_t i = 0; i < 4; ++i) {
    out.legs[i] = lerp(a.legs[i], b.legs[i], t);
  }
  return out;
}

TrickId TrickPlayer::idFromString(const std::string & name)
{
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  if (lower == "wave") {return TrickId::Wave;}
  if (lower == "play_bow" || lower == "playbow" || lower == "bow") {
    return TrickId::PlayBow;
  }
  if (lower == "beg") {return TrickId::Beg;}
  if (lower == "shake" || lower == "body_shake") {return TrickId::Shake;}
  return TrickId::None;
}

const char * TrickPlayer::toString(TrickId id)
{
  switch (id) {
    case TrickId::Wave: return "wave";
    case TrickId::PlayBow: return "play_bow";
    case TrickId::Beg: return "beg";
    case TrickId::Shake: return "shake";
    case TrickId::None: return "none";
  }
  return "none";
}

bool TrickPlayer::start(TrickId id)
{
  switch (id) {
    case TrickId::Wave: load(buildWave(), id); return true;
    case TrickId::PlayBow: load(buildPlayBow(), id); return true;
    case TrickId::Beg: load(buildBeg(), id); return true;
    case TrickId::Shake: load(buildShake(), id); return true;
    case TrickId::None: return false;
  }
  return false;
}

void TrickPlayer::cancel()
{
  active_ = false;
  current_id_ = TrickId::None;
  keyframes_.clear();
  segment_index_ = 0;
  segment_elapsed_ = 0.0;
  blend_weight_ = 0.0;
  current_pose_ = standingPose();
}

void TrickPlayer::load(std::vector<TrickKeyframe> keyframes, TrickId id)
{
  if (keyframes.empty()) {
    cancel();
    return;
  }
  keyframes_ = std::move(keyframes);
  current_id_ = id;
  active_ = true;
  segment_index_ = 0;
  segment_elapsed_ = 0.0;
  blend_weight_ = 0.0;
  segment_start_pose_ = standingPose();
  current_pose_ = segment_start_pose_;
}

void TrickPlayer::sampleCurrentPose()
{
  if (!active_ || keyframes_.empty() || segment_index_ >= keyframes_.size()) {
    current_pose_ = standingPose();
    blend_weight_ = 0.0;
    return;
  }

  const TrickKeyframe & target = keyframes_[segment_index_];
  const double dur = std::max(target.duration_sec, kEps);
  const double t = std::clamp(segment_elapsed_ / dur, 0.0, 1.0);
  current_pose_ = lerp(segment_start_pose_, target.pose, t);

  // Soft entry on the first segment and soft exit on the last (standing)
  // segment so gait <-> trick handoff never snaps.
  const bool first = (segment_index_ == 0);
  const bool last = (segment_index_ + 1 == keyframes_.size());
  if (first && last) {
    // Single-keyframe trick: triangle 0 -> 1 -> 0 over the segment.
    blend_weight_ = (t < 0.5) ? (2.0 * t) : (2.0 * (1.0 - t));
  } else if (first) {
    blend_weight_ = t;
  } else if (last) {
    blend_weight_ = 1.0 - t;
  } else {
    blend_weight_ = 1.0;
  }
}

void TrickPlayer::update(double dt)
{
  if (!active_) {
    return;
  }
  if (dt < 0.0) {
    dt = 0.0;
  }

  segment_elapsed_ += dt;

  while (active_ && segment_index_ < keyframes_.size()) {
    const double dur = std::max(keyframes_[segment_index_].duration_sec, kEps);
    if (segment_elapsed_ < dur) {
      break;
    }
    // Snap to the keyframe we just finished, then advance.
    segment_start_pose_ = keyframes_[segment_index_].pose;
    segment_elapsed_ -= dur;
    ++segment_index_;
    if (segment_index_ >= keyframes_.size()) {
      cancel();
      return;
    }
  }

  sampleCurrentPose();
}

std::vector<TrickKeyframe> TrickPlayer::buildWave()
{
  // Front-left paw raised FORWARD (hip_pitch below stand swings the thigh
  // ahead; the old 1.35 value was sit-like and tucked the paw backward),
  // then hip_roll wag. Other three legs stay planted.
  // stand hip_pitch = 0.764
  const LegJointAngles fl_up{0.12, 0.32, -2.20};
  const LegJointAngles fl_left{-0.20, 0.28, -2.20};
  const LegJointAngles fl_right{0.45, 0.38, -2.20};

  const auto planted = [&](const LegJointAngles & fl) {
      return poseFromLegs(fl, kStand, kStand, kStand);
    };

  return {
    {planted(fl_up), 0.45},
    {planted(fl_left), 0.22},
    {planted(fl_right), 0.22},
    {planted(fl_left), 0.22},
    {planted(fl_right), 0.22},
    {planted(fl_up), 0.20},
    {standingPose(), 0.45},
  };
}

std::vector<TrickKeyframe> TrickPlayer::buildPlayBow()
{
  // Play-bow silhouette without pitching over the front feet: only the
  // front folds; rear stays near stand so the CoM remains inside the
  // support polygon (deep front + extended rear was tipping the robot).
  const LegJointAngles front{0.0, 1.12, -2.10};
  const LegJointAngles rear{0.0, 0.78, -1.60};
  const BodyJointPose bow = poseFromLegs(front, front, rear, rear);

  return {
    {bow, 0.60},
    {bow, 0.70},
    {standingPose(), 0.60},
  };
}

std::vector<TrickKeyframe> TrickPlayer::buildBeg()
{
  // Sit on haunches, both front paws raised (circus "beg").
  const LegJointAngles front{0.18, 1.45, -2.05};
  const LegJointAngles rear{0.0, 1.35, -2.55};
  const BodyJointPose beg = poseFromLegs(front, front, rear, rear);

  return {
    {beg, 0.65},
    {beg, 0.90},
    {standingPose(), 0.65},
  };
}

std::vector<TrickKeyframe> TrickPlayer::buildShake()
{
  // Wet-dog body shake from the standing pose: opposite L/R hip_roll only,
  // hip_pitch/knee stay at stand so the feet stay planted and CoM stable.
  const LegJointAngles l_pos{0.22, 0.764, -1.646};
  const LegJointAngles l_neg{-0.16, 0.764, -1.646};
  const LegJointAngles r_pos{0.22, 0.764, -1.646};
  const LegJointAngles r_neg{-0.16, 0.764, -1.646};
  const BodyJointPose shake_a = poseFromLegs(l_pos, r_neg, l_pos, r_neg);
  const BodyJointPose shake_b = poseFromLegs(l_neg, r_pos, l_neg, r_pos);

  return {
    {shake_a, 0.18},
    {shake_b, 0.14},
    {shake_a, 0.14},
    {shake_b, 0.14},
    {shake_a, 0.14},
    {shake_b, 0.14},
    {shake_a, 0.14},
    {shake_b, 0.14},
    {standingPose(), 0.35},
  };
}

}  // namespace robot_dog_laser_designator
