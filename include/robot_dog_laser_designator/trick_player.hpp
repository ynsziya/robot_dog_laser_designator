#ifndef ROBOT_DOG_LASER_DESIGNATOR__TRICK_PLAYER_HPP_
#define ROBOT_DOG_LASER_DESIGNATOR__TRICK_PLAYER_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "robot_dog_laser_designator/robot_dog_model.hpp"

namespace robot_dog_laser_designator
{

/// One full-body joint-space snapshot (4 legs x 3 joints).
struct BodyJointPose
{
  std::array<LegJointAngles, 4> legs{};
};

/// Timed keyframe for a showpiece posture sequence. `duration_sec` is the
/// time spent blending from the previous keyframe into this one.
struct TrickKeyframe
{
  BodyJointPose pose;
  double duration_sec{0.4};
};

enum class TrickId
{
  None = 0,
  Wave,
  PlayBow,
  Beg,
  Shake,  ///< wet-dog body shake from stand, then return to stand
};

/// Plays short joint-space "trick" choreographies (wave, play-bow, beg,
/// shake) as linear blends between keyframes. Designed to plug into
/// RobotDogControllerNode the same way sit/stand does: while active, gait
/// is locked out and the controller blends walking angles toward
/// `targetPose()` by `blendWeight()`.
///
/// Sequences always finish on the nominal standing pose so the robot returns
/// cleanly to teleop without a separate "stand" command.
class TrickPlayer
{
public:
  TrickPlayer() = default;

  /// Begin a named trick. Cancels any trick already in progress.
  /// Returns false if `id` is None / unknown (no state change).
  bool start(TrickId id);

  /// Parse posture_cmd strings: "wave", "play_bow"/"bow", "beg", "shake".
  /// Returns TrickId::None on no match.
  static TrickId idFromString(const std::string & name);

  static const char * toString(TrickId id);

  void cancel();

  bool active() const { return active_; }

  TrickId currentTrick() const { return current_id_; }

  /// Advance the keyframe clock. Call once per control-loop tick.
  void update(double dt);

  /// 0 = fully on the gait/stand pipeline, 1 = fully on targetPose().
  /// Ramps 0->1 into the first keyframe and 1->0 out of the last
  /// (standing) keyframe so start/end are soft.
  double blendWeight() const { return blend_weight_; }

  /// Current interpolated full-body joint target.
  const BodyJointPose & targetPose() const { return current_pose_; }

  /// Built-in choreographies (also useful for tests / tuning).
  static std::vector<TrickKeyframe> buildWave();
  static std::vector<TrickKeyframe> buildPlayBow();
  static std::vector<TrickKeyframe> buildBeg();
  static std::vector<TrickKeyframe> buildShake();

private:
  void load(std::vector<TrickKeyframe> keyframes, TrickId id);
  void sampleCurrentPose();

  static BodyJointPose standingPose();
  static BodyJointPose poseFromLegs(
    const LegJointAngles & fl,
    const LegJointAngles & fr,
    const LegJointAngles & rl,
    const LegJointAngles & rr);
  static LegJointAngles lerp(const LegJointAngles & a, const LegJointAngles & b, double t);
  static BodyJointPose lerp(const BodyJointPose & a, const BodyJointPose & b, double t);

  bool active_{false};
  TrickId current_id_{TrickId::None};
  std::vector<TrickKeyframe> keyframes_;
  std::size_t segment_index_{0};   ///< blending toward keyframes_[segment_index_]
  double segment_elapsed_{0.0};
  double blend_weight_{0.0};
  BodyJointPose current_pose_{};
  BodyJointPose segment_start_pose_{};
};

}  // namespace robot_dog_laser_designator

#endif  // ROBOT_DOG_LASER_DESIGNATOR__TRICK_PLAYER_HPP_
