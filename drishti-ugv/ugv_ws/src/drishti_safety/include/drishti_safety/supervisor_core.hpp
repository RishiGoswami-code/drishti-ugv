// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// Deterministic safety supervisor — decision core.
//
// SPEC.md section 9. This header deliberately depends on nothing but the C++
// standard library: no ROS, no messages, no clock, no I/O. That is what makes
// invariant 9.4.5 ("its behaviour is unit-testable without ROS running")
// achievable, and it is why the stop decision can be audited by reading one
// function.
//
// The ROS node is a thin wrapper: it collects inputs, calls evaluate(), and
// publishes. It contains no policy.

#ifndef DRISHTI_SAFETY__SUPERVISOR_CORE_HPP_
#define DRISHTI_SAFETY__SUPERVISOR_CORE_HPP_

#include <cstdint>
#include <limits>
#include <string>

namespace drishti_safety
{

/// Sentinel for "no message of this kind has ever arrived".
inline constexpr double kNever = -1.0e18;

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
inline constexpr double kInf = std::numeric_limits<double>::infinity();

/// What the supervisor did with the incoming command.
enum class Action : std::uint8_t
{
  PASS = 0,   ///< forwarded unchanged
  SLOW = 1,   ///< forwarded, clamped to v_slow
  STOP = 2,   ///< zero velocity published
};

/// Why. Values match drishti_msgs/msg/SafetyState constants exactly.
///
/// NOTE: these numbers follow the SPEC.md section 9.1 *listing* order. They are
/// not the evaluation order — see the comment on SupervisorCore::evaluate.
enum class Reason : std::uint8_t
{
  NONE = 0,
  LOCALIZATION_LOST = 1,
  DEPTH_STALE = 2,
  CAMERA_STALE = 3,
  OBSTACLE_EMERGENCY = 4,
  LOW_CONFIDENCE = 5,
  PATH_INVALID = 6,
  COMMAND_INVALID = 7,
  CAMERA_FROZEN = 8,
};

const char * to_string(Action a) noexcept;
const char * to_string(Reason r) noexcept;

/// Thresholds. Names and meanings are fixed by SPEC.md section 9.3; the values
/// here are the same initial defaults as drishti_bringup/config/drishti.yaml
/// and are not tuned.
struct Params
{
  double t_camera_stale{0.30};    ///< s, max age of an RGB frame
  double t_depth_stale{0.30};     ///< s, max age of a depth frame
  double d_emergency{0.80};       ///< m, lethal obstacle distance
  double c_critical{0.40};        ///< 0..1 confidence floor for full speed
  double v_max{1.20};             ///< m/s normal ceiling
  double v_slow{0.35};            ///< m/s reduced ceiling
  double cov_max{0.50};           ///< max pose covariance diagonal before "lost"
  double watchdog_period{0.02};   ///< s, supervisor tick

  /// Seconds of unchanging RGB content before the camera is treated as frozen.
  ///
  /// D18/D19. `t_camera_stale` catches a camera that goes silent; it cannot
  /// catch one that republishes the same image with a fresh stamp, because the
  /// age never grows. That failure looks perfectly healthy and is the more
  /// dangerous of the two.
  ///
  /// Deliberately far longer than t_camera_stale. A genuinely motionless
  /// vehicle looking at a static scene through a noiseless simulated camera
  /// produces identical frames, so a short threshold would fire constantly in
  /// simulation. If it does fire spuriously the result is a STOP, which is the
  /// safe direction to be wrong in.
  double t_frame_static{2.0};     ///< s

  /// Rejects a configuration that would silently weaken the safety envelope.
  /// Returns false and, if \p why is non-null, sets it to the first problem.
  bool valid(std::string * why = nullptr) const;
};

/// One tick's worth of evidence. The caller is responsible for filling this
/// honestly; anything it does not know should be left at its default, which is
/// always the unsafe-until-proven value.
struct Inputs
{
  double now{0.0};                       ///< s, current time on the chosen clock

  double last_rgb_stamp{kNever};         ///< s, sensor stamp of last RGB frame
  double last_depth_stamp{kNever};       ///< s, sensor stamp of last depth frame

  /// Seconds the RGB content has been unchanged, from /perception/health.
  /// Defaults to 0 rather than to a large value: absence of this signal must
  /// not by itself stop the vehicle, because a perception node too old to
  /// publish it would otherwise be undriveable. Silence is already covered by
  /// the health message going stale.
  double rgb_static_for{0.0};

  bool pose_valid{false};                ///< localisation produced a pose at all
  double pose_covariance_max{kInf};      ///< largest diagonal term

  double nearest_obstacle{kNaN};         ///< m; NaN = nothing found in range
  double perception_confidence{0.0};     ///< 0..1
  bool path_valid{false};                ///< planner reports a usable path

  double cmd_linear_x{0.0};              ///< m/s from /cmd_vel_nav
  double cmd_angular_z{0.0};             ///< rad/s from /cmd_vel_nav
};

/// The decision, and the evidence that produced it.
struct Decision
{
  Action action{Action::STOP};
  Reason reason{Reason::NONE};
  double v_limit{0.0};        ///< m/s ceiling applied
  double linear_x{0.0};       ///< m/s to publish on /cmd_vel
  double angular_z{0.0};      ///< rad/s to publish on /cmd_vel
  bool stop{true};

  double rgb_age{kInf};       ///< s, as computed this tick
  double depth_age{kInf};     ///< s, as computed this tick
  double rgb_static_for{0.0}; ///< s, as reported this tick
};

class SupervisorCore
{
public:
  explicit SupervisorCore(const Params & p) noexcept
  : params_(p) {}

  /// Evaluate one tick. Pure: same inputs always give the same decision.
  ///
  /// Evaluation order — every STOP condition is checked before the SLOW
  /// condition:
  ///
  ///   1. localisation lost         -> STOP
  ///   2. depth stale               -> STOP
  ///   3. camera stale              -> STOP
  ///   4. camera frozen             -> STOP
  ///   5. obstacle within d_emergency -> STOP
  ///   6. no valid path             -> STOP
  ///   7. command not finite        -> STOP
  ///   8. confidence < c_critical   -> SLOW
  ///   9. otherwise                 -> PASS
  ///
  /// This DIVERGES from the SPEC.md section 9.1 pseudocode, which places the
  /// low-confidence SLOW branch ahead of the path-validity check under a
  /// "first match wins" rule. Taken literally that would forward a command
  /// while the planner reports no usable path, merely because confidence was
  /// low — a weaker outcome than stopping. Sorting all STOP conditions first
  /// can only ever turn a SLOW into a STOP, never the reverse, so it cannot
  /// weaken the envelope. SPEC.md section 9.1 should be corrected to match.
  ///
  /// Ordering is load-bearing in one other place: a NaN nearest_obstacle is
  /// read as "nothing found in range", not as a fault. That is only safe
  /// because depth staleness (2) is already ruled out by the time step 5 runs.
  Decision evaluate(const Inputs & in) const;

  const Params & params() const noexcept {return params_;}

private:
  Params params_;
};

/// Age of a timestamped input, in seconds.
///
/// Returns +inf when nothing has arrived, when either value is not finite, or
/// when the stamp lies more than one watchdog period in the future — clocks
/// that disagree are treated as no evidence at all, per SPEC.md section 3.2
/// rule 4 and section 9.4.2.
double stamp_age(double now, double stamp, double future_tolerance) noexcept;

}  // namespace drishti_safety

#endif  // DRISHTI_SAFETY__SUPERVISOR_CORE_HPP_
