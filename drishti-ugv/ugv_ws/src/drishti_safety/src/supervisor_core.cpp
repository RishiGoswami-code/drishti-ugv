// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// SPEC.md section 9. See supervisor_core.hpp for the evaluation order and the
// one documented divergence from the section 9.1 pseudocode.

#include "drishti_safety/supervisor_core.hpp"

#include <cmath>
#include <string>

namespace drishti_safety
{

const char * to_string(Action a) noexcept
{
  switch (a) {
    case Action::PASS: return "PASS";
    case Action::SLOW: return "SLOW";
    case Action::STOP: return "STOP";
  }
  return "UNKNOWN";
}

const char * to_string(Reason r) noexcept
{
  switch (r) {
    case Reason::NONE: return "none";
    case Reason::LOCALIZATION_LOST: return "localisation lost";
    case Reason::DEPTH_STALE: return "depth stale";
    case Reason::CAMERA_STALE: return "camera stale";
    case Reason::OBSTACLE_EMERGENCY: return "obstacle inside emergency distance";
    case Reason::LOW_CONFIDENCE: return "perception confidence below floor";
    case Reason::PATH_INVALID: return "no valid path";
    case Reason::COMMAND_INVALID: return "planner command invalid";
    case Reason::CAMERA_FROZEN: return "camera frozen; frames unchanging";
  }
  return "unknown";
}

bool Params::valid(std::string * why) const
{
  auto fail = [why](const char * msg) {
      if (why != nullptr) {*why = msg;}
      return false;
    };

  if (!(std::isfinite(t_camera_stale) && t_camera_stale > 0.0)) {
    return fail("t_camera_stale must be finite and > 0");
  }
  if (!(std::isfinite(t_depth_stale) && t_depth_stale > 0.0)) {
    return fail("t_depth_stale must be finite and > 0");
  }
  if (!(std::isfinite(d_emergency) && d_emergency > 0.0)) {
    return fail("d_emergency must be finite and > 0");
  }
  if (!(std::isfinite(c_critical) && c_critical >= 0.0 && c_critical <= 1.0)) {
    return fail("c_critical must be within [0, 1]");
  }
  if (!(std::isfinite(v_max) && v_max > 0.0)) {
    return fail("v_max must be finite and > 0");
  }
  if (!(std::isfinite(v_slow) && v_slow > 0.0)) {
    return fail("v_slow must be finite and > 0");
  }
  if (v_slow > v_max) {
    // Otherwise "slow down" could raise the ceiling, breaking invariant 9.4.3.
    return fail("v_slow must not exceed v_max");
  }
  if (!(std::isfinite(cov_max) && cov_max > 0.0)) {
    return fail("cov_max must be finite and > 0");
  }
  if (!(std::isfinite(watchdog_period) && watchdog_period > 0.0)) {
    return fail("watchdog_period must be finite and > 0");
  }
  if (!(std::isfinite(t_frame_static) && t_frame_static > 0.0)) {
    return fail("t_frame_static must be finite and > 0");
  }
  if (t_frame_static <= t_camera_stale) {
    // Below the staleness window it would fire on ordinary frame timing rather
    // than on a genuinely frozen stream.
    return fail("t_frame_static must exceed t_camera_stale");
  }
  return true;
}

double stamp_age(double now, double stamp, double future_tolerance) noexcept
{
  if (!std::isfinite(now) || !std::isfinite(stamp)) {return kInf;}
  if (stamp <= kNever / 2.0) {return kInf;}          // nothing ever arrived

  const double age = now - stamp;
  if (age < -std::fabs(future_tolerance)) {
    return kInf;                                     // clocks disagree
  }
  return age < 0.0 ? 0.0 : age;                      // small jitter is fine
}

Decision SupervisorCore::evaluate(const Inputs & in) const
{
  // Fail-safe default: everything below either returns this or relaxes it.
  Decision d;
  d.action = Action::STOP;
  d.reason = Reason::NONE;
  d.v_limit = 0.0;
  d.linear_x = 0.0;
  d.angular_z = 0.0;
  d.stop = true;

  d.rgb_age = stamp_age(in.now, in.last_rgb_stamp, params_.watchdog_period);
  d.depth_age = stamp_age(in.now, in.last_depth_stamp, params_.watchdog_period);
  d.rgb_static_for = std::isfinite(in.rgb_static_for) ? in.rgb_static_for : 0.0;

  // --- 1. localisation ----------------------------------------------------
  if (!in.pose_valid ||
    !std::isfinite(in.pose_covariance_max) ||
    in.pose_covariance_max > params_.cov_max)
  {
    d.reason = Reason::LOCALIZATION_LOST;
    return d;
  }

  // --- 2. depth freshness -------------------------------------------------
  if (d.depth_age > params_.t_depth_stale) {
    d.reason = Reason::DEPTH_STALE;
    return d;
  }

  // --- 3. camera freshness ------------------------------------------------
  if (d.rgb_age > params_.t_camera_stale) {
    d.reason = Reason::CAMERA_STALE;
    return d;
  }

  // --- 4. frozen camera ---------------------------------------------------
  // Liveness is not enough. A camera republishing one image with a fresh stamp
  // passes step 3 forever while the view of the world grows arbitrarily old
  // (D18). Only a positive, finite report counts: a missing signal leaves
  // rgb_static_for at 0 and cannot stop the vehicle on its own.
  if (std::isfinite(in.rgb_static_for) &&
    in.rgb_static_for >= params_.t_frame_static)
  {
    d.reason = Reason::CAMERA_FROZEN;
    return d;
  }

  // --- 5. emergency geometry ----------------------------------------------
  // A non-finite distance means "nothing found in range", not "sensor broken":
  // a broken sensor was already caught by step 2.
  if (std::isfinite(in.nearest_obstacle) && in.nearest_obstacle < params_.d_emergency) {
    d.reason = Reason::OBSTACLE_EMERGENCY;
    return d;
  }

  // --- 6. planner has somewhere to go -------------------------------------
  if (!in.path_valid) {
    d.reason = Reason::PATH_INVALID;
    return d;
  }

  // --- 7. the command itself ----------------------------------------------
  // Checked before any forwarding path: clamping NaN yields NaN.
  if (!std::isfinite(in.cmd_linear_x) || !std::isfinite(in.cmd_angular_z)) {
    d.reason = Reason::COMMAND_INVALID;
    return d;
  }

  // --- 8. confidence: the only non-stop branch ----------------------------
  const bool slow =
    !std::isfinite(in.perception_confidence) ||
    in.perception_confidence < params_.c_critical;

  const double limit = slow ? params_.v_slow : params_.v_max;

  double lin = in.cmd_linear_x;
  double ang = in.cmd_angular_z;

  // Invariant 9.4.3: never increase a commanded velocity. Only ever scale
  // down, and scale angular by the same factor so the commanded path
  // curvature is preserved rather than tightened.
  const double speed = std::fabs(lin);
  if (speed > limit) {
    const double k = limit / speed;
    lin *= k;
    ang *= k;
  }

  d.action = slow ? Action::SLOW : Action::PASS;
  d.reason = slow ? Reason::LOW_CONFIDENCE : Reason::NONE;
  d.v_limit = limit;
  d.linear_x = lin;
  d.angular_z = ang;
  d.stop = false;
  return d;
}

}  // namespace drishti_safety
