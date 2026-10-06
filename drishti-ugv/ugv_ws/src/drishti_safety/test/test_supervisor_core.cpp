// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// Unit tests for the safety supervisor decision core.
//
// No ROS, no gtest, no network: this is a plain executable so it can be run on
// any machine with a C++17 compiler, including one that cannot install ROS 2.
// SPEC.md invariant 9.4.5.
//
//   g++ -std=c++17 -Iinclude src/supervisor_core.cpp test/test_supervisor_core.cpp -o t && ./t

#include "drishti_safety/supervisor_core.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

using drishti_safety::Action;
using drishti_safety::Decision;
using drishti_safety::Inputs;
using drishti_safety::kInf;
using drishti_safety::kNaN;
using drishti_safety::Params;
using drishti_safety::Reason;
using drishti_safety::stamp_age;
using drishti_safety::SupervisorCore;
using drishti_safety::to_string;

namespace
{

int g_checks = 0;
int g_failures = 0;
const char * g_case = "";

void begin(const char * name)
{
  g_case = name;
}

void check(bool cond, const char * expr, int line)
{
  ++g_checks;
  if (!cond) {
    ++g_failures;
    std::cerr << "  FAIL  [" << g_case << "] line " << line << ": " << expr << "\n";
  }
}

void check_action(const Decision & d, Action want, Reason why, int line)
{
  ++g_checks;
  if (d.action != want || d.reason != why) {
    ++g_failures;
    std::cerr << "  FAIL  [" << g_case << "] line " << line
              << ": got " << to_string(d.action) << "/" << to_string(d.reason)
              << ", want " << to_string(want) << "/" << to_string(why) << "\n";
  }
}

#define CHECK(cond) check((cond), #cond, __LINE__)
#define CHECK_DECISION(d, a, r) check_action((d), (a), (r), __LINE__)

/// A tick where everything is healthy. Individual tests spoil one field.
Inputs healthy(double now = 100.0)
{
  Inputs in;
  in.now = now;
  in.last_rgb_stamp = now - 0.01;
  in.last_depth_stamp = now - 0.01;
  in.pose_valid = true;
  in.pose_covariance_max = 0.05;
  in.nearest_obstacle = 5.0;
  in.perception_confidence = 0.9;
  in.rgb_static_for = 0.0;
  in.path_valid = true;
  in.cmd_linear_x = 0.5;
  in.cmd_angular_z = 0.2;
  return in;
}

}  // namespace

int main()
{
  const Params p;                       // defaults, as shipped in drishti.yaml
  const SupervisorCore sup(p);

  // ---------------------------------------------------------------- params
  begin("params: defaults are valid");
  {
    std::string why;
    CHECK(p.valid(&why));
  }

  begin("params: v_slow above v_max is rejected");
  {
    Params bad = p;
    bad.v_slow = 2.0;                   // would let SLOW raise the ceiling
    std::string why;
    CHECK(!bad.valid(&why));
    CHECK(!why.empty());
  }

  begin("params: non-finite and out-of-range values are rejected");
  {
    Params bad = p; bad.t_camera_stale = 0.0;
    CHECK(!bad.valid(nullptr));
    bad = p; bad.d_emergency = kNaN;
    CHECK(!bad.valid(nullptr));
    bad = p; bad.c_critical = 1.5;
    CHECK(!bad.valid(nullptr));
    bad = p; bad.watchdog_period = -0.01;
    CHECK(!bad.valid(nullptr));
  }

  // ------------------------------------------------------------- stamp_age
  begin("stamp_age: never-received is infinite");
  CHECK(std::isinf(stamp_age(100.0, drishti_safety::kNever, 0.02)));

  begin("stamp_age: normal case");
  CHECK(std::fabs(stamp_age(100.0, 99.5, 0.02) - 0.5) < 1e-9);

  begin("stamp_age: small negative jitter clamps to zero");
  CHECK(stamp_age(100.0, 100.005, 0.02) == 0.0);

  begin("stamp_age: gross future stamp is infinite (clocks disagree)");
  CHECK(std::isinf(stamp_age(100.0, 105.0, 0.02)));

  begin("stamp_age: non-finite input is infinite");
  CHECK(std::isinf(stamp_age(kNaN, 99.0, 0.02)));
  CHECK(std::isinf(stamp_age(100.0, kNaN, 0.02)));

  // -------------------------------------------------- the fail-safe default
  begin("default-constructed Inputs stop the vehicle");
  {
    // Nothing has arrived and nothing is known. This is the state at startup.
    const Decision d = sup.evaluate(Inputs{});
    CHECK_DECISION(d, Action::STOP, Reason::LOCALIZATION_LOST);
    CHECK(d.stop);
    CHECK(d.linear_x == 0.0 && d.angular_z == 0.0);
  }

  // ------------------------------------------------------- the happy path
  begin("healthy tick passes the command through unchanged");
  {
    const Decision d = sup.evaluate(healthy());
    CHECK_DECISION(d, Action::PASS, Reason::NONE);
    CHECK(!d.stop);
    CHECK(std::fabs(d.linear_x - 0.5) < 1e-12);
    CHECK(std::fabs(d.angular_z - 0.2) < 1e-12);
    CHECK(std::fabs(d.v_limit - p.v_max) < 1e-12);
  }

  // ------------------------------------------------- each STOP condition
  begin("localisation: pose not valid");
  {
    Inputs in = healthy(); in.pose_valid = false;
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::LOCALIZATION_LOST);
  }

  begin("localisation: covariance above cov_max");
  {
    Inputs in = healthy(); in.pose_covariance_max = p.cov_max + 0.01;
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::LOCALIZATION_LOST);
  }

  begin("localisation: covariance exactly at cov_max is still allowed");
  {
    Inputs in = healthy(); in.pose_covariance_max = p.cov_max;
    CHECK_DECISION(sup.evaluate(in), Action::PASS, Reason::NONE);
  }

  begin("depth staleness");
  {
    Inputs in = healthy();
    in.last_depth_stamp = in.now - (p.t_depth_stale + 0.01);
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::DEPTH_STALE);
  }

  begin("camera staleness");
  {
    Inputs in = healthy();
    in.last_rgb_stamp = in.now - (p.t_camera_stale + 0.01);
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::CAMERA_STALE);
  }

  begin("depth is checked before camera when both are stale");
  {
    Inputs in = healthy();
    in.last_rgb_stamp = in.now - 10.0;
    in.last_depth_stamp = in.now - 10.0;
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::DEPTH_STALE);
  }

  begin("a frozen camera stops the vehicle (D18/D19)");
  {
    // The failure t_camera_stale cannot see: frames keep arriving with fresh
    // stamps, so rgb_age stays small while the view of the world is minutes
    // old. Everything else about this tick is perfectly healthy.
    Inputs in = healthy();
    in.rgb_static_for = p.t_frame_static;
    const Decision d = sup.evaluate(in);
    CHECK_DECISION(d, Action::STOP, Reason::CAMERA_FROZEN);
    CHECK(d.stop);
    CHECK(d.linear_x == 0.0);
    // The audit record must carry the number that drove the decision.
    CHECK(std::fabs(d.rgb_static_for - p.t_frame_static) < 1e-12);
  }

  begin("a briefly static scene does not stop the vehicle");
  {
    Inputs in = healthy();
    in.rgb_static_for = p.t_frame_static - 0.01;
    CHECK_DECISION(sup.evaluate(in), Action::PASS, Reason::NONE);
  }

  begin("a missing static-duration signal cannot stop the vehicle by itself");
  {
    // A perception node too old to publish rgb_static_for leaves it at 0.
    // That must not be read as a fault: silence is already covered by the
    // health message going stale, and treating absence as a freeze would make
    // an older perception build undriveable.
    Inputs in = healthy();
    in.rgb_static_for = 0.0;
    CHECK_DECISION(sup.evaluate(in), Action::PASS, Reason::NONE);

    in.rgb_static_for = kNaN;
    CHECK_DECISION(sup.evaluate(in), Action::PASS, Reason::NONE);
  }

  begin("silence is checked before freeze");
  {
    // A camera that has gone silent AND was frozen before it died reports as
    // stale, the simpler and more urgent diagnosis.
    Inputs in = healthy();
    in.last_rgb_stamp = in.now - 10.0;
    in.rgb_static_for = 60.0;
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::CAMERA_STALE);
  }

  begin("t_frame_static must exceed t_camera_stale");
  {
    Params bad = p;
    bad.t_frame_static = bad.t_camera_stale;
    std::string why;
    CHECK(!bad.valid(&why));
    bad = p; bad.t_frame_static = 0.0;
    CHECK(!bad.valid(nullptr));
    bad = p; bad.t_frame_static = kNaN;
    CHECK(!bad.valid(nullptr));
  }

  begin("obstacle inside the emergency distance");
  {
    Inputs in = healthy(); in.nearest_obstacle = p.d_emergency - 0.01;
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::OBSTACLE_EMERGENCY);
  }

  begin("obstacle exactly at the emergency distance does not stop");
  {
    Inputs in = healthy(); in.nearest_obstacle = p.d_emergency;
    CHECK_DECISION(sup.evaluate(in), Action::PASS, Reason::NONE);
  }

  begin("NaN obstacle distance means 'nothing in range', not a fault");
  {
    // Safe only because depth freshness is proven first; see supervisor_core.hpp.
    Inputs in = healthy(); in.nearest_obstacle = kNaN;
    CHECK_DECISION(sup.evaluate(in), Action::PASS, Reason::NONE);
  }

  begin("no valid path");
  {
    Inputs in = healthy(); in.path_valid = false;
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::PATH_INVALID);
  }

  begin("non-finite command");
  {
    Inputs in = healthy(); in.cmd_linear_x = kNaN;
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::COMMAND_INVALID);
    in = healthy(); in.cmd_angular_z = kInf;
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::COMMAND_INVALID);
  }

  begin("a stopped decision always publishes exactly zero");
  {
    Inputs in = healthy();
    in.cmd_linear_x = 3.0; in.cmd_angular_z = -2.0;
    in.path_valid = false;
    const Decision d = sup.evaluate(in);
    CHECK(d.stop);
    CHECK(d.linear_x == 0.0);
    CHECK(d.angular_z == 0.0);
    CHECK(d.v_limit == 0.0);
  }

  // ------------------------------------------------------- the SLOW branch
  begin("low confidence slows rather than stops");
  {
    Inputs in = healthy();
    in.perception_confidence = p.c_critical - 0.01;
    in.cmd_linear_x = 1.0;
    const Decision d = sup.evaluate(in);
    CHECK_DECISION(d, Action::SLOW, Reason::LOW_CONFIDENCE);
    CHECK(!d.stop);
    CHECK(std::fabs(d.v_limit - p.v_slow) < 1e-12);
    CHECK(std::fabs(d.linear_x - p.v_slow) < 1e-12);
  }

  begin("confidence exactly at the floor is full speed");
  {
    Inputs in = healthy(); in.perception_confidence = p.c_critical;
    CHECK_DECISION(sup.evaluate(in), Action::PASS, Reason::NONE);
  }

  begin("NaN confidence is treated as low, not as high");
  {
    Inputs in = healthy(); in.perception_confidence = kNaN;
    CHECK_DECISION(sup.evaluate(in), Action::SLOW, Reason::LOW_CONFIDENCE);
  }

  begin("a slow command below v_slow is not sped up");
  {
    Inputs in = healthy();
    in.perception_confidence = 0.1;
    in.cmd_linear_x = 0.1;
    const Decision d = sup.evaluate(in);
    CHECK_DECISION(d, Action::SLOW, Reason::LOW_CONFIDENCE);
    CHECK(std::fabs(d.linear_x - 0.1) < 1e-12);   // invariant 9.4.3
  }

  // ------------------------------------ divergence from the 9.1 pseudocode
  begin("low confidence AND no valid path stops (STOP beats SLOW)");
  {
    Inputs in = healthy();
    in.perception_confidence = 0.01;
    in.path_valid = false;
    CHECK_DECISION(sup.evaluate(in), Action::STOP, Reason::PATH_INVALID);
  }

  // ------------------------------------------------------------- clamping
  begin("over-limit command is clamped to v_max");
  {
    Inputs in = healthy(); in.cmd_linear_x = 5.0; in.cmd_angular_z = 1.0;
    const Decision d = sup.evaluate(in);
    CHECK_DECISION(d, Action::PASS, Reason::NONE);
    CHECK(std::fabs(d.linear_x - p.v_max) < 1e-12);
  }

  begin("clamping scales angular velocity to preserve path curvature");
  {
    Inputs in = healthy();
    in.cmd_linear_x = 2.4;              // exactly 2x v_max
    in.cmd_angular_z = 1.0;
    const Decision d = sup.evaluate(in);
    CHECK(std::fabs(d.linear_x - 1.2) < 1e-12);
    CHECK(std::fabs(d.angular_z - 0.5) < 1e-12);
    // curvature omega/v is unchanged
    const double before = 1.0 / 2.4;
    const double after = d.angular_z / d.linear_x;
    CHECK(std::fabs(before - after) < 1e-12);
  }

  begin("reverse commands are clamped by magnitude, keeping their sign");
  {
    Inputs in = healthy(); in.cmd_linear_x = -5.0; in.cmd_angular_z = -1.0;
    const Decision d = sup.evaluate(in);
    CHECK(std::fabs(d.linear_x + p.v_max) < 1e-12);
    CHECK(d.angular_z < 0.0);
  }

  begin("pure rotation with zero linear velocity is left alone");
  {
    Inputs in = healthy(); in.cmd_linear_x = 0.0; in.cmd_angular_z = 0.8;
    const Decision d = sup.evaluate(in);
    CHECK_DECISION(d, Action::PASS, Reason::NONE);
    CHECK(std::fabs(d.angular_z - 0.8) < 1e-12);
  }

  begin("the supervisor never increases speed (property sweep)");
  {
    for (int i = 0; i <= 60; ++i) {
      const double v = -3.0 + 0.1 * i;
      Inputs in = healthy();
      in.cmd_linear_x = v;
      const Decision d = sup.evaluate(in);
      CHECK(std::fabs(d.linear_x) <= std::fabs(v) + 1e-12);
      CHECK(std::fabs(d.linear_x) <= p.v_max + 1e-12);
    }
  }

  begin("determinism: the same inputs give the same decision");
  {
    const Inputs in = healthy();
    const Decision a = sup.evaluate(in);
    for (int i = 0; i < 100; ++i) {
      const Decision b = sup.evaluate(in);
      CHECK(a.action == b.action && a.reason == b.reason);
      CHECK(a.linear_x == b.linear_x && a.angular_z == b.angular_z);
    }
  }

  // ----------------------------------------------------------------- report
  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("supervisor core: OK\n");
  }
  return g_failures == 0 ? 0 : 1;
}
