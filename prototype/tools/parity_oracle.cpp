// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// Emits the SHIPPING C++ cores' decisions for a deterministic grid of inputs,
// as CSV on stdout. check_parity.py replays the identical grid through the
// Python ports in prototype/drishti_proto and demands the same answers.
//
// This is what makes the prototype trustworthy. Without it, the demo could
// drift away from the system it claims to demonstrate and nobody would notice
// until someone asked why the real robot behaved differently.
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 \
//       -Iugv_ws/src/drishti_safety/include \
//       -Iugv_ws/src/drishti_traversability/include \
//       ugv_ws/src/drishti_safety/src/supervisor_core.cpp \
//       ugv_ws/src/drishti_traversability/src/traversability_core.cpp \
//       prototype/tools/parity_oracle.cpp -o parity_oracle

#include "drishti_safety/supervisor_core.hpp"
#include "drishti_traversability/traversability_core.hpp"

#include <cstdio>
#include <vector>

namespace ds = drishti_safety;
namespace dt = drishti_traversability;

namespace
{

// Deterministic, portable pseudo-random source. Not std::mt19937: the point is
// that Python must reproduce the SAME input grid, and re-implementing this
// three-line generator is trivial where reproducing a Mersenne Twister stream
// across languages is not.
struct Lcg
{
  unsigned long long s;
  explicit Lcg(unsigned long long seed)
  : s(seed) {}

  unsigned int next()
  {
    s = s * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<unsigned int>(s >> 33);
  }

  double uniform(double lo, double hi)
  {
    return lo + (hi - lo) * (next() / 4294967296.0);
  }
};

/// Values that exercise the boundaries, plus the pathological ones.
double spice(Lcg & rng, double lo, double hi)
{
  const unsigned int pick = rng.next() % 20;
  if (pick == 0) {return ds::kNaN;}
  if (pick == 1) {return ds::kInf;}
  if (pick == 2) {return -ds::kInf;}
  return rng.uniform(lo, hi);
}

}  // namespace

int main()
{
  Lcg rng(20260907ULL);

  // ---------------------------------------------------------- supervisor
  const ds::Params sp;
  const ds::SupervisorCore supervisor(sp);

  std::printf("# supervisor\n");
  std::printf("now,last_rgb,last_depth,rgb_static,pose_valid,cov,"
              "obstacle,confidence,path_valid,cmd_lin,cmd_ang,"
              "action,reason,v_limit,lin,ang,stop\n");

  for (int i = 0; i < 4000; ++i) {
    ds::Inputs in;
    in.now = 100.0;
    // Ages spanning fresh, marginal and long-dead, plus "never arrived".
    in.last_rgb_stamp = (rng.next() % 10 == 0)
      ? ds::kNever : in.now - rng.uniform(-0.05, 1.2);
    in.last_depth_stamp = (rng.next() % 10 == 0)
      ? ds::kNever : in.now - rng.uniform(-0.05, 1.2);
    in.rgb_static_for = spice(rng, 0.0, 4.0);
    in.pose_valid = (rng.next() % 5) != 0;
    in.pose_covariance_max = spice(rng, 0.0, 1.0);
    in.nearest_obstacle = spice(rng, 0.0, 4.0);
    in.perception_confidence = spice(rng, 0.0, 1.0);
    in.path_valid = (rng.next() % 4) != 0;
    in.cmd_linear_x = spice(rng, -2.5, 2.5);
    in.cmd_angular_z = spice(rng, -2.0, 2.0);

    const ds::Decision d = supervisor.evaluate(in);
    std::printf(
      "%.17g,%.17g,%.17g,%.17g,%d,%.17g,%.17g,%.17g,%d,%.17g,%.17g,"
      "%d,%d,%.17g,%.17g,%.17g,%d\n",
      in.now, in.last_rgb_stamp, in.last_depth_stamp, in.rgb_static_for,
      in.pose_valid ? 1 : 0, in.pose_covariance_max, in.nearest_obstacle,
      in.perception_confidence, in.path_valid ? 1 : 0,
      in.cmd_linear_x, in.cmd_angular_z,
      static_cast<int>(d.action), static_cast<int>(d.reason),
      d.v_limit, d.linear_x, d.angular_z, d.stop ? 1 : 0);
  }

  // ------------------------------------------------------ traversability
  const dt::Weights tw;
  const dt::Limits tl;
  const dt::TraversabilityCore terrain(tw, tl);

  std::printf("# traversability\n");
  std::printf("observed,slope,roughness,height_var,step,semantic,"
              "semantic_lethal,visibility,confidence,"
              "cost,lethal,unknown,byte\n");

  for (int i = 0; i < 4000; ++i) {
    dt::Cell c;
    c.observed = (rng.next() % 6) != 0;
    c.slope = spice(rng, 0.0, 0.7);
    c.roughness = spice(rng, 0.0, 0.15);
    c.height_variance = spice(rng, 0.0, 0.02);
    c.step_height = spice(rng, 0.0, 0.35);
    c.semantic_cost = spice(rng, -0.2, 1.2);
    c.semantic_lethal = (rng.next() % 12) == 0;
    c.visibility = spice(rng, 0.0, 1.0);
    c.confidence = spice(rng, 0.0, 1.0);

    const dt::CellCost r = terrain.evaluate(c);
    std::printf(
      "%d,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%.17g,%.17g,"
      "%.17g,%d,%d,%d\n",
      c.observed ? 1 : 0, c.slope, c.roughness, c.height_variance,
      c.step_height, c.semantic_cost, c.semantic_lethal ? 1 : 0,
      c.visibility, c.confidence,
      r.cost, r.lethal ? 1 : 0, r.unknown ? 1 : 0,
      static_cast<int>(dt::TraversabilityCore::to_costmap(r)));
  }

  return 0;
}
