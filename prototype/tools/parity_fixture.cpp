// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// Emits a compact JSON fixture of C++ decisions for the browser to check
// itself against.
//
// The web console runs the cost function and supervisor in JavaScript, because
// it lets you edit the world and watch the system react. A second port is a
// second chance to drift, so the page carries this fixture and re-runs it on
// load: if its JavaScript ever disagrees with the shipping C++, the page says
// so in its own header instead of quietly demonstrating something else.
//
// Values are round-tripped through their printed decimal form BEFORE being
// evaluated, so the double JavaScript parses is bit-identical to the double
// C++ used. Without that the fixture would show phantom mismatches in the last
// ulp and the check would be worthless.
//
//   g++ -std=c++17 -O2 -I<safety/include> -I<traversability/include> \
//       supervisor_core.cpp traversability_core.cpp parity_fixture.cpp -o fx

#include "drishti_safety/supervisor_core.hpp"
#include "drishti_traversability/traversability_core.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

namespace ds = drishti_safety;
namespace dt = drishti_traversability;

namespace
{

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

/// Round-trip through the printed form so C++ and JS evaluate the same double.
double pin(double v)
{
  if (v != v) {return v;}                       // NaN survives as-is
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.9g", v);
  return std::strtod(buf, nullptr);
}

/// Inputs: printed at the same precision they were pinned to, so the double
/// JavaScript parses is the double C++ evaluated.
std::string num(double v)
{
  if (v != v) {return "\"nan\"";}
  if (v > 1e300) {return "\"inf\"";}
  if (v < -1e300) {return "\"-inf\"";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.9g", v);
  return buf;
}

/// Outputs: full precision. Printing these at %.9g truncated the expected
/// value while the browser computed the full one, and the check reported 106
/// phantom terrain mismatches that were entirely an artefact of this function.
/// The comparison is meant to be exact, so the fixture must carry exact values.
std::string out(double v)
{
  if (v != v) {return "\"nan\"";}
  if (v > 1e300) {return "\"inf\"";}
  if (v < -1e300) {return "\"-inf\"";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

double spice(Lcg & rng, double lo, double hi)
{
  const unsigned int pick = rng.next() % 16;
  if (pick == 0) {return ds::kNaN;}
  if (pick == 1) {return ds::kInf;}
  return pin(rng.uniform(lo, hi));
}

}  // namespace

int main(int argc, char ** argv)
{
  const int n = (argc > 1) ? std::atoi(argv[1]) : 300;
  Lcg rng(20260908ULL);

  const ds::Params sp;
  const ds::SupervisorCore supervisor(sp);
  const dt::TraversabilityCore terrain(dt::Weights{}, dt::Limits{});

  std::printf("{\"supervisor\":[");
  for (int i = 0; i < n; ++i) {
    ds::Inputs in;
    in.now = 100.0;
    in.last_rgb_stamp = (rng.next() % 8 == 0)
      ? ds::kNever : pin(in.now - rng.uniform(-0.05, 1.2));
    in.last_depth_stamp = (rng.next() % 8 == 0)
      ? ds::kNever : pin(in.now - rng.uniform(-0.05, 1.2));
    in.rgb_static_for = spice(rng, 0.0, 4.0);
    in.pose_valid = (rng.next() % 5) != 0;
    in.pose_covariance_max = spice(rng, 0.0, 1.0);
    in.nearest_obstacle = spice(rng, 0.0, 4.0);
    in.perception_confidence = spice(rng, 0.0, 1.0);
    in.path_valid = (rng.next() % 4) != 0;
    in.cmd_linear_x = spice(rng, -2.5, 2.5);
    in.cmd_angular_z = spice(rng, -2.0, 2.0);

    const ds::Decision d = supervisor.evaluate(in);
    std::printf("%s[%s,%s,%s,%d,%s,%s,%s,%d,%s,%s,%d,%d,%s,%s,%s]",
      i ? "," : "",
      num(in.last_rgb_stamp).c_str(), num(in.last_depth_stamp).c_str(),
      num(in.rgb_static_for).c_str(), in.pose_valid ? 1 : 0,
      num(in.pose_covariance_max).c_str(), num(in.nearest_obstacle).c_str(),
      num(in.perception_confidence).c_str(), in.path_valid ? 1 : 0,
      num(in.cmd_linear_x).c_str(), num(in.cmd_angular_z).c_str(),
      static_cast<int>(d.action), static_cast<int>(d.reason),
      out(d.v_limit).c_str(), out(d.linear_x).c_str(), out(d.angular_z).c_str());
  }

  std::printf("],\"terrain\":[");
  for (int i = 0; i < n; ++i) {
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
    std::printf("%s[%d,%s,%s,%s,%s,%s,%d,%s,%s,%s,%d,%d,%d]",
      i ? "," : "",
      c.observed ? 1 : 0, num(c.slope).c_str(), num(c.roughness).c_str(),
      num(c.height_variance).c_str(), num(c.step_height).c_str(),
      num(c.semantic_cost).c_str(), c.semantic_lethal ? 1 : 0,
      num(c.visibility).c_str(), num(c.confidence).c_str(),
      out(r.cost).c_str(), r.lethal ? 1 : 0, r.unknown ? 1 : 0,
      static_cast<int>(dt::TraversabilityCore::to_costmap(r)));
  }
  std::printf("]}\n");
  return 0;
}
