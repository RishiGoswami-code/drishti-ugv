// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// Tests for the traversability cost function.
//
// This is the component the whole idea rests on: it is what turns a camera
// into terrain judgement, and it is the only thing standing between the
// planner and a ditch. Every case below has an answer that can be worked out
// by hand from SPEC.md 6.1.
//
// The tests that matter most are the ones asserting a cell CANNOT be cheap:
// a missing observation, a NaN slope, a zero-confidence reading. Getting those
// wrong does not produce a visibly broken map -- it produces a map that looks
// fine and drives into a hole.
//
// No ROS, no gtest. Build with:
//   g++ -std=c++17 -Iinclude src/traversability_core.cpp
//   test/test_traversability_core.cpp -o t && ./t

#include "drishti_traversability/traversability_core.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

using drishti_traversability::Cell;
using drishti_traversability::CellCost;
using drishti_traversability::kLethal;
using drishti_traversability::kMaxNonLethal;
using drishti_traversability::kNaN;
using drishti_traversability::Limits;
using drishti_traversability::normalise;
using drishti_traversability::TraversabilityCore;
using drishti_traversability::Weights;

namespace
{

int g_checks = 0;
int g_failures = 0;
const char * g_case = "";

void begin(const char * name) {g_case = name;}

void check(bool cond, const char * what, int line)
{
  ++g_checks;
  if (!cond) {
    ++g_failures;
    std::cerr << "  FAIL  [" << g_case << "] line " << line << ": " << what << "\n";
  }
}

void close_to(double got, double want, double tol, const char * what, int line)
{
  ++g_checks;
  if (!(std::fabs(got - want) <= tol)) {
    ++g_failures;
    std::cerr << "  FAIL  [" << g_case << "] line " << line << ": " << what
              << " got " << got << " want " << want << " (tol " << tol << ")\n";
  }
}

#define CHECK(cond) check((cond), #cond, __LINE__)
#define CLOSE(a, b, tol) close_to((a), (b), (tol), #a, __LINE__)

/// A cell that is perfectly observed, perfectly flat and perfectly safe.
Cell ideal()
{
  Cell c;
  c.observed = true;
  c.slope = 0.0;
  c.roughness = 0.0;
  c.height_variance = 0.0;
  c.step_height = 0.0;
  c.semantic_cost = 0.0;
  c.semantic_lethal = false;
  c.visibility = 1.0;
  c.confidence = 1.0;
  return c;
}

}  // namespace

int main()
{
  const Weights w;                    // defaults, as shipped
  const Limits l;
  const TraversabilityCore core(w, l);

  // -------------------------------------------------------------- config
  begin("default weights and limits are valid");
  {
    std::string why;
    CHECK(w.valid(&why));
    CHECK(l.valid(&why));
  }

  begin("a zero uncertainty weight is rejected");
  {
    // SPEC.md 6.2 would become unenforceable: an unconfident observation would
    // cost exactly as much as a confident one.
    Weights bad = w;
    bad.uncertainty = 0.0;
    std::string why;
    CHECK(!bad.valid(&why));
    CHECK(!why.empty());
  }

  begin("all-zero weights are rejected");
  {
    Weights bad{0, 0, 0, 0, 0, 0};
    CHECK(!bad.valid(nullptr));
  }

  begin("negative and non-finite weights are rejected");
  {
    Weights bad = w; bad.slope = -1.0;
    CHECK(!bad.valid(nullptr));
    bad = w; bad.obstacle = kNaN;
    CHECK(!bad.valid(nullptr));
  }

  begin("a zero unknown_cost is rejected");
  {
    Limits bad = l;
    bad.unknown_cost = 0.0;
    std::string why;
    CHECK(!bad.valid(&why));           // unknown terrain would be free
  }

  begin("lethal thresholds must sit above saturation");
  {
    Limits bad = l; bad.slope_lethal = bad.slope_max;
    CHECK(!bad.valid(nullptr));
    bad = l; bad.step_lethal = bad.step_max * 0.5;
    CHECK(!bad.valid(nullptr));
  }

  // ------------------------------------------------------- the ideal cell
  begin("a perfect cell is free");
  {
    const CellCost c = core.evaluate(ideal());
    CLOSE(c.cost, 0.0, 1e-12);
    CHECK(!c.lethal);
    CHECK(!c.unknown);
    CHECK(TraversabilityCore::to_costmap(c) == 0);
  }

  // ------------------------------------------- unknown is never free (6.2)
  begin("an unobserved cell is expensive but not lethal");
  {
    const CellCost c = core.evaluate(Cell{});   // all defaults
    CLOSE(c.cost, l.unknown_cost, 1e-12);
    CHECK(c.unknown);
    CHECK(!c.lethal);                  // must stay passable; see the header
    CHECK(TraversabilityCore::to_costmap(c) < kLethal);
    CHECK(TraversabilityCore::to_costmap(c) > kMaxNonLethal / 2);
  }

  begin("a NaN in any geometry field makes the cell unknown, never flat");
  {
    // The ditch failure mode: a hole in the elevation map must not read as a
    // road. Each field is spoiled on its own.
    Cell c = ideal(); c.slope = kNaN;
    CellCost r = core.evaluate(c);
    CHECK(r.unknown);
    CLOSE(r.cost, l.unknown_cost, 1e-12);

    c = ideal(); c.roughness = kNaN;
    r = core.evaluate(c);
    CHECK(r.unknown);

    c = ideal(); c.height_variance = kNaN;
    r = core.evaluate(c);
    CHECK(r.unknown);

    c = ideal(); c.step_height = kNaN;
    r = core.evaluate(c);
    CHECK(r.unknown);
  }

  begin("an infinite reading is unknown, not saturated-but-known");
  {
    Cell c = ideal();
    c.roughness = std::numeric_limits<double>::infinity();
    const CellCost r = core.evaluate(c);
    CHECK(r.unknown);
    CLOSE(r.cost, l.unknown_cost, 1e-12);
  }

  begin("zero visibility or zero confidence costs the uncertainty share");
  {
    // Fully observed, perfectly flat, but nothing can be trusted about it.
    // Cost must be exactly the uncertainty weight's share of the total.
    Cell c = ideal(); c.visibility = 0.0;
    CellCost r = core.evaluate(c);
    CHECK(!r.unknown);                 // it WAS observed
    CLOSE(r.uncertainty_term, 1.0, 1e-12);
    CLOSE(r.cost, w.uncertainty / w.sum(), 1e-12);

    c = ideal(); c.confidence = 0.0;
    r = core.evaluate(c);
    CLOSE(r.cost, w.uncertainty / w.sum(), 1e-12);
  }

  begin("the worse of visibility and confidence governs");
  {
    Cell c = ideal();
    c.visibility = 0.2;
    c.confidence = 0.9;
    const CellCost r = core.evaluate(c);
    CLOSE(r.uncertainty_term, 0.8, 1e-12);
  }

  begin("a NaN visibility or confidence is treated as none at all");
  {
    Cell c = ideal(); c.confidence = kNaN;
    const CellCost r = core.evaluate(c);
    CLOSE(r.uncertainty_term, 1.0, 1e-12);
  }

  // ---------------------------------------------------------- saturation
  begin("lethal slope saturates and short-circuits");
  {
    Cell c = ideal();
    c.slope = l.slope_lethal;
    const CellCost r = core.evaluate(c);
    CHECK(r.lethal);
    CLOSE(r.cost, 1.0, 1e-12);
    CHECK(TraversabilityCore::to_costmap(r) == kLethal);
    CHECK(std::string(r.lethal_reason).find("slope") != std::string::npos);
  }

  begin("lethal step saturates");
  {
    Cell c = ideal();
    c.step_height = l.step_lethal;
    const CellCost r = core.evaluate(c);
    CHECK(r.lethal);
    CHECK(std::string(r.lethal_reason).find("step") != std::string::npos);
  }

  begin("a lethal semantic class saturates a geometrically perfect cell");
  {
    // Water lying flat is the case: geometry says road, semantics say no.
    Cell c = ideal();
    c.semantic_lethal = true;
    const CellCost r = core.evaluate(c);
    CHECK(r.lethal);
    CHECK(TraversabilityCore::to_costmap(r) == kLethal);
  }

  begin("lethal geometry beats a missing observation");
  {
    // A measured cliff edge in an otherwise unobserved cell is still a cliff.
    Cell c;                            // observed = false
    c.slope = l.slope_lethal + 0.1;
    const CellCost r = core.evaluate(c);
    CHECK(r.lethal);
    CHECK(!r.unknown);
  }

  begin("just below a lethal threshold is expensive, not lethal");
  {
    Cell c = ideal();
    c.slope = std::nextafter(l.slope_lethal, 0.0);
    const CellCost r = core.evaluate(c);
    CHECK(!r.lethal);
    CLOSE(r.slope_term, 1.0, 1e-9);    // past slope_max, so saturated at 1
    CHECK(TraversabilityCore::to_costmap(r) < kLethal);
  }

  // ----------------------------------------------------- individual terms
  begin("each term saturates at its own limit and no sooner");
  {
    Cell c = ideal(); c.slope = l.slope_max;
    CLOSE(core.evaluate(c).slope_term, 1.0, 1e-12);
    c = ideal(); c.slope = l.slope_max / 2.0;
    CLOSE(core.evaluate(c).slope_term, 0.5, 1e-12);

    c = ideal(); c.roughness = l.roughness_max / 4.0;
    CLOSE(core.evaluate(c).roughness_term, 0.25, 1e-12);

    c = ideal(); c.height_variance = l.height_variance_max * 3.0;
    CLOSE(core.evaluate(c).height_variance_term, 1.0, 1e-12);   // clamped

    c = ideal(); c.step_height = l.step_max / 2.0;
    CLOSE(core.evaluate(c).obstacle_term, 0.5, 1e-12);
  }

  begin("a semantic cost outside [0, 1] is clamped");
  {
    Cell c = ideal(); c.semantic_cost = 5.0;
    CLOSE(core.evaluate(c).semantic_term, 1.0, 1e-12);
    c = ideal(); c.semantic_cost = -3.0;
    CLOSE(core.evaluate(c).semantic_term, 0.0, 1e-12);
  }

  begin("the total is the weighted mean of the terms");
  {
    Cell c = ideal();
    c.slope = l.slope_max;              // term 1.0
    c.step_height = l.step_max / 2.0;   // term 0.5
    const CellCost r = core.evaluate(c);
    const double expect =
      (w.slope * 1.0 + w.obstacle * 0.5) / w.sum();
    CLOSE(r.cost, expect, 1e-12);
  }

  begin("scaling every weight changes nothing");
  {
    Weights doubled = w;
    doubled.slope *= 2; doubled.roughness *= 2; doubled.height_variance *= 2;
    doubled.obstacle *= 2; doubled.semantic *= 2; doubled.uncertainty *= 2;
    const TraversabilityCore other(doubled, l);

    Cell c = ideal();
    c.slope = 0.2; c.roughness = 0.03; c.step_height = 0.05; c.confidence = 0.7;
    CLOSE(other.evaluate(c).cost, core.evaluate(c).cost, 1e-12);
  }

  begin("raising one weight raises that term's influence");
  {
    Weights slope_heavy = w;
    slope_heavy.slope = w.slope * 10.0;
    const TraversabilityCore other(slope_heavy, l);

    Cell c = ideal();
    c.slope = l.slope_max;              // only this term is non-zero
    CHECK(other.evaluate(c).cost > core.evaluate(c).cost);
  }

  begin("cost is monotonic in every geometric term");
  {
    double previous = -1.0;
    for (int i = 0; i <= 20; ++i) {
      Cell c = ideal();
      c.slope = l.slope_max * i / 20.0;
      const double cost = core.evaluate(c).cost;
      CHECK(cost >= previous - 1e-12);
      previous = cost;
    }

    previous = -1.0;
    for (int i = 0; i <= 20; ++i) {
      Cell c = ideal();
      c.step_height = l.step_max * i / 20.0;
      const double cost = core.evaluate(c).cost;
      CHECK(cost >= previous - 1e-12);
      previous = cost;
    }
  }

  begin("a fully bad observed cell costs 1.0 without being lethal");
  {
    Cell c = ideal();
    c.slope = l.slope_max;              // below slope_lethal
    c.roughness = l.roughness_max;
    c.height_variance = l.height_variance_max;
    c.step_height = l.step_max;         // below step_lethal
    c.semantic_cost = 1.0;
    c.visibility = 0.0;
    const CellCost r = core.evaluate(c);
    CLOSE(r.cost, 1.0, 1e-12);
    CHECK(!r.lethal);
    CHECK(TraversabilityCore::to_costmap(r) == kMaxNonLethal);
  }

  // ------------------------------------------------------ costmap mapping
  begin("costmap mapping never collides with Nav2's reserved values");
  {
    for (int i = 0; i <= 1000; ++i) {
      CellCost c;
      c.lethal = false;
      c.cost = i / 1000.0;
      const std::uint8_t byte = TraversabilityCore::to_costmap(c);
      CHECK(byte <= kMaxNonLethal);     // never 253, 254 or 255 by accident
    }
    CellCost lethal_cell;
    lethal_cell.lethal = true;
    CHECK(TraversabilityCore::to_costmap(lethal_cell) == kLethal);
  }

  begin("costmap mapping is monotonic and hits both ends");
  {
    CellCost c;
    c.lethal = false;
    c.cost = 0.0;
    CHECK(TraversabilityCore::to_costmap(c) == 0);
    c.cost = 1.0;
    CHECK(TraversabilityCore::to_costmap(c) == kMaxNonLethal);

    int previous = -1;
    for (int i = 0; i <= 100; ++i) {
      c.cost = i / 100.0;
      const int byte = TraversabilityCore::to_costmap(c);
      CHECK(byte >= previous);
      previous = byte;
    }
  }

  begin("a NaN cost maps to maximum expense, not to free");
  {
    CellCost c;
    c.lethal = false;
    c.cost = kNaN;
    CHECK(TraversabilityCore::to_costmap(c) == kMaxNonLethal);
  }

  // ------------------------------------------------------------ normalise
  begin("normalise clamps, takes magnitude, and falls back on nonsense");
  {
    CLOSE(normalise(0.5, 1.0, 9.0), 0.5, 1e-12);
    CLOSE(normalise(2.0, 1.0, 9.0), 1.0, 1e-12);      // clamped
    CLOSE(normalise(-0.5, 1.0, 9.0), 0.5, 1e-12);     // magnitude
    CLOSE(normalise(kNaN, 1.0, 9.0), 9.0, 1e-12);     // fallback
    CLOSE(normalise(1.0, 0.0, 9.0), 9.0, 1e-12);      // zero limit
  }

  // --------------------------------------------------------- the scenario
  begin("T07: a ditch is refused while the flat ground beside it is not");
  {
    // EVALUATION.md T07, and the claim made in the write-up. Geometry alone must
    // settle this, with no semantic model in the loop (TASK.md Phase 3).
    Cell ditch = ideal();
    ditch.step_height = 0.40;           // a 40 cm drop, wheel radius is 10 cm
    const CellCost d = core.evaluate(ditch);
    CHECK(d.lethal);
    CHECK(TraversabilityCore::to_costmap(d) == kLethal);

    Cell beside = ideal();
    beside.roughness = 0.01;
    const CellCost b = core.evaluate(beside);
    CHECK(!b.lethal);
    CHECK(TraversabilityCore::to_costmap(b) < 40);   // cheap enough to prefer
  }

  begin("T09: water reads as flat geometry and must be caught semantically");
  {
    // Standing water is the case geometry cannot see: perfectly smooth,
    // perfectly level, and not drivable. This is why the semantic term exists.
    Cell water = ideal();               // geometry says ideal road
    const CellCost as_geometry = core.evaluate(water);
    CHECK(TraversabilityCore::to_costmap(as_geometry) == 0);

    water.semantic_lethal = true;
    CHECK(core.evaluate(water).lethal);
  }

  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("traversability core: OK\n");
  }
  return g_failures == 0 ? 0 : 1;
}
