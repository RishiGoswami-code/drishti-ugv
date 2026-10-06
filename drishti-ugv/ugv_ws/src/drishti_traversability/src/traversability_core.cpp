// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// SPEC.md section 6.1 cost function. See the header for the decision order.

#include "drishti_traversability/traversability_core.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace drishti_traversability
{

bool Weights::valid(std::string * why) const
{
  auto fail = [why](const char * msg) {
      if (why != nullptr) {*why = msg;}
      return false;
    };

  const double values[] = {slope, roughness, height_variance,
    obstacle, semantic, uncertainty};
  for (double v : values) {
    if (!std::isfinite(v) || v < 0.0) {
      return fail("every weight must be finite and >= 0");
    }
  }
  if (sum() <= 0.0) {
    // Every cell would cost the same, so the layer would do nothing at all
    // while appearing to be configured.
    return fail("at least one weight must be > 0");
  }
  if (uncertainty <= 0.0) {
    // SPEC.md 6.2 is not optional: with no uncertainty weight, an observed
    // cell with zero confidence costs the same as a perfectly observed one.
    return fail("uncertainty weight must be > 0; unknown terrain is never free");
  }
  return true;
}

bool Limits::valid(std::string * why) const
{
  auto fail = [why](const char * msg) {
      if (why != nullptr) {*why = msg;}
      return false;
    };

  const double positive[] = {slope_max, slope_lethal, roughness_max,
    height_variance_max, step_max, step_lethal};
  for (double v : positive) {
    if (!std::isfinite(v) || v <= 0.0) {
      return fail("every limit must be finite and > 0");
    }
  }
  if (slope_lethal <= slope_max) {
    return fail("slope_lethal must exceed slope_max, or every saturated slope "
                "is already lethal");
  }
  if (step_lethal <= step_max) {
    return fail("step_lethal must exceed step_max");
  }
  if (!std::isfinite(unknown_cost) || unknown_cost < 0.0 || unknown_cost > 1.0) {
    return fail("unknown_cost must be within [0, 1]");
  }
  if (unknown_cost <= 0.0) {
    return fail("unknown_cost must be > 0; SPEC.md 6.2 -- unknown terrain is "
                "expensive, never free");
  }
  return true;
}

double normalise(double value, double limit, double fallback) noexcept
{
  if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0) {
    return fallback;
  }
  return std::clamp(std::fabs(value) / limit, 0.0, 1.0);
}

CellCost TraversabilityCore::evaluate(const Cell & cell) const
{
  CellCost out;

  // ---- 1. lethal geometry and lethal classes saturate ------------------
  // Checked before the observation test on purpose: a measured cliff edge is
  // lethal whether or not the rest of the cell is well observed.
  if (std::isfinite(cell.slope) && cell.slope >= limits_.slope_lethal) {
    out.cost = 1.0;
    out.lethal = true;
    out.unknown = false;
    out.lethal_reason = "slope above slope_lethal";
    return out;
  }
  if (std::isfinite(cell.step_height) && cell.step_height >= limits_.step_lethal) {
    out.cost = 1.0;
    out.lethal = true;
    out.unknown = false;
    out.lethal_reason = "step above step_lethal";
    return out;
  }
  if (cell.semantic_lethal) {
    out.cost = 1.0;
    out.lethal = true;
    out.unknown = false;
    out.lethal_reason = "lethal semantic class";
    return out;
  }

  // ---- 2. no usable observation ----------------------------------------
  // A cell is unknown if nothing observed it, or if the geometry it reports
  // is not finite. Treating a NaN slope as 0 would make a hole in the map
  // look like a flat road, which is precisely the ditch failure SPEC.md 6.2
  // exists to prevent.
  const bool geometry_usable =
    std::isfinite(cell.slope) &&
    std::isfinite(cell.roughness) &&
    std::isfinite(cell.height_variance) &&
    std::isfinite(cell.step_height);

  if (!cell.observed || !geometry_usable) {
    out.cost = limits_.unknown_cost;
    out.lethal = false;              // expensive, not forbidden
    out.unknown = true;
    out.uncertainty_term = 1.0;
    return out;
  }

  // ---- 3. the weighted sum, SPEC.md 6.1 --------------------------------
  out.unknown = false;
  out.slope_term = normalise(cell.slope, limits_.slope_max, 1.0);
  out.roughness_term = normalise(cell.roughness, limits_.roughness_max, 1.0);
  out.height_variance_term =
    normalise(cell.height_variance, limits_.height_variance_max, 1.0);
  out.obstacle_term = normalise(cell.step_height, limits_.step_max, 1.0);

  out.semantic_term = std::isfinite(cell.semantic_cost)
    ? std::clamp(cell.semantic_cost, 0.0, 1.0)
    : 1.0;

  // Uncertainty rises as either visibility or confidence falls; the worse of
  // the two governs. A well-lit cell the model is unsure about and a confident
  // reading of a barely-seen cell are both untrustworthy.
  const double vis = std::isfinite(cell.visibility)
    ? std::clamp(cell.visibility, 0.0, 1.0) : 0.0;
  const double conf = std::isfinite(cell.confidence)
    ? std::clamp(cell.confidence, 0.0, 1.0) : 0.0;
  out.uncertainty_term = 1.0 - std::min(vis, conf);

  const double weighted =
    weights_.slope * out.slope_term +
    weights_.roughness * out.roughness_term +
    weights_.height_variance * out.height_variance_term +
    weights_.obstacle * out.obstacle_term +
    weights_.semantic * out.semantic_term +
    weights_.uncertainty * out.uncertainty_term;

  const double total = weights_.sum();
  out.cost = (total > 0.0) ? std::clamp(weighted / total, 0.0, 1.0) : 1.0;
  return out;
}

std::uint8_t TraversabilityCore::to_costmap(const CellCost & c) noexcept
{
  if (c.lethal) {
    return kLethal;
  }
  const double cost = std::isfinite(c.cost) ? std::clamp(c.cost, 0.0, 1.0) : 1.0;
  const double scaled = std::lround(cost * static_cast<double>(kMaxNonLethal));
  return static_cast<std::uint8_t>(
    std::clamp(scaled, 0.0, static_cast<double>(kMaxNonLethal)));
}

}  // namespace drishti_traversability
