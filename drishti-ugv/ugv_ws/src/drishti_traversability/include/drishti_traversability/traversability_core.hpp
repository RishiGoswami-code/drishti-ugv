// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// Traversability cost function -- decision core.
//
// SPEC.md section 6.1. Like the safety supervisor, this header depends on
// nothing but the C++ standard library: no ROS, no grid_map, no costmap_2d.
// The fusion node and the Nav2 layer are thin wrappers that feed cells through
// evaluate() and translate the answer. All the judgement lives here, where it
// can be tested exhaustively on any machine.
//
// This is the part of the system that is genuinely specific to the problem
// statement. Nav2, RTAB-Map and the elevation mapper are all reused; turning
// camera-derived terrain into a number a planner will respect is the work.
//
// !! The ROS wrappers are UNVERIFIED. This core is compiled and tested.

#ifndef DRISHTI_TRAVERSABILITY__TRAVERSABILITY_CORE_HPP_
#define DRISHTI_TRAVERSABILITY__TRAVERSABILITY_CORE_HPP_

#include <cstdint>
#include <limits>
#include <string>

namespace drishti_traversability
{

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// nav2_costmap_2d cost values, restated so this core needs no ROS header.
inline constexpr std::uint8_t kFreeSpace = 0;
inline constexpr std::uint8_t kMaxNonLethal = 252;
inline constexpr std::uint8_t kInscribed = 253;
inline constexpr std::uint8_t kLethal = 254;
inline constexpr std::uint8_t kNoInformation = 255;

/// Relative importance of each term in SPEC.md 6.1.
///
/// These are RELATIVE. evaluate() divides by their sum, so doubling every
/// weight changes nothing and raising one weight raises that term's share.
/// Without that normalisation the mapping from C(x) to costmap bytes would
/// silently rescale every time a single weight was retuned, and no two
/// experiments in EVALUATION.md 5 would be comparable.
struct Weights
{
  double slope{1.0};
  double roughness{1.0};
  double height_variance{0.8};
  double obstacle{1.5};
  double semantic{1.2};
  double uncertainty{1.0};

  double sum() const noexcept
  {
    return slope + roughness + height_variance + obstacle + semantic + uncertainty;
  }

  bool valid(std::string * why = nullptr) const;
};

/// Where each term saturates, and where geometry becomes lethal.
///
/// Defaults are sized for the platform in drishti_description: 0.10 m wheel
/// radius, 0.44 m track, hull 0.10 m off the ground. They are starting points,
/// not tuned values -- SPEC.md 6.1 requires weights and thresholds to be
/// settled by controlled experiment, and EVALUATION.md 5 records which set
/// produced which mission result.
struct Limits
{
  double slope_max{0.35};            ///< rad, ~20 deg, term reaches 1.0
  double slope_lethal{0.52};         ///< rad, ~30 deg, cell is lethal
  double roughness_max{0.08};        ///< m, residual to the fitted plane
  double height_variance_max{0.010};  ///< m^2
  double step_max{0.12};             ///< m, term reaches 1.0
  double step_lethal{0.25};          ///< m, above the wheel radius: lethal

  /// SPEC.md 6.2: unknown terrain is expensive, never free.
  ///
  /// Deliberately high and deliberately NOT lethal. Lethal would forbid the
  /// planner from ever entering unobserved space, and a vehicle that refuses
  /// to drive anywhere it has not already seen cannot reach a goal. Expensive
  /// lets it route around ignorance when there is an alternative and press on
  /// when there is not.
  double unknown_cost{0.85};

  bool valid(std::string * why = nullptr) const;
};

/// One elevation-map cell, as handed over by the fusion node.
///
/// Every default is the pessimistic value. A cell nobody filled in is
/// unobserved, and unobserved is expensive.
struct Cell
{
  bool observed{false};

  double slope{kNaN};             ///< rad, from the surface normal
  double roughness{kNaN};         ///< m, residual to a locally fitted plane
  double height_variance{kNaN};   ///< m^2, from the elevation map
  double step_height{kNaN};       ///< m, largest height jump to a neighbour

  double semantic_cost{0.0};      ///< 0..1 from the SPEC.md 5.2 taxonomy
  bool semantic_lethal{false};    ///< ditch, cliff, water, trunk, person

  double visibility{1.0};         ///< 0..1, ray-traced coverage of the cell
  double confidence{1.0};         ///< 0..1, perception confidence
};

/// The cost, and every term that produced it.
///
/// The breakdown is not decoration: weight tuning is an experiment
/// (EVALUATION.md 5), and without per-term values a bad mission result cannot
/// be attributed to a term.
struct CellCost
{
  double cost{1.0};               ///< 0..1 normalised
  bool lethal{false};
  bool unknown{true};             ///< no usable observation

  double slope_term{0.0};
  double roughness_term{0.0};
  double height_variance_term{0.0};
  double obstacle_term{0.0};
  double semantic_term{0.0};
  double uncertainty_term{1.0};

  /// Reason the cell saturated, empty when it did not. For RViz and logs.
  const char * lethal_reason{""};
};

class TraversabilityCore
{
public:
  TraversabilityCore(const Weights & w, const Limits & l) noexcept
  : weights_(w), limits_(l) {}

  /// Cost for one cell. Pure: same input, same answer, no state.
  ///
  /// Order of decisions:
  ///   1. any lethal geometry or lethal semantic class saturates the cell and
  ///      short-circuits everything else
  ///   2. an unobserved cell, or one whose inputs are not finite, costs
  ///      `unknown_cost` -- never free, never lethal (SPEC.md 6.2)
  ///   3. otherwise the weighted normalised sum of SPEC.md 6.1
  ///
  /// Non-finite inputs are treated as absence of observation, not as zero.
  /// A NaN slope must never make a cell look flat.
  CellCost evaluate(const Cell & cell) const;

  /// Map a cost to the nav2_costmap_2d byte range.
  ///
  /// Lethal saturates. Everything else lands in [0, 252], leaving 253
  /// (inscribed) and 255 (no information) to mean what Nav2 expects. An
  /// unknown cell is emitted as an expensive KNOWN cost, not as
  /// NO_INFORMATION: the planner runs with allow_unknown false, so
  /// NO_INFORMATION would make unobserved space impassable rather than
  /// merely costly.
  static std::uint8_t to_costmap(const CellCost & c) noexcept;

  const Weights & weights() const noexcept {return weights_;}
  const Limits & limits() const noexcept {return limits_;}

private:
  Weights weights_;
  Limits limits_;
};

/// Normalise `value` onto [0, 1] against `limit`, treating non-finite input as
/// "no information" and returning `fallback` for it.
double normalise(double value, double limit, double fallback) noexcept;

}  // namespace drishti_traversability

#endif  // DRISHTI_TRAVERSABILITY__TRAVERSABILITY_CORE_HPP_
