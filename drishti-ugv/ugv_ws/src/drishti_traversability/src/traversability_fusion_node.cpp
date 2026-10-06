// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// Reads the elevation map, writes a traversability map.
//
//   /elevation_map  (grid_map_msgs/GridMap)   from elevation_mapping_cupy
//        |
//        v   TraversabilityCore::evaluate, cell by cell
//   /traversability (grid_map_msgs/GridMap)   consumed by the Nav2 layer
//
// This node holds no policy. Every judgement is in traversability_core.cpp,
// which is compiled and tested without ROS (1217 checks). What lives here is
// plumbing: pull named layers out of the incoming map, hand each cell to the
// core, write the answer back.
//
// LAYER NAMES ARE PARAMETERS, not constants. elevation_mapping_cupy publishes
// whatever its own plugin configuration produces, and slope/roughness come
// from optional plugins that must be enabled. A layer that is absent leaves
// its cells UNOBSERVED, which the core prices at unknown_cost -- expensive,
// never free. That is the safe failure: a mis-set layer name makes the robot
// cautious, not blind.
//
// !! UNVERIFIED !! Never compiled. grid_map and ROS 2 are not installed
// anywhere on the project (STATUS.md D17).

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "drishti_traversability/traversability_core.hpp"

namespace drishti_traversability
{

class TraversabilityFusionNode : public rclcpp::Node
{
public:
  TraversabilityFusionNode()
  : rclcpp::Node("traversability_fusion"),
    core_(load_weights(), load_limits())
  {
    layer_elevation_ = declare_parameter<std::string>("layer.elevation", "elevation");
    layer_variance_ = declare_parameter<std::string>("layer.variance", "variance");
    layer_slope_ = declare_parameter<std::string>("layer.slope", "slope");
    layer_roughness_ = declare_parameter<std::string>("layer.roughness", "roughness");
    layer_step_ = declare_parameter<std::string>("layer.step", "step");
    layer_visibility_ = declare_parameter<std::string>("layer.visibility", "visibility");
    layer_semantic_ = declare_parameter<std::string>("layer.semantic_cost", "semantic_cost");
    layer_semantic_lethal_ =
      declare_parameter<std::string>("layer.semantic_lethal", "semantic_lethal");
    layer_confidence_ = declare_parameter<std::string>("layer.confidence", "confidence");

    publish_debug_layers_ = declare_parameter<bool>("publish_debug_layers", true);

    const auto qos = rclcpp::QoS(1).reliable();
    sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      "/elevation_map", qos,
      [this](grid_map_msgs::msg::GridMap::SharedPtr msg) {on_map(*msg);});
    pub_ = create_publisher<grid_map_msgs::msg::GridMap>("/traversability", qos);

    log_configuration();
  }

private:
  Weights load_weights()
  {
    Weights w;
    w.slope = declare_parameter<double>("weights.slope", w.slope);
    w.roughness = declare_parameter<double>("weights.roughness", w.roughness);
    w.height_variance =
      declare_parameter<double>("weights.height_variance", w.height_variance);
    w.obstacle = declare_parameter<double>("weights.obstacle", w.obstacle);
    w.semantic = declare_parameter<double>("weights.semantic", w.semantic);
    w.uncertainty = declare_parameter<double>("weights.uncertainty", w.uncertainty);

    std::string why;
    if (!w.valid(&why)) {
      RCLCPP_FATAL(get_logger(), "invalid traversability weights: %s", why.c_str());
      throw std::invalid_argument("invalid traversability weights: " + why);
    }
    return w;
  }

  Limits load_limits()
  {
    Limits l;
    l.slope_max = declare_parameter<double>("limits.slope_max", l.slope_max);
    l.slope_lethal = declare_parameter<double>("limits.slope_lethal", l.slope_lethal);
    l.roughness_max = declare_parameter<double>("limits.roughness_max", l.roughness_max);
    l.height_variance_max =
      declare_parameter<double>("limits.height_variance_max", l.height_variance_max);
    l.step_max = declare_parameter<double>("limits.step_max", l.step_max);
    l.step_lethal = declare_parameter<double>("limits.step_lethal", l.step_lethal);
    l.unknown_cost = declare_parameter<double>("limits.unknown_cost", l.unknown_cost);

    std::string why;
    if (!l.valid(&why)) {
      RCLCPP_FATAL(get_logger(), "invalid traversability limits: %s", why.c_str());
      throw std::invalid_argument("invalid traversability limits: " + why);
    }
    return l;
  }

  /// SPEC.md 6.1: record every weight set with the result it produced. Logging
  /// them at startup is what makes a rosbag self-describing.
  void log_configuration() const
  {
    const auto & w = core_.weights();
    const auto & l = core_.limits();
    RCLCPP_INFO(
      get_logger(),
      "weights: slope=%.3f roughness=%.3f height_var=%.3f obstacle=%.3f "
      "semantic=%.3f uncertainty=%.3f (sum %.3f)",
      w.slope, w.roughness, w.height_variance, w.obstacle, w.semantic,
      w.uncertainty, w.sum());
    RCLCPP_INFO(
      get_logger(),
      "limits: slope_max=%.3f slope_lethal=%.3f roughness_max=%.3f "
      "height_var_max=%.4f step_max=%.3f step_lethal=%.3f unknown_cost=%.3f",
      l.slope_max, l.slope_lethal, l.roughness_max, l.height_variance_max,
      l.step_max, l.step_lethal, l.unknown_cost);
  }

  /// Read one cell of a layer, or NaN when the layer is absent.
  static double at(const grid_map::GridMap & map, const std::string & layer,
                   const grid_map::Index & idx)
  {
    if (!map.exists(layer)) {
      return kNaN;
    }
    return static_cast<double>(map.at(layer, idx));
  }

  void on_map(const grid_map_msgs::msg::GridMap & msg)
  {
    grid_map::GridMap in;
    if (!grid_map::GridMapRosConverter::fromMessage(msg, in)) {
      RCLCPP_WARN(get_logger(), "could not convert incoming elevation map");
      return;
    }

    warn_once_about_missing_layers(in);

    grid_map::GridMap out(in);
    out.add("traversability", grid_map::Matrix::Constant(
        in.getSize()(0), in.getSize()(1), static_cast<float>(kNaN)));
    out.add("lethal", 0.0f);
    if (publish_debug_layers_) {
      out.add("t_slope", 0.0f);
      out.add("t_obstacle", 0.0f);
      out.add("t_uncertainty", 0.0f);
      out.add("t_unknown", 0.0f);
    }

    std::size_t unknown = 0, lethal = 0, total = 0;

    for (grid_map::GridMapIterator it(in); !it.isPastEnd(); ++it) {
      const grid_map::Index idx(*it);
      ++total;

      Cell cell;
      // A cell is observed when the elevation map has a finite height for it.
      const double elevation = at(in, layer_elevation_, idx);
      cell.observed = std::isfinite(elevation);

      cell.slope = at(in, layer_slope_, idx);
      cell.roughness = at(in, layer_roughness_, idx);
      cell.height_variance = at(in, layer_variance_, idx);
      cell.step_height = at(in, layer_step_, idx);

      const double semantic = at(in, layer_semantic_, idx);
      cell.semantic_cost = std::isfinite(semantic) ? semantic : 0.0;
      const double semantic_lethal = at(in, layer_semantic_lethal_, idx);
      cell.semantic_lethal = std::isfinite(semantic_lethal) && semantic_lethal > 0.5;

      // Absent visibility or confidence means we do not know how well the cell
      // was seen. That is not the same as "seen perfectly": default to zero so
      // the uncertainty term prices it, rather than to one which would assert
      // a confidence nothing measured.
      const double visibility = at(in, layer_visibility_, idx);
      cell.visibility = std::isfinite(visibility) ? visibility : 0.0;
      const double confidence = at(in, layer_confidence_, idx);
      cell.confidence = std::isfinite(confidence) ? confidence : 0.0;

      const CellCost c = core_.evaluate(cell);

      out.at("traversability", idx) = static_cast<float>(c.cost);
      out.at("lethal", idx) = c.lethal ? 1.0f : 0.0f;
      if (publish_debug_layers_) {
        out.at("t_slope", idx) = static_cast<float>(c.slope_term);
        out.at("t_obstacle", idx) = static_cast<float>(c.obstacle_term);
        out.at("t_uncertainty", idx) = static_cast<float>(c.uncertainty_term);
        out.at("t_unknown", idx) = c.unknown ? 1.0f : 0.0f;
      }
      unknown += c.unknown ? 1 : 0;
      lethal += c.lethal ? 1 : 0;
    }

    auto out_msg = grid_map::GridMapRosConverter::toMessage(out);
    // Keep the source timestamp. Restamping with now() would hide the
    // pose/depth desynchronisation TASK.md Phase 3 asks us to look for.
    out_msg->header = msg.header;
    pub_->publish(*out_msg);

    RCLCPP_DEBUG(
      get_logger(), "traversability: %zu cells, %zu unknown, %zu lethal",
      total, unknown, lethal);
  }

  void warn_once_about_missing_layers(const grid_map::GridMap & in)
  {
    if (warned_) {
      return;
    }
    warned_ = true;
    const std::vector<std::pair<std::string, std::string>> wanted = {
      {"elevation", layer_elevation_}, {"variance", layer_variance_},
      {"slope", layer_slope_}, {"roughness", layer_roughness_},
      {"step", layer_step_}, {"visibility", layer_visibility_},
    };
    for (const auto & [role, name] : wanted) {
      if (!in.exists(name)) {
        RCLCPP_WARN(
          get_logger(),
          "elevation map has no '%s' layer (role: %s). Those cells will be "
          "priced as unobserved. Enable the matching elevation_mapping_cupy "
          "plugin or correct layer.%s.",
          name.c_str(), role.c_str(), role.c_str());
      }
    }
  }

  TraversabilityCore core_;
  bool warned_{false};
  bool publish_debug_layers_{true};

  std::string layer_elevation_, layer_variance_, layer_slope_, layer_roughness_;
  std::string layer_step_, layer_visibility_, layer_semantic_;
  std::string layer_semantic_lethal_, layer_confidence_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
};

}  // namespace drishti_traversability

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drishti_traversability::TraversabilityFusionNode>());
  rclcpp::shutdown();
  return 0;
}
