// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// Nav2 costmap layer that consumes /traversability.
//
// SPEC.md 6.3: the intended adaptation is to feed a costmap layer a
// camera-derived terrain representation, not to reinvent the costmap concept.
// This layer therefore does almost nothing -- it projects an already-computed
// cost field into the costmap and takes the maximum against what is there.
//
// It NEVER lowers a cost. A cell another layer marked lethal stays lethal, and
// a terrain map that has not seen a cell cannot clear an obstacle the depth
// sensor saw. Taking the maximum is what makes layer ordering harmless.
//
// !! UNVERIFIED !! Never compiled; nav2_costmap_2d is not installed.

#ifndef DRISHTI_TRAVERSABILITY__TRAVERSABILITY_LAYER_HPP_
#define DRISHTI_TRAVERSABILITY__TRAVERSABILITY_LAYER_HPP_

#include <mutex>
#include <string>

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "drishti_traversability/traversability_core.hpp"

namespace drishti_traversability
{

class TraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TraversabilityLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double * min_x, double * min_y,
                    double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  void onFootprintChanged() override {}

  /// The terrain map is evidence, not clutter. Nav2 must not clear it as if it
  /// were a transient sensor reading.
  bool isClearable() override {return false;}

private:
  void onMap(grid_map_msgs::msg::GridMap::SharedPtr msg);

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  std::mutex mutex_;
  grid_map::GridMap map_;
  bool have_map_{false};
  rclcpp::Time last_stamp_;

  std::string topic_{"/traversability"};
  std::string cost_layer_{"traversability"};
  std::string lethal_layer_{"lethal"};
  double max_age_{2.0};
};

}  // namespace drishti_traversability

#endif  // DRISHTI_TRAVERSABILITY__TRAVERSABILITY_LAYER_HPP_
