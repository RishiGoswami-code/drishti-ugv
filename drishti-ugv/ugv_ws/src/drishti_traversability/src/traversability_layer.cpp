// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// See the header. This layer projects /traversability into the costmap and
// never lowers an existing cost.
//
// !! UNVERIFIED !! Never compiled.

#include "drishti_traversability/traversability_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace drishti_traversability
{

void TraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TraversabilityLayer: owning node has gone away");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(topic_));
  declareParameter("cost_layer", rclcpp::ParameterValue(cost_layer_));
  declareParameter("lethal_layer", rclcpp::ParameterValue(lethal_layer_));
  declareParameter("max_age", rclcpp::ParameterValue(max_age_));

  bool enabled = true;
  node->get_parameter(name_ + ".enabled", enabled);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".cost_layer", cost_layer_);
  node->get_parameter(name_ + ".lethal_layer", lethal_layer_);
  node->get_parameter(name_ + ".max_age", max_age_);

  enabled_ = enabled;
  current_ = false;
  last_stamp_ = node->now();

  sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    topic_, rclcpp::QoS(1).reliable(),
    std::bind(&TraversabilityLayer::onMap, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "TraversabilityLayer on %s (cost layer %s, max age %.1f s)",
    topic_.c_str(), cost_layer_.c_str(), max_age_);
}

void TraversabilityLayer::onMap(grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  grid_map::GridMap incoming;
  if (!grid_map::GridMapRosConverter::fromMessage(*msg, incoming)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  map_ = std::move(incoming);
  last_stamp_ = rclcpp::Time(msg->header.stamp);
  have_map_ = true;
}

void TraversabilityLayer::updateBounds(
  double, double, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_ || !have_map_) {
    return;
  }
  const grid_map::Position centre = map_.getPosition();
  const grid_map::Length length = map_.getLength();
  *min_x = std::min(*min_x, centre.x() - length.x() / 2.0);
  *min_y = std::min(*min_y, centre.y() - length.y() / 2.0);
  *max_x = std::max(*max_x, centre.x() + length.x() / 2.0);
  *max_y = std::max(*max_y, centre.y() + length.y() / 2.0);
}

void TraversabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_ || !have_map_) {
    return;
  }

  auto node = node_.lock();
  if (node) {
    const double age = (node->now() - last_stamp_).seconds();
    if (age > max_age_) {
      // A stale terrain map must not keep authorising motion. Reporting the
      // layer as not-current tells Nav2 the costmap is unreliable; the safety
      // supervisor stops the vehicle independently on stale depth, so this is
      // the second of two defences, not the only one.
      current_ = false;
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 5000,
        "traversability map is %.1f s old (max %.1f); layer is stale",
        age, max_age_);
      return;
    }
    current_ = true;
  }

  if (!map_.exists(cost_layer_)) {
    return;
  }
  const bool has_lethal = map_.exists(lethal_layer_);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(static_cast<unsigned int>(i),
                             static_cast<unsigned int>(j), wx, wy);

      grid_map::Index idx;
      if (!map_.getIndex(grid_map::Position(wx, wy), idx)) {
        continue;                        // outside the terrain map
      }

      CellCost c;
      // A non-finite value stays non-finite on purpose: to_costmap prices it
      // at maximum expense rather than free.
      c.cost = static_cast<double>(map_.at(cost_layer_, idx));
      c.lethal = has_lethal && map_.at(lethal_layer_, idx) > 0.5f;

      const unsigned char proposed = TraversabilityCore::to_costmap(c);
      const unsigned char existing = master_grid.getCost(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j));

      // NO_INFORMATION is an absence, not a low cost, so it may be replaced.
      // Anything else may only ever rise.
      const unsigned char updated =
        (existing == nav2_costmap_2d::NO_INFORMATION)
        ? proposed
        : std::max(existing, proposed);

      master_grid.setCost(static_cast<unsigned int>(i),
                          static_cast<unsigned int>(j), updated);
    }
  }
}

void TraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  have_map_ = false;
  current_ = false;
}

}  // namespace drishti_traversability

PLUGINLIB_EXPORT_CLASS(
  drishti_traversability::TraversabilityLayer, nav2_costmap_2d::Layer)
