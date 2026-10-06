// Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
//
// ROS 2 wrapper around SupervisorCore.
//
// This node holds NO policy. It collects evidence, ticks on its own timer,
// calls SupervisorCore::evaluate(), and publishes the result. Every safety
// decision lives in supervisor_core.cpp, which is unit-tested without ROS.
//
// SPEC.md section 9.4:
//   1. This is the only publisher on /cmd_vel.
//   2. Loss of input is a stop condition, never a pass-through.
//   3. It never increases a commanded velocity.
//   4. It ticks on its own timer, independent of whether Nav2 is publishing.
//
// !! UNVERIFIED !! This file has never been compiled: no machine on the
// project has ROS 2 installed yet (STATUS.md B3). supervisor_core.cpp and its
// tests ARE verified. Expect to fix small API details here on the first real
// colcon build.

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <drishti_msgs/msg/perception_health.hpp>
#include <drishti_msgs/msg/safety_state.hpp>

#include "drishti_safety/supervisor_core.hpp"

namespace drishti_safety
{

class SafetySupervisorNode : public rclcpp::Node
{
public:
  SafetySupervisorNode()
  : rclcpp::Node("safety_supervisor"), core_(load_params())
  {
    const std::string cmd_in = declare_parameter<std::string>("cmd_vel_in", "/cmd_vel_nav");
    const std::string cmd_out = declare_parameter<std::string>("cmd_vel_out", "/cmd_vel");
    use_twist_stamped_ = declare_parameter<bool>("use_twist_stamped", false);

    // Reliable QoS for anything the stop decision depends on. Sensor data
    // arrives best-effort, but its *health summary* must not be dropped.
    const auto qos = rclcpp::QoS(10).reliable();

    health_sub_ = create_subscription<drishti_msgs::msg::PerceptionHealth>(
      "/perception/health", qos,
      [this](drishti_msgs::msg::PerceptionHealth::SharedPtr m) {
        last_health_ = *m;
        last_health_rx_ = now().seconds();
      });

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/rtabmap/localization_pose", qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        // Largest positional diagonal term: xx, yy, yaw-yaw.
        const auto & c = m->pose.covariance;
        pose_cov_max_ = std::max({c[0], c[7], c[35]});
        pose_rx_ = now().seconds();
      });

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", qos,
      [this](nav_msgs::msg::Path::SharedPtr m) {
        plan_usable_ = m->poses.size() >= 2;
        plan_rx_ = now().seconds();
      });

    obstacle_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/perception/nearest_obstacle", qos,
      [this](std_msgs::msg::Float32::SharedPtr m) {
        nearest_obstacle_ = static_cast<double>(m->data);
        obstacle_rx_ = now().seconds();
      });

    if (use_twist_stamped_) {
      cmd_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_in, qos,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr m) {
          accept_command(m->twist.linear.x, m->twist.angular.z);
        });
      cmd_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_out, qos);
    } else {
      cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_in, qos,
        [this](geometry_msgs::msg::Twist::SharedPtr m) {
          accept_command(m->linear.x, m->angular.z);
        });
      cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_out, qos);
    }

    state_pub_ = create_publisher<drishti_msgs::msg::SafetyState>("/safety/state", qos);
    stop_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/stop", qos);

    // Invariant 9.4.4: our own timer, not a callback on Nav2's rate.
    const auto period = std::chrono::duration<double>(core_.params().watchdog_period);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {tick();});

    RCLCPP_INFO(
      get_logger(),
      "safety_supervisor up: %s -> %s (%s), tick %.0f Hz",
      cmd_in.c_str(), cmd_out.c_str(),
      use_twist_stamped_ ? "TwistStamped" : "Twist",
      1.0 / core_.params().watchdog_period);
    log_params();
  }

private:
  Params load_params()
  {
    Params p;
    p.t_camera_stale = declare_parameter<double>("t_camera_stale", p.t_camera_stale);
    p.t_depth_stale = declare_parameter<double>("t_depth_stale", p.t_depth_stale);
    p.d_emergency = declare_parameter<double>("d_emergency", p.d_emergency);
    p.c_critical = declare_parameter<double>("c_critical", p.c_critical);
    p.v_max = declare_parameter<double>("v_max", p.v_max);
    p.v_slow = declare_parameter<double>("v_slow", p.v_slow);
    p.cov_max = declare_parameter<double>("cov_max", p.cov_max);
    p.watchdog_period = declare_parameter<double>("watchdog_period", p.watchdog_period);

    t_plan_stale_ = declare_parameter<double>("t_plan_stale", 2.0);
    t_cmd_stale_ = declare_parameter<double>("t_cmd_stale", 0.5);
    t_pose_stale_ = declare_parameter<double>("t_pose_stale", 0.5);

    std::string why;
    if (!p.valid(&why)) {
      // A bad safety configuration must not start. Failing loudly here is far
      // better than discovering it from a collision in the results log.
      RCLCPP_FATAL(get_logger(), "invalid safety parameters: %s", why.c_str());
      throw std::invalid_argument("invalid safety parameters: " + why);
    }
    return p;
  }

  /// SPEC.md section 9.3: thresholds are logged at startup with the run.
  void log_params() const
  {
    const auto & p = core_.params();
    RCLCPP_INFO(
      get_logger(),
      "thresholds: t_camera_stale=%.3f t_depth_stale=%.3f d_emergency=%.3f "
      "c_critical=%.3f v_max=%.3f v_slow=%.3f cov_max=%.3f watchdog=%.3f "
      "t_plan_stale=%.3f t_cmd_stale=%.3f t_pose_stale=%.3f",
      p.t_camera_stale, p.t_depth_stale, p.d_emergency, p.c_critical,
      p.v_max, p.v_slow, p.cov_max, p.watchdog_period,
      t_plan_stale_, t_cmd_stale_, t_pose_stale_);
  }

  void accept_command(double linear_x, double angular_z)
  {
    cmd_linear_x_ = linear_x;
    cmd_angular_z_ = angular_z;
    cmd_rx_ = now().seconds();
  }

  Inputs gather(double t) const
  {
    Inputs in;
    in.now = t;

    // Perception health is itself an input that can go stale. If the health
    // report stops arriving, perception is as untrustworthy as a dead camera.
    const bool health_fresh =
      (t - last_health_rx_) <= core_.params().t_camera_stale;

    if (health_fresh) {
      in.last_rgb_stamp = rclcpp::Time(last_health_.last_rgb_stamp).seconds();
      in.last_depth_stamp = rclcpp::Time(last_health_.last_depth_stamp).seconds();
      in.perception_confidence = last_health_.mean_confidence;
      in.rgb_static_for = last_health_.rgb_static_for;
    }  // otherwise both stay at kNever -> stale -> STOP

    in.pose_valid = (t - pose_rx_) <= t_pose_stale_;
    in.pose_covariance_max = in.pose_valid ? pose_cov_max_ : kInf;

    in.nearest_obstacle =
      ((t - obstacle_rx_) <= core_.params().t_depth_stale) ? nearest_obstacle_ : kNaN;

    in.path_valid = plan_usable_ && ((t - plan_rx_) <= t_plan_stale_);

    if ((t - cmd_rx_) <= t_cmd_stale_) {
      in.cmd_linear_x = cmd_linear_x_;
      in.cmd_angular_z = cmd_angular_z_;
    } else {
      // Nav2 has gone quiet. Do not keep republishing its last command.
      in.cmd_linear_x = kNaN;
      in.cmd_angular_z = kNaN;
    }
    return in;
  }

  void tick()
  {
    const double t = now().seconds();
    const Inputs in = gather(t);
    const Decision d = core_.evaluate(in);

    publish_cmd(d);

    drishti_msgs::msg::SafetyState s;
    s.header.stamp = now();
    s.header.frame_id = "base_link";
    s.action = static_cast<uint8_t>(d.action);
    s.reason = static_cast<uint8_t>(d.reason);
    s.detail = to_string(d.reason);
    s.stop = d.stop;
    s.v_limit = static_cast<float>(d.v_limit);
    s.commanded_linear_x = static_cast<float>(d.linear_x);
    s.commanded_angular_z = static_cast<float>(d.angular_z);
    s.rgb_age = static_cast<float>(d.rgb_age);
    s.depth_age = static_cast<float>(d.depth_age);
    s.pose_covariance_max = static_cast<float>(in.pose_covariance_max);
    s.nearest_obstacle = static_cast<float>(in.nearest_obstacle);
    s.perception_confidence = static_cast<float>(in.perception_confidence);
    state_pub_->publish(s);

    std_msgs::msg::Bool stop;
    stop.data = d.stop;
    stop_pub_->publish(stop);

    if (d.reason != last_reason_) {
      RCLCPP_WARN(
        get_logger(), "safety %s: %s", to_string(d.action), to_string(d.reason));
      last_reason_ = d.reason;
    }
  }

  void publish_cmd(const Decision & d)
  {
    if (use_twist_stamped_) {
      geometry_msgs::msg::TwistStamped m;
      m.header.stamp = now();
      m.header.frame_id = "base_link";
      m.twist.linear.x = d.linear_x;
      m.twist.angular.z = d.angular_z;
      cmd_stamped_pub_->publish(m);
    } else {
      geometry_msgs::msg::Twist m;
      m.linear.x = d.linear_x;
      m.angular.z = d.angular_z;
      cmd_pub_->publish(m);
    }
  }

  // Declared before core_ deliberately: core_'s mem-initializer calls
  // load_params(), which assigns these three via declare_parameter(). C++
  // initializes members in declaration order regardless of the constructor
  // initializer-list order, so if these were declared after core_, their own
  // in-class default initializers ({2.0}, {0.5}, {0.5}) would run right after
  // load_params() and silently clobber whatever value it just assigned --
  // any params.yaml override of t_plan_stale/t_cmd_stale/t_pose_stale would
  // then be discarded with no error. Declaring them first means their
  // defaults apply BEFORE load_params() runs, so its assignment is the one
  // that sticks.
  bool use_twist_stamped_{false};
  double t_plan_stale_{2.0};
  double t_cmd_stale_{0.5};
  double t_pose_stale_{0.5};

  SupervisorCore core_;

  drishti_msgs::msg::PerceptionHealth last_health_;
  double last_health_rx_{kNever};
  double pose_cov_max_{kInf};
  double pose_rx_{kNever};
  bool plan_usable_{false};
  double plan_rx_{kNever};
  double nearest_obstacle_{kNaN};
  double obstacle_rx_{kNever};
  double cmd_linear_x_{0.0};
  double cmd_angular_z_{0.0};
  double cmd_rx_{kNever};
  Reason last_reason_{Reason::NONE};

  rclcpp::Subscription<drishti_msgs::msg::PerceptionHealth>::SharedPtr health_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;
  rclcpp::Publisher<drishti_msgs::msg::SafetyState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drishti_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drishti_safety::SafetySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
