#include "sura_controllers/usv/position_hold_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace sura_controllers::usv
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kCircleSegments = 96;
constexpr double kTargetPositionChangeTolerance = 1e-3;
constexpr double kTargetYawChangeTolerance = 1e-3;

int64_t steadyTimeNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double shortestAngularDistance(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

}  // namespace

void PositionHoldController::resetDebugStats()
{
  debug_desired_period_us_.store(0, std::memory_order_relaxed);
  debug_cycle_count_.store(0, std::memory_order_relaxed);
  debug_deadline_miss_count_.store(0, std::memory_order_relaxed);
  debug_total_update_us_.store(0, std::memory_order_relaxed);
  debug_last_update_us_.store(0, std::memory_order_relaxed);
  debug_max_update_us_.store(0, std::memory_order_relaxed);
  debug_min_update_us_.store(
    std::numeric_limits<uint64_t>::max(), std::memory_order_relaxed);
}

void PositionHoldController::publishDebugStats()
{
  if (!debug_enabled_ || !debug_pub_) {
    return;
  }

  const uint64_t cycle_count = debug_cycle_count_.load(std::memory_order_relaxed);
  const uint64_t total_update_us = debug_total_update_us_.load(std::memory_order_relaxed);
  const uint64_t min_update_us = debug_min_update_us_.load(std::memory_order_relaxed);

  DebugMsg msg;
  msg.header.stamp = get_node()->now();
  msg.controller_name = get_node()->get_name();
  msg.active = controller_active_;
  msg.chained_mode = false;
  msg.desired_period_us =
    static_cast<double>(debug_desired_period_us_.load(std::memory_order_relaxed));
  msg.last_update_us =
    static_cast<double>(debug_last_update_us_.load(std::memory_order_relaxed));
  msg.avg_update_us = cycle_count > 0 ?
    static_cast<double>(total_update_us) / static_cast<double>(cycle_count) : 0.0;
  msg.max_update_us =
    static_cast<double>(debug_max_update_us_.load(std::memory_order_relaxed));
  msg.min_update_us = static_cast<double>(
    min_update_us == std::numeric_limits<uint64_t>::max() ? 0ULL : min_update_us);
  msg.deadline_miss_count = debug_deadline_miss_count_.load(std::memory_order_relaxed);
  msg.cycle_count = cycle_count;
  debug_pub_->publish(msg);
}

void PositionHoldController::recordDebugCycle(
  const std::chrono::steady_clock::time_point & update_start,
  const rclcpp::Duration & period)
{
  if (!debug_enabled_) {
    return;
  }

  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - update_start).count();
  const uint64_t elapsed_us = static_cast<uint64_t>(std::max<int64_t>(elapsed, 0));
  const uint64_t target_period_us = static_cast<uint64_t>(
    std::max<int64_t>(period.nanoseconds() / 1000, 0));

  debug_desired_period_us_.store(target_period_us, std::memory_order_relaxed);
  debug_cycle_count_.fetch_add(1, std::memory_order_relaxed);
  debug_total_update_us_.fetch_add(elapsed_us, std::memory_order_relaxed);
  debug_last_update_us_.store(elapsed_us, std::memory_order_relaxed);

  uint64_t current_max = debug_max_update_us_.load(std::memory_order_relaxed);
  while (elapsed_us > current_max &&
    !debug_max_update_us_.compare_exchange_weak(
      current_max, elapsed_us, std::memory_order_relaxed, std::memory_order_relaxed))
  {
  }

  uint64_t current_min = debug_min_update_us_.load(std::memory_order_relaxed);
  while (elapsed_us < current_min &&
    !debug_min_update_us_.compare_exchange_weak(
      current_min, elapsed_us, std::memory_order_relaxed, std::memory_order_relaxed))
  {
  }

  if (target_period_us > 0 && elapsed_us > target_period_us) {
    debug_deadline_miss_count_.fetch_add(1, std::memory_order_relaxed);
  }
}

double PositionHoldController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double PositionHoldController::clampAbs(double value, double limit)
{
  if (limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

double PositionHoldController::yawFromPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion PositionHoldController::quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

bool PositionHoldController::setActiveTarget(
  double x, double y, double yaw, bool hold_current_position)
{
  yaw = normalizeAngle(yaw);
  const bool target_changed =
    !has_active_target_ ||
    std::abs(x - active_target_.x) > kTargetPositionChangeTolerance ||
    std::abs(y - active_target_.y) > kTargetPositionChangeTolerance ||
    std::abs(shortestAngularDistance(active_target_.yaw, yaw)) > kTargetYawChangeTolerance;

  if (!target_changed) {
    return false;
  }

  active_target_.x = x;
  active_target_.y = y;
  active_target_.yaw = yaw;
  has_active_target_ = true;
  holding_current_position_ = hold_current_position;
  return true;
}

void PositionHoldController::publishCurrentSetpoint(const rclcpp::Time & stamp)
{
  if (!has_active_target_ || !setpoint_rt_pub_ || !setpoint_rt_pub_->trylock()) {
    return;
  }

  auto & msg = setpoint_rt_pub_->msg_;
  msg.header.stamp = stamp;
  msg.header.frame_id = active_target_frame_id_.empty() ? "world_ned" : active_target_frame_id_;
  msg.pose.position.x = active_target_.x;
  msg.pose.position.y = active_target_.y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation = quaternionFromYaw(active_target_.yaw);
  setpoint_rt_pub_->unlockAndPublish();
}

void PositionHoldController::publishThresholdMarkers()
{
  if (!threshold_marker_pub_ || !has_active_target_) {
    return;
  }

  auto make_circle = [this](int id, double radius, float r, float g, float b) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = setpoint_frame_id_.empty() ? "world_ned" : setpoint_frame_id_;
      marker.header.stamp = get_node()->now();
      marker.ns = "position_hold_thresholds";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.04;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 0.9;
      marker.points.reserve(kCircleSegments + 1);

      for (int i = 0; i <= kCircleSegments; ++i) {
        const double angle = 2.0 * kPi * static_cast<double>(i) /
          static_cast<double>(kCircleSegments);
        geometry_msgs::msg::Point point;
        point.x = active_target_.x + radius * std::cos(angle);
        point.y = active_target_.y + radius * std::sin(angle);
        point.z = 0.05;
        marker.points.push_back(point);
      }
      return marker;
    };

  MarkerArrayMsg markers;
  markers.markers.push_back(
    make_circle(0, position_hold_radius_, 0.0F, 1.0F, 0.0F));
  markers.markers.push_back(
    make_circle(1, position_release_radius_, 1.0F, 0.55F, 0.0F));
  threshold_marker_pub_->publish(markers);
}

void PositionHoldController::publishBodyVelocitySetpoint(const TwistMsg & command)
{
  if (body_velocity_setpoint_rt_pub_ && body_velocity_setpoint_rt_pub_->trylock()) {
    body_velocity_setpoint_rt_pub_->msg_ = command;
    body_velocity_setpoint_rt_pub_->unlockAndPublish();
  }
}

void PositionHoldController::publishZeroBodyVelocitySetpoint()
{
  if (body_velocity_setpoint_pub_) {
    body_velocity_setpoint_pub_->publish(TwistMsg{});
  }
}

void PositionHoldController::publishTelemetry(
  const TwistMsg & command,
  const std::array<double, 6> & control_terms)
{
  if (output_rt_pub_ && output_rt_pub_->trylock()) {
    output_rt_pub_->msg_ = command;
    output_rt_pub_->unlockAndPublish();
  }

  if (pid_terms_rt_pub_ && pid_terms_rt_pub_->trylock()) {
    std::copy(
      control_terms.begin(), control_terms.end(), pid_terms_rt_pub_->msg_.data.begin());
    pid_terms_rt_pub_->unlockAndPublish();
  }
}

controller_interface::CallbackReturn PositionHoldController::on_init()
{
  try {
    auto_declare<std::string>("setpoint_topic", "position_hold/setpoint");
    auto_declare<std::string>("feedforward_topic", "position_hold/feedforward");
    auto_declare<std::string>(
      "reposition_feedforward_topic", "position_hold/reposition_feedforward");
    auto_declare<std::string>("navigator_topic", "navigator/navigation");
    auto_declare<std::string>("output_topic", "position_hold/output");
    auto_declare<std::string>("pid_terms_topic", "position_hold/pid_terms");
    auto_declare<std::string>(
      "threshold_markers_topic", "position_hold/threshold_markers");
    auto_declare<std::string>("body_velocity_controller_name", "body_velocity");
    auto_declare<std::string>("body_velocity_setpoint_topic", "body_velocity/setpoint");
    auto_declare<std::string>("setpoint_frame_id", "world_ned");

    auto_declare<double>("kp_position", 0.6);
    auto_declare<double>("kp_yaw", 0.8);
    auto_declare<double>("max_linear_speed", 0.20);
    auto_declare<double>("max_angular_speed", 0.25);
    auto_declare<double>("position_hold_radius", 0.30);
    auto_declare<double>("position_release_radius", 0.60);
    auto_declare<double>("yaw_tolerance", 0.12);
    auto_declare<double>("slow_down_radius", 1.2);
    auto_declare<double>("heading_error_stop", 0.70);
    auto_declare<double>("reverse_distance_threshold", 0.0);
    auto_declare<double>("max_reverse_speed", 0.0);

    auto_declare<double>("hold_position_deadband", 0.12);
    auto_declare<double>("hold_yaw_deadband", 0.12);
    auto_declare<double>("hold_kp_position", 0.25);
    auto_declare<double>("hold_kp_yaw", 0.6);
    auto_declare<double>("hold_max_linear_speed", 0.05);
    auto_declare<double>("hold_max_angular_speed", 0.10);

    auto_declare<double>("linear_feedforward_threshold", 1e-3);
    auto_declare<double>("angular_feedforward_threshold", 1e-3);
    auto_declare<double>("feedforward_timeout", 0.25);
    auto_declare<bool>("debug.enabled", false);
    auto_declare<std::string>("debug.topic", "debug");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PositionHoldController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
PositionHoldController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn PositionHoldController::on_configure(
  const rclcpp_lifecycle::State &)
{
  setpoint_topic_ = get_node()->get_parameter("setpoint_topic").as_string();
  feedforward_topic_ = get_node()->get_parameter("feedforward_topic").as_string();
  reposition_feedforward_topic_ =
    get_node()->get_parameter("reposition_feedforward_topic").as_string();
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  output_topic_ = get_node()->get_parameter("output_topic").as_string();
  pid_terms_topic_ = get_node()->get_parameter("pid_terms_topic").as_string();
  threshold_markers_topic_ =
    get_node()->get_parameter("threshold_markers_topic").as_string();
  body_velocity_controller_name_ =
    get_node()->get_parameter("body_velocity_controller_name").as_string();
  body_velocity_setpoint_topic_ =
    get_node()->get_parameter("body_velocity_setpoint_topic").as_string();
  setpoint_frame_id_ = get_node()->get_parameter("setpoint_frame_id").as_string();
  debug_enabled_ = get_node()->get_parameter("debug.enabled").as_bool();
  debug_topic_ = get_node()->get_parameter("debug.topic").as_string();

  kp_position_ = get_node()->get_parameter("kp_position").as_double();
  kp_yaw_ = get_node()->get_parameter("kp_yaw").as_double();
  max_linear_speed_ = get_node()->get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = get_node()->get_parameter("max_angular_speed").as_double();
  position_hold_radius_ = get_node()->get_parameter("position_hold_radius").as_double();
  position_release_radius_ = get_node()->get_parameter("position_release_radius").as_double();
  yaw_tolerance_ = get_node()->get_parameter("yaw_tolerance").as_double();
  slow_down_radius_ = get_node()->get_parameter("slow_down_radius").as_double();
  heading_error_stop_ = get_node()->get_parameter("heading_error_stop").as_double();
  reverse_distance_threshold_ = get_node()->get_parameter(
    "reverse_distance_threshold").as_double();
  max_reverse_speed_ = get_node()->get_parameter("max_reverse_speed").as_double();
  hold_position_deadband_ = get_node()->get_parameter("hold_position_deadband").as_double();
  hold_yaw_deadband_ = get_node()->get_parameter("hold_yaw_deadband").as_double();
  hold_kp_position_ = get_node()->get_parameter("hold_kp_position").as_double();
  hold_kp_yaw_ = get_node()->get_parameter("hold_kp_yaw").as_double();
  hold_max_linear_speed_ = get_node()->get_parameter("hold_max_linear_speed").as_double();
  hold_max_angular_speed_ = get_node()->get_parameter("hold_max_angular_speed").as_double();
  linear_feedforward_threshold_ = std::max(
    0.0, get_node()->get_parameter("linear_feedforward_threshold").as_double());
  angular_feedforward_threshold_ = std::max(
    0.0, get_node()->get_parameter("angular_feedforward_threshold").as_double());
  feedforward_timeout_ = std::max(
    0.0, get_node()->get_parameter("feedforward_timeout").as_double());

  if (position_release_radius_ < position_hold_radius_) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "position_release_radius < position_hold_radius. Forcing release radius to hold radius.");
    position_release_radius_ = position_hold_radius_;
  }

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg) {navigator_buffer_.writeFromNonRT(msg);});

  rclcpp::SubscriptionOptions setpoint_options;
  setpoint_options.ignore_local_publications = true;
  setpoint_sub_ = get_node()->create_subscription<PoseStampedMsg>(
    setpoint_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const PoseStampedMsg::SharedPtr msg) {
      setpoint_buffer_.writeFromNonRT(msg);
      new_setpoint_requested_.store(true);
    }, setpoint_options);

  feedforward_sub_ = get_node()->create_subscription<TwistMsg>(
    feedforward_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const TwistMsg::SharedPtr msg) {
      feedforward_buffer_.writeFromNonRT(msg);
      last_feedforward_time_ns_.store(steadyTimeNanoseconds());
    });
  reposition_feedforward_sub_ = get_node()->create_subscription<TwistMsg>(
    reposition_feedforward_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const TwistMsg::SharedPtr msg) {
      reposition_feedforward_buffer_.writeFromNonRT(msg);
      last_reposition_feedforward_time_ns_.store(steadyTimeNanoseconds());
    });

  setpoint_pub_ = get_node()->create_publisher<PoseStampedMsg>(
    setpoint_topic_, rclcpp::SystemDefaultsQoS());
  setpoint_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<PoseStampedMsg>>(setpoint_pub_);
  body_velocity_setpoint_pub_ = get_node()->create_publisher<TwistMsg>(
    body_velocity_setpoint_topic_, rclcpp::SystemDefaultsQoS());
  body_velocity_setpoint_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<TwistMsg>>(body_velocity_setpoint_pub_);
  output_pub_ = get_node()->create_publisher<TwistMsg>(
    output_topic_, rclcpp::SystemDefaultsQoS());
  output_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<TwistMsg>>(output_pub_);
  pid_terms_pub_ = get_node()->create_publisher<Float64MultiArrayMsg>(
    pid_terms_topic_, rclcpp::SystemDefaultsQoS());
  pid_terms_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<Float64MultiArrayMsg>>(pid_terms_pub_);
  pid_terms_rt_pub_->msg_.data.resize(6, 0.0);
  threshold_marker_pub_ = get_node()->create_publisher<MarkerArrayMsg>(
    threshold_markers_topic_, rclcpp::SystemDefaultsQoS());

  debug_pub_.reset();
  debug_timer_.reset();
  resetDebugStats();
  if (debug_enabled_) {
    debug_pub_ = get_node()->create_publisher<DebugMsg>(debug_topic_, 10);
    debug_timer_ = get_node()->create_wall_timer(
      std::chrono::seconds(1), [this]() {publishDebugStats();});
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured USV PositionHoldController: setpoint='%s', feedforward='%s', "
    "reposition='%s', body_velocity='%s' (%s)",
    setpoint_topic_.c_str(), feedforward_topic_.c_str(),
    reposition_feedforward_topic_.c_str(), body_velocity_setpoint_topic_.c_str(),
    body_velocity_controller_name_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionHoldController::on_activate(
  const rclcpp_lifecycle::State &)
{
  has_active_target_ = false;
  holding_current_position_ = false;
  feedforward_active_ = false;
  reposition_feedforward_active_ = false;
  new_setpoint_requested_.store(false);
  last_feedforward_time_ns_.store(0);
  last_reposition_feedforward_time_ns_.store(0);
  active_target_frame_id_ = setpoint_frame_id_.empty() ? "world_ned" : setpoint_frame_id_;
  controller_active_ = true;
  resetDebugStats();
  publishZeroBodyVelocitySetpoint();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionHoldController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = false;
  has_active_target_ = false;
  holding_current_position_ = false;
  feedforward_active_ = false;
  reposition_feedforward_active_ = false;
  new_setpoint_requested_.store(false);
  last_feedforward_time_ns_.store(0);
  last_reposition_feedforward_time_ns_.store(0);
  publishZeroBodyVelocitySetpoint();
  if (debug_enabled_ && debug_pub_) {
    publishDebugStats();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PositionHoldController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  const auto update_start = debug_enabled_ ? std::chrono::steady_clock::now() :
    std::chrono::steady_clock::time_point{};
  auto navigator_msg = navigator_buffer_.readFromRT();
  if (!navigator_msg || !(*navigator_msg)) {
    const TwistMsg zero_command;
    publishBodyVelocitySetpoint(zero_command);
    publishTelemetry(zero_command, std::array<double, 6>{});
    recordDebugCycle(update_start, period);
    return controller_interface::return_type::OK;
  }

  const double x = (*navigator_msg)->position.position.x;
  const double y = (*navigator_msg)->position.position.y;
  const double yaw = normalizeAngle((*navigator_msg)->rpy.z);

  auto setpoint_msg = setpoint_buffer_.readFromRT();
  if (new_setpoint_requested_.exchange(false) && setpoint_msg && *setpoint_msg) {
    active_target_frame_id_ = (*setpoint_msg)->header.frame_id.empty() ?
      (setpoint_frame_id_.empty() ? "world_ned" : setpoint_frame_id_) :
      (*setpoint_msg)->header.frame_id;
    if (setActiveTarget(
        (*setpoint_msg)->pose.position.x,
        (*setpoint_msg)->pose.position.y,
        yawFromPose((*setpoint_msg)->pose), false))
    {
      RCLCPP_INFO(
        get_node()->get_logger(), "New external target: (%.2f, %.2f, %.2f)",
        active_target_.x, active_target_.y, active_target_.yaw);
      publishCurrentSetpoint(time);
      publishThresholdMarkers();
    }
  }

  if (!has_active_target_) {
    active_target_frame_id_ = setpoint_frame_id_.empty() ? "world_ned" : setpoint_frame_id_;
    if (setActiveTarget(x, y, yaw, true)) {
      publishCurrentSetpoint(time);
      publishThresholdMarkers();
    }
  }

  TwistMsg effective_feedforward;
  const int64_t now_ns = steadyTimeNanoseconds();
  const int64_t feedforward_time_ns = last_feedforward_time_ns_.load();
  const bool feedforward_is_fresh = feedforward_time_ns > 0 &&
    static_cast<double>(now_ns - feedforward_time_ns) * 1e-9 <= feedforward_timeout_;
  auto feedforward_msg = feedforward_buffer_.readFromRT();
  if (feedforward_is_fresh && feedforward_msg && *feedforward_msg) {
    effective_feedforward.linear.x = (*feedforward_msg)->linear.x;
    effective_feedforward.angular.z = (*feedforward_msg)->angular.z;
  }
  const bool has_feedforward =
    std::abs(effective_feedforward.linear.x) > linear_feedforward_threshold_ ||
    std::abs(effective_feedforward.angular.z) > angular_feedforward_threshold_;

  TwistMsg effective_reposition;
  const int64_t reposition_time_ns = last_reposition_feedforward_time_ns_.load();
  const bool reposition_is_fresh = reposition_time_ns > 0 &&
    static_cast<double>(now_ns - reposition_time_ns) * 1e-9 <= feedforward_timeout_;
  auto reposition_msg = reposition_feedforward_buffer_.readFromRT();
  if (reposition_is_fresh && reposition_msg && *reposition_msg && !has_feedforward) {
    effective_reposition.linear.x = (*reposition_msg)->linear.x;
    effective_reposition.angular.z = (*reposition_msg)->angular.z;
  }
  const bool has_reposition =
    std::abs(effective_reposition.linear.x) > linear_feedforward_threshold_ ||
    std::abs(effective_reposition.angular.z) > angular_feedforward_threshold_;

  if (!has_feedforward && (has_reposition || reposition_feedforward_active_)) {
    active_target_frame_id_ = setpoint_frame_id_.empty() ? "world_ned" : setpoint_frame_id_;
    if (setActiveTarget(x, y, yaw, true)) {
      publishCurrentSetpoint(time);
      publishThresholdMarkers();
    }
  }

  const double dx = active_target_.x - x;
  const double dy = active_target_.y - y;
  const double distance = std::hypot(dx, dy);
  const double desired_heading = std::atan2(dy, dx);
  const double heading_error = shortestAngularDistance(yaw, desired_heading);
  const double final_yaw_error = shortestAngularDistance(yaw, active_target_.yaw);
  const double x_body_error = std::cos(yaw) * dx + std::sin(yaw) * dy;

  double linear_control = 0.0;
  double angular_control = 0.0;
  if (!holding_current_position_ && distance <= position_hold_radius_) {
    holding_current_position_ = true;
    RCLCPP_INFO(
      get_node()->get_logger(), "Entering HOLD mode: dist=%.3f <= %.3f",
      distance, position_hold_radius_);
  }
  if (holding_current_position_ && distance > position_release_radius_) {
    holding_current_position_ = false;
    RCLCPP_INFO(
      get_node()->get_logger(), "Leaving HOLD mode: dist=%.3f > %.3f",
      distance, position_release_radius_);
  }

  if (!has_feedforward) {
    if (!holding_current_position_) {
      const double distance_for_gain = slow_down_radius_ > 0.0 ?
        std::min(distance, slow_down_radius_) : distance;
      const bool use_reverse = reverse_distance_threshold_ > 0.0 &&
        distance <= reverse_distance_threshold_ && std::abs(heading_error) > (kPi / 2.0);
      const double motion_heading_error = use_reverse ?
        normalizeAngle(heading_error - std::copysign(kPi, heading_error)) : heading_error;
      double heading_weight = std::max(0.0, std::cos(motion_heading_error));
      if (heading_error_stop_ > 0.0 &&
        std::abs(motion_heading_error) > heading_error_stop_)
      {
        heading_weight = 0.0;
      }
      const double speed_limit = use_reverse ? max_reverse_speed_ : max_linear_speed_;
      const double direction = use_reverse ? -1.0 : 1.0;
      linear_control = direction * clampAbs(
        kp_position_ * distance_for_gain * heading_weight, speed_limit);
      angular_control = clampAbs(
        kp_yaw_ * motion_heading_error, max_angular_speed_);
    } else {
      if (std::abs(x_body_error) > hold_position_deadband_) {
        linear_control = clampAbs(
          hold_kp_position_ * x_body_error, hold_max_linear_speed_);
      }
      if (std::abs(final_yaw_error) > hold_yaw_deadband_) {
        angular_control = clampAbs(
          hold_kp_yaw_ * final_yaw_error, hold_max_angular_speed_);
      }
    }
  }

  TwistMsg command;
  command.linear.x = effective_feedforward.linear.x + effective_reposition.linear.x +
    linear_control;
  command.angular.z = effective_feedforward.angular.z + effective_reposition.angular.z +
    angular_control;
  publishBodyVelocitySetpoint(command);
  publishTelemetry(
    command, {linear_control, 0.0, 0.0, angular_control, 0.0, 0.0});

  feedforward_active_ = has_feedforward;
  reposition_feedforward_active_ = has_reposition;
  recordDebugCycle(update_start, period);

  return controller_interface::return_type::OK;
}

}  // namespace sura_controllers::usv

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::usv::PositionHoldController,
  controller_interface::ControllerInterface)
