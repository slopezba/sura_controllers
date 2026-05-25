#include "sura_controllers/usv/body_position_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace sura_controllers::usv
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kCircleSegments = 96;

constexpr double kTargetPositionChangeTolerance = 1e-3;  // m
constexpr double kTargetYawChangeTolerance = 1e-3;       // rad

double shortestAngularDistance(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

}  // namespace

void BodyPositionController::resetDebugStats()
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

void BodyPositionController::publishDebugStats()
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
  msg.avg_update_us =
    cycle_count > 0 ? static_cast<double>(total_update_us) / static_cast<double>(cycle_count) : 0.0;
  msg.max_update_us =
    static_cast<double>(debug_max_update_us_.load(std::memory_order_relaxed));
  msg.min_update_us = static_cast<double>(
    min_update_us == std::numeric_limits<uint64_t>::max() ? 0ULL : min_update_us);
  msg.deadline_miss_count =
    debug_deadline_miss_count_.load(std::memory_order_relaxed);
  msg.cycle_count = cycle_count;

  debug_pub_->publish(msg);
}

void BodyPositionController::recordDebugCycle(
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

double BodyPositionController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double BodyPositionController::clampAbs(double value, double limit)
{
  if (limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

double BodyPositionController::yawFromPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void BodyPositionController::publishThresholdMarkers()
{
  if (!threshold_marker_pub_ || !has_active_target_) {
    return;
  }

  auto make_circle =
    [this](int id, double radius, float r, float g, float b)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "blueboat/map";
      marker.header.stamp = get_node()->now();
      marker.ns = "body_position_thresholds";
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

controller_interface::CallbackReturn BodyPositionController::on_init()
{
  try {
    auto_declare<std::string>("navigator_topic", "/navigator_msg");
    auto_declare<std::string>("setpoint_topic", "/body_position/setpoint");
    auto_declare<std::string>("body_velocity_controller_name", "body_velocity_controller");

    // APPROACH mode
    auto_declare<double>("kp_position", 0.6);
    auto_declare<double>("kp_yaw", 0.8);
    auto_declare<double>("max_linear_speed", 0.20);
    auto_declare<double>("max_angular_speed", 0.25);
    auto_declare<double>("position_hold_radius", 0.30);
    auto_declare<double>("position_release_radius", 0.60);
    auto_declare<double>("yaw_tolerance", 0.12);
    auto_declare<double>("slow_down_radius", 1.2);
    auto_declare<double>("heading_error_stop", 0.70);

    // Reverse. Para depurar lo dejo desactivable por YAML.
    auto_declare<double>("reverse_distance_threshold", 0.0);
    auto_declare<double>("max_reverse_speed", 0.0);

    auto_declare<double>("yaw_command_sign", 1.0);

    // HOLD mode
    auto_declare<double>("hold_position_deadband", 0.12);
    auto_declare<double>("hold_yaw_deadband", 0.12);
    auto_declare<double>("hold_kp_position", 0.25);
    auto_declare<double>("hold_kp_yaw", 0.6);
    auto_declare<double>("hold_max_linear_speed", 0.05);
    auto_declare<double>("hold_max_angular_speed", 0.10);
    auto_declare<bool>("debug.enabled", false);
    auto_declare<std::string>("debug.topic", "debug");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BodyPositionController::command_interface_configuration() const
{
  const std::string prefix = body_velocity_controller_name_;

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      prefix + "/linear.x",
      prefix + "/angular.z"
    }
  };
}

controller_interface::InterfaceConfiguration
BodyPositionController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::CallbackReturn BodyPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  setpoint_topic_ = get_node()->get_parameter("setpoint_topic").as_string();
  body_velocity_controller_name_ =
    get_node()->get_parameter("body_velocity_controller_name").as_string();
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
  reverse_distance_threshold_ = get_node()->get_parameter("reverse_distance_threshold").as_double();
  max_reverse_speed_ = get_node()->get_parameter("max_reverse_speed").as_double();
  yaw_command_sign_ = get_node()->get_parameter("yaw_command_sign").as_double() < 0.0 ? -1.0 : 1.0;

  hold_position_deadband_ = get_node()->get_parameter("hold_position_deadband").as_double();
  hold_yaw_deadband_ = get_node()->get_parameter("hold_yaw_deadband").as_double();
  hold_kp_position_ = get_node()->get_parameter("hold_kp_position").as_double();
  hold_kp_yaw_ = get_node()->get_parameter("hold_kp_yaw").as_double();
  hold_max_linear_speed_ = get_node()->get_parameter("hold_max_linear_speed").as_double();
  hold_max_angular_speed_ = get_node()->get_parameter("hold_max_angular_speed").as_double();

  if (position_release_radius_ < position_hold_radius_) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "position_release_radius < position_hold_radius. Forcing release radius to hold radius.");
    position_release_radius_ = position_hold_radius_;
  }

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
    });

  setpoint_sub_ = get_node()->create_subscription<SetPointMsg>(
    setpoint_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const SetPointMsg::SharedPtr msg)
    {
      setpoint_buffer_.writeFromNonRT(msg);
    });

  threshold_marker_pub_ =
    get_node()->create_publisher<MarkerArrayMsg>("/body_position/threshold_markers", 10);

  debug_pub_.reset();
  debug_timer_.reset();
  resetDebugStats();

  if (debug_enabled_) {
    debug_pub_ = get_node()->create_publisher<DebugMsg>(debug_topic_, 10);
    debug_timer_ = get_node()->create_wall_timer(
      std::chrono::seconds(1),
      [this]()
      {
        publishDebugStats();
      });
  }

  RCLCPP_INFO(get_node()->get_logger(), "Configured BodyPositionController");
  RCLCPP_INFO(get_node()->get_logger(), "navigator topic: %s", navigator_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "setpoint topic: %s", setpoint_topic_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(),
    "body_velocity_controller_name: %s",
    body_velocity_controller_name_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(),
    "approach: kp_position=%.3f kp_yaw=%.3f max_u=%.3f max_r=%.3f",
    kp_position_,
    kp_yaw_,
    max_linear_speed_,
    max_angular_speed_);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "hold: enter=%.3f release=%.3f pos_deadband=%.3f yaw_deadband=%.3f max_u=%.3f max_r=%.3f",
    position_hold_radius_,
    position_release_radius_,
    hold_position_deadband_,
    hold_yaw_deadband_,
    hold_max_linear_speed_,
    hold_max_angular_speed_);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "yaw_command_sign: %.1f",
    yaw_command_sign_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  last_setpoint_msg_ptr_ = nullptr;
  has_active_target_ = false;
  holding_current_position_ = false;
  controller_active_ = true;
  resetDebugStats();

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  last_setpoint_msg_ptr_ = nullptr;
  has_active_target_ = false;
  holding_current_position_ = false;
  controller_active_ = false;
  if (debug_enabled_ && debug_pub_) {
    publishDebugStats();
  }

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BodyPositionController::update(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const auto update_start = debug_enabled_ ? std::chrono::steady_clock::now() :
    std::chrono::steady_clock::time_point{};
  auto navigator_msg = navigator_buffer_.readFromRT();
  auto setpoint_msg = setpoint_buffer_.readFromRT();

  if (!navigator_msg || !(*navigator_msg)) {
    return controller_interface::return_type::OK;
  }

  if (command_interfaces_.size() != 2) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Expected 2 command interfaces, got %zu",
      command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  const double x = (*navigator_msg)->position.position.x;
  const double y = (*navigator_msg)->position.position.y;
  const double yaw = normalizeAngle((*navigator_msg)->rpy.z);

  if (setpoint_msg && *setpoint_msg) {
    const double candidate_x = (*setpoint_msg)->pose.position.x;
    const double candidate_y = (*setpoint_msg)->pose.position.y;
    const double candidate_yaw = normalizeAngle(yawFromPose((*setpoint_msg)->pose));

    const bool target_changed =
      !has_active_target_ ||
      std::abs(candidate_x - active_target_.x) > kTargetPositionChangeTolerance ||
      std::abs(candidate_y - active_target_.y) > kTargetPositionChangeTolerance ||
      std::abs(shortestAngularDistance(active_target_.yaw, candidate_yaw)) >
        kTargetYawChangeTolerance;

    if (target_changed) {
      active_target_.x = candidate_x;
      active_target_.y = candidate_y;
      active_target_.yaw = candidate_yaw;

      has_active_target_ = true;
      holding_current_position_ = false;

      RCLCPP_INFO(
        get_node()->get_logger(),
        "New position target: target=(%.2f, %.2f, %.2f)",
        active_target_.x,
        active_target_.y,
        active_target_.yaw);

      publishThresholdMarkers();
    }
  }

  if (!has_active_target_) {
    active_target_.x = x;
    active_target_.y = y;
    active_target_.yaw = yaw;
    has_active_target_ = true;
    holding_current_position_ = true;
    publishThresholdMarkers();
  }

  const double x_ref = active_target_.x;
  const double y_ref = active_target_.y;
  const double yaw_ref = active_target_.yaw;

  const double dx = x_ref - x;
  const double dy = y_ref - y;
  const double distance = std::hypot(dx, dy);

  const double desired_heading = std::atan2(dy, dx);
  const double heading_error = shortestAngularDistance(yaw, desired_heading);
  const double final_yaw_error = shortestAngularDistance(yaw, yaw_ref);

  // Error de posición expresado en el frame del barco.
  // x_body_error > 0: el target queda por delante.
  // x_body_error < 0: el target queda por detrás.
  // y_body_error > 0: el target queda hacia la izquierda.
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double x_body_error = cos_yaw * dx + sin_yaw * dy;
  const double y_body_error = -sin_yaw * dx + cos_yaw * dy;

  double linear_ref = 0.0;
  double angular_ref = 0.0;

  // HISTERESIS DE LLEGADA:
  // - Entramos en HOLD al entrar en position_hold_radius.
  // - No salimos de HOLD hasta alejarnos más de position_release_radius.
  if (!holding_current_position_ && distance <= position_hold_radius_) {
    holding_current_position_ = true;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Entering HOLD mode: dist=%.3f <= %.3f",
      distance,
      position_hold_radius_);
  }

  if (holding_current_position_ && distance > position_release_radius_) {
    holding_current_position_ = false;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Leaving HOLD mode: dist=%.3f > %.3f",
      distance,
      position_release_radius_);
  }

  if (!holding_current_position_) {
    // ============================================================
    // APPROACH MODE
    // ============================================================
    const double distance_for_gain =
      slow_down_radius_ > 0.0 ? std::min(distance, slow_down_radius_) : distance;

    const bool use_reverse =
      reverse_distance_threshold_ > 0.0 &&
      distance <= reverse_distance_threshold_ &&
      std::abs(heading_error) > (kPi / 2.0);

    const double motion_heading_error = use_reverse
      ? normalizeAngle(heading_error - std::copysign(kPi, heading_error))
      : heading_error;

    double heading_weight = std::max(0.0, std::cos(motion_heading_error));

    if (
      heading_error_stop_ > 0.0 &&
      std::abs(motion_heading_error) > heading_error_stop_)
    {
      heading_weight = 0.0;
    }

    const double speed_limit = use_reverse ? max_reverse_speed_ : max_linear_speed_;
    const double direction = use_reverse ? -1.0 : 1.0;

    linear_ref = direction * clampAbs(
      kp_position_ * distance_for_gain * heading_weight,
      speed_limit);

    angular_ref = yaw_command_sign_ *
      clampAbs(kp_yaw_ * motion_heading_error, max_angular_speed_);
  } else {
    // ============================================================
    // HOLD MODE
    // ============================================================
    // Aquí NO usamos atan2(dx, dy) para mandar yaw hacia el punto.
    // Eso cerca del objetivo es lo que provoca vueltas/orbiting.
    //
    // En HOLD:
    // 1) mantengo yaw_ref suavemente;
    // 2) corrijo posición solo en el eje longitudinal del barco;
    // 3) si la deriva lateral/total es grande, salgo de HOLD por release_radius.

    if (std::abs(x_body_error) > hold_position_deadband_) {
      linear_ref = clampAbs(
        hold_kp_position_ * x_body_error,
        hold_max_linear_speed_);
    } else {
      linear_ref = 0.0;
    }

    if (std::abs(final_yaw_error) > hold_yaw_deadband_) {
      angular_ref = yaw_command_sign_ *
        clampAbs(hold_kp_yaw_ * final_yaw_error, hold_max_angular_speed_);
    } else {
      angular_ref = 0.0;
    }
  }

  command_interfaces_[0].set_value(linear_ref);
  command_interfaces_[1].set_value(angular_ref);

  recordDebugCycle(update_start, period);

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "mode=%s target=(%.2f, %.2f, %.2f) pos=(%.2f, %.2f) "
    "dist=%.3f hold_radius=%.3f release_radius=%.3f "
    "yaw=%.2f desired_heading=%.2f heading_error=%.2f final_yaw_error=%.2f "
    "body_error=(%.3f, %.3f) u_ref=%.3f r_ref=%.3f",
    holding_current_position_ ? "HOLD" : "APPROACH",
    x_ref,
    y_ref,
    yaw_ref,
    x,
    y,
    distance,
    position_hold_radius_,
    position_release_radius_,
    yaw,
    desired_heading,
    heading_error,
    final_yaw_error,
    x_body_error,
    y_body_error,
    linear_ref,
    angular_ref);

  return controller_interface::return_type::OK;
}

}  // namespace sura_controllers::usv

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::usv::BodyPositionController,
  controller_interface::ControllerInterface)
