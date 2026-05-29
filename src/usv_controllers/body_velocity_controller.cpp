#include "sura_controllers/usv/body_velocity_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace sura_controllers::usv
{

void BodyVelocityController::resetDebugStats()
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

void BodyVelocityController::publishDebugStats()
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
  msg.chained_mode = chained_mode_;
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

void BodyVelocityController::recordDebugCycle(
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

controller_interface::CallbackReturn BodyVelocityController::on_init()
{
  try {
    auto_declare<std::string>("cmd_vel_topic", "cmd_vel");
    auto_declare<std::string>("navigator_topic", "navigator/msg");
    auto_declare<std::string>("body_force_controller_name", "body_force_controller");

    auto_declare<double>("kp_u", 0.0);
    auto_declare<double>("ki_u", 0.0);
    auto_declare<double>("kd_u", 0.0);
    auto_declare<double>("integral_limit_u", 0.0);
    auto_declare<double>("max_force_x", 0.0);

    auto_declare<double>("kp_r", 0.0);
    auto_declare<double>("ki_r", 0.0);
    auto_declare<double>("kd_r", 0.0);
    auto_declare<double>("integral_limit_r", 0.0);
    auto_declare<bool>("debug.enabled", false);
    auto_declare<std::string>("debug.topic", "debug");

    reference_interface_names_ = {
      "linear.x",
      "angular.z"
    };
    reference_interfaces_.resize(
      reference_interface_names_.size(),
      std::numeric_limits<double>::quiet_NaN());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BodyVelocityController::command_interface_configuration() const
{
  const std::string prefix = body_force_controller_name_;

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      prefix + "/force.x",
      prefix + "/force.y",
      prefix + "/force.z",
      prefix + "/torque.x",
      prefix + "/torque.y",
      prefix + "/torque.z"
    }
  };
}

controller_interface::InterfaceConfiguration
BodyVelocityController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::CallbackReturn BodyVelocityController::on_configure(
  const rclcpp_lifecycle::State &)
{
  cmd_vel_topic_ = get_node()->get_parameter("cmd_vel_topic").as_string();
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  body_force_controller_name_ =
    get_node()->get_parameter("body_force_controller_name").as_string();
  debug_enabled_ = get_node()->get_parameter("debug.enabled").as_bool();
  debug_topic_ = get_node()->get_parameter("debug.topic").as_string();

  kp_u_ = get_node()->get_parameter("kp_u").as_double();
  ki_u_ = get_node()->get_parameter("ki_u").as_double();
  kd_u_ = get_node()->get_parameter("kd_u").as_double();
  integral_limit_u_ = get_node()->get_parameter("integral_limit_u").as_double();
  max_force_x_ = get_node()->get_parameter("max_force_x").as_double();

  kp_r_ = get_node()->get_parameter("kp_r").as_double();
  ki_r_ = get_node()->get_parameter("ki_r").as_double();
  kd_r_ = get_node()->get_parameter("kd_r").as_double();
  integral_limit_r_ = get_node()->get_parameter("integral_limit_r").as_double();

  cmd_vel_sub_ = get_node()->create_subscription<TwistMsg>(
    cmd_vel_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const TwistMsg::SharedPtr msg)
    {
      cmd_vel_buffer_.writeFromNonRT(msg);
    });

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
    });

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
    std::bind(&BodyVelocityController::parametersCallback, this, std::placeholders::_1));

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

  RCLCPP_INFO(get_node()->get_logger(), "Configured BodyVelocityController");
  RCLCPP_INFO(get_node()->get_logger(), "cmd_vel topic: %s", cmd_vel_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "navigator topic: %s", navigator_topic_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(),
    "body_force_controller_name: %s",
    body_force_controller_name_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Initial gains: kp_u=%.3f ki_u=%.3f kd_u=%.3f | kp_r=%.3f ki_r=%.3f kd_r=%.3f",
    kp_u_, ki_u_, kd_u_, kp_r_, ki_r_, kd_r_);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Integral limits: u=%.3f r=%.3f",
    integral_limit_u_, integral_limit_r_);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Max force x: %.3f",
    max_force_x_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyVelocityController::on_activate(
  const rclcpp_lifecycle::State &)
{
  integral_u_ = 0.0;
  integral_r_ = 0.0;
  prev_error_u_ = 0.0;
  prev_error_r_ = 0.0;
  first_update_ = true;
  controller_active_ = true;
  resetDebugStats();
  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyVelocityController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  integral_u_ = 0.0;
  integral_r_ = 0.0;
  prev_error_u_ = 0.0;
  prev_error_r_ = 0.0;
  first_update_ = true;
  controller_active_ = false;
  if (debug_enabled_ && debug_pub_) {
    publishDebugStats();
  }
  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
BodyVelocityController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> exported_reference_interfaces;
  exported_reference_interfaces.reserve(reference_interface_names_.size());

  for (size_t i = 0; i < reference_interface_names_.size(); ++i) {
    exported_reference_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        get_node()->get_name(),
        reference_interface_names_[i],
        &reference_interfaces_[i]));
  }

  return exported_reference_interfaces;
}

bool BodyVelocityController::on_set_chained_mode(bool chained_mode)
{
  chained_mode_ = chained_mode;
  if (chained_mode) {
    RCLCPP_INFO(get_node()->get_logger(), "BodyVelocityController switched to chained mode");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "BodyVelocityController switched to topic mode");
  }
  return true;
}

controller_interface::return_type BodyVelocityController::update_reference_from_subscribers()
{
  auto cmd_vel_msg = cmd_vel_buffer_.readFromRT();

  if (!(!cmd_vel_msg || !(*cmd_vel_msg))) {
    reference_interfaces_[0] = (*cmd_vel_msg)->linear.x;
    reference_interfaces_[1] = (*cmd_vel_msg)->angular.z;
  }

  return controller_interface::return_type::OK;
}

rcl_interfaces::msg::SetParametersResult BodyVelocityController::parametersCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : params) {
    const auto & name = param.get_name();

    if (name == "kp_u") {
      kp_u_ = param.as_double();
    } else if (name == "ki_u") {
      ki_u_ = param.as_double();
      integral_u_ = 0.0;
    } else if (name == "kd_u") {
      kd_u_ = param.as_double();
    } else if (name == "integral_limit_u") {
      integral_limit_u_ = param.as_double();
    } else if (name == "kp_r") {
      kp_r_ = param.as_double();
    } else if (name == "ki_r") {
      ki_r_ = param.as_double();
      integral_r_ = 0.0;
    } else if (name == "kd_r") {
      kd_r_ = param.as_double();
    } else if (name == "integral_limit_r") {
      integral_limit_r_ = param.as_double();
    }
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Updated gains: kp_u=%.3f ki_u=%.3f kd_u=%.3f | kp_r=%.3f ki_r=%.3f kd_r=%.3f",
    kp_u_, ki_u_, kd_u_, kp_r_, ki_r_, kd_r_);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Updated integral limits: u=%.3f r=%.3f",
    integral_limit_u_, integral_limit_r_);

  return result;
}

controller_interface::return_type BodyVelocityController::update_and_write_commands(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const auto update_start = debug_enabled_ ? std::chrono::steady_clock::now() :
    std::chrono::steady_clock::time_point{};
  auto navigator_msg = navigator_buffer_.readFromRT();

  if (!navigator_msg || !(*navigator_msg)) {
    return controller_interface::return_type::OK;
  }

  const double u_ref = std::isnan(reference_interfaces_[0]) ? 0.0 : reference_interfaces_[0];
  const double r_ref = std::isnan(reference_interfaces_[1]) ? 0.0 : reference_interfaces_[1];

  const double u_meas = (*navigator_msg)->body_velocity.linear.x;
  const double r_meas = (*navigator_msg)->body_velocity.angular.z;

  const double error_u = u_ref - u_meas;
  const double error_r = r_ref - r_meas;

  const double dt = period.seconds();

  integral_u_ += error_u * dt;
  integral_r_ += error_r * dt;

  if (integral_limit_u_ > 0.0) {
    integral_u_ = std::clamp(integral_u_, -integral_limit_u_, integral_limit_u_);
  }
  if (integral_limit_r_ > 0.0) {
    integral_r_ = std::clamp(integral_r_, -integral_limit_r_, integral_limit_r_);
  }

  double derivative_u = 0.0;
  double derivative_r = 0.0;

  if (!first_update_ && dt > 0.0) {
    derivative_u = (error_u - prev_error_u_) / dt;
    derivative_r = (error_r - prev_error_r_) / dt;
  }

  prev_error_u_ = error_u;
  prev_error_r_ = error_r;
  first_update_ = false;

  const double force_x_raw =
    kp_u_ * error_u +
    ki_u_ * integral_u_ +
    kd_u_ * derivative_u;

  double force_x = force_x_raw;
  if (max_force_x_ > 0.0) {
    force_x = std::clamp(force_x_raw, -max_force_x_, max_force_x_);
    if (ki_u_ > 0.0 && dt > 0.0 && std::abs(force_x - force_x_raw) > 1.0e-9) {
      integral_u_ = (force_x - kp_u_ * error_u - kd_u_ * derivative_u) / ki_u_;
      if (integral_limit_u_ > 0.0) {
        integral_u_ = std::clamp(integral_u_, -integral_limit_u_, integral_limit_u_);
      }
    }
  }

  const double torque_z =
    kp_r_ * error_r +
    ki_r_ * integral_r_ +
    kd_r_ * derivative_r;

  if (command_interfaces_.size() != 6) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Expected 6 command interfaces, got %zu",
      command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  command_interfaces_[0].set_value(force_x);
  command_interfaces_[1].set_value(0.0);
  command_interfaces_[2].set_value(0.0);
  command_interfaces_[3].set_value(0.0);
  command_interfaces_[4].set_value(0.0);
  command_interfaces_[5].set_value(torque_z);

  recordDebugCycle(update_start, period);

  return controller_interface::return_type::OK;
}

}  // namespace sura_controllers::usv

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::usv::BodyVelocityController,
  controller_interface::ChainableControllerInterface)
