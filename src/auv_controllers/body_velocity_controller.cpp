#include "sura_controllers/auv/body_velocity_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace sura_controllers::auv
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

  sura_msgs::msg::ControllerDebug msg;
  const uint64_t cycle_count = debug_cycle_count_.load(std::memory_order_relaxed);
  const uint64_t total_update_us = debug_total_update_us_.load(std::memory_order_relaxed);
  const uint64_t min_update_us = debug_min_update_us_.load(std::memory_order_relaxed);

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

controller_interface::CallbackReturn BodyVelocityController::on_init()
{
  try {
    auto_declare<std::string>(
      "setpoint_topic", "/cirtesub/controller/body_velocity/setpoint");
    auto_declare<std::string>(
      "navigator_topic", "/cirtesub/navigator/navigation");
    auto_declare<std::string>(
      "feedforward_topic", "/cirtesub/controller/stabilize/feedforward");
    auto_declare<std::string>(
      "output_topic", "/cirtesub/controller/body_velocity/output");
    auto_declare<std::string>(
      "pid_terms_topic", "/cirtesub/controller/body_velocity/pid_terms");
    auto_declare<std::string>(
      "body_force_controller_name", "body_force");

    auto_declare<double>("kp_x", 0.0);
    auto_declare<double>("ki_x", 0.0);
    auto_declare<double>("kd_x", 0.0);
    auto_declare<double>("antiwindup_x", 0.0);

    auto_declare<double>("kp_y", 0.0);
    auto_declare<double>("ki_y", 0.0);
    auto_declare<double>("kd_y", 0.0);
    auto_declare<double>("antiwindup_y", 0.0);

    auto_declare<double>("kp_z", 0.0);
    auto_declare<double>("ki_z", 0.0);
    auto_declare<double>("kd_z", 0.0);
    auto_declare<double>("antiwindup_z", 0.0);

    auto_declare<double>("kp_roll", 0.0);
    auto_declare<double>("ki_roll", 0.0);
    auto_declare<double>("kd_roll", 0.0);
    auto_declare<double>("antiwindup_roll", 0.0);

    auto_declare<double>("kp_pitch", 0.0);
    auto_declare<double>("ki_pitch", 0.0);
    auto_declare<double>("kd_pitch", 0.0);
    auto_declare<double>("antiwindup_pitch", 0.0);

    auto_declare<double>("kp_yaw", 0.0);
    auto_declare<double>("ki_yaw", 0.0);
    auto_declare<double>("kd_yaw", 0.0);
    auto_declare<double>("antiwindup_yaw", 0.0);
    auto_declare<bool>("debug.enabled", false);
    auto_declare<std::string>("debug.topic", "debug");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BodyVelocityController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
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
  setpoint_topic_ = get_node()->get_parameter("setpoint_topic").as_string();
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  feedforward_topic_ = get_node()->get_parameter("feedforward_topic").as_string();
  output_topic_ = get_node()->get_parameter("output_topic").as_string();
  pid_terms_topic_ = get_node()->get_parameter("pid_terms_topic").as_string();
  debug_enabled_ = get_node()->get_parameter("debug.enabled").as_bool();
  debug_topic_ = get_node()->get_parameter("debug.topic").as_string();
  body_force_controller_name_ =
    get_node()->get_parameter("body_force_controller_name").as_string();

  kp_x_ = get_node()->get_parameter("kp_x").as_double();
  ki_x_ = get_node()->get_parameter("ki_x").as_double();
  kd_x_ = get_node()->get_parameter("kd_x").as_double();
  antiwindup_x_ = std::abs(get_node()->get_parameter("antiwindup_x").as_double());

  kp_y_ = get_node()->get_parameter("kp_y").as_double();
  ki_y_ = get_node()->get_parameter("ki_y").as_double();
  kd_y_ = get_node()->get_parameter("kd_y").as_double();
  antiwindup_y_ = std::abs(get_node()->get_parameter("antiwindup_y").as_double());

  kp_z_ = get_node()->get_parameter("kp_z").as_double();
  ki_z_ = get_node()->get_parameter("ki_z").as_double();
  kd_z_ = get_node()->get_parameter("kd_z").as_double();
  antiwindup_z_ = std::abs(get_node()->get_parameter("antiwindup_z").as_double());

  kp_roll_ = get_node()->get_parameter("kp_roll").as_double();
  ki_roll_ = get_node()->get_parameter("ki_roll").as_double();
  kd_roll_ = get_node()->get_parameter("kd_roll").as_double();
  antiwindup_roll_ = std::abs(get_node()->get_parameter("antiwindup_roll").as_double());

  kp_pitch_ = get_node()->get_parameter("kp_pitch").as_double();
  ki_pitch_ = get_node()->get_parameter("ki_pitch").as_double();
  kd_pitch_ = get_node()->get_parameter("kd_pitch").as_double();
  antiwindup_pitch_ = std::abs(get_node()->get_parameter("antiwindup_pitch").as_double());

  kp_yaw_ = get_node()->get_parameter("kp_yaw").as_double();
  ki_yaw_ = get_node()->get_parameter("ki_yaw").as_double();
  kd_yaw_ = get_node()->get_parameter("kd_yaw").as_double();
  antiwindup_yaw_ = std::abs(get_node()->get_parameter("antiwindup_yaw").as_double());

  setpoint_sub_ = get_node()->create_subscription<TwistMsg>(
    setpoint_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const TwistMsg::SharedPtr msg)
    {
      setpoint_buffer_.writeFromNonRT(msg);
    });

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
    });

  feedforward_pub_ = get_node()->create_publisher<WrenchMsg>(
    feedforward_topic_,
    rclcpp::SystemDefaultsQoS());
  feedforward_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<WrenchMsg>>(feedforward_pub_);
  output_pub_ = get_node()->create_publisher<WrenchMsg>(
    output_topic_,
    rclcpp::SystemDefaultsQoS());
  output_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<WrenchMsg>>(output_pub_);
  pid_terms_pub_ = get_node()->create_publisher<Float64MultiArrayMsg>(
    pid_terms_topic_,
    rclcpp::SystemDefaultsQoS());
  pid_terms_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<Float64MultiArrayMsg>>(pid_terms_pub_);
  pid_terms_rt_pub_->msg_.data.resize(18, 0.0);
  debug_pub_.reset();
  debug_timer_.reset();
  resetDebugStats();

  if (debug_enabled_) {
    debug_pub_ =
      get_node()->create_publisher<sura_msgs::msg::ControllerDebug>(debug_topic_, 10);
    debug_timer_ = get_node()->create_wall_timer(
      std::chrono::seconds(1),
      [this]()
      {
        publishDebugStats();
      });
  }

  reference_interface_names_ = {
    "linear.x",
    "linear.y",
    "linear.z",
    "angular.x",
    "angular.y",
    "angular.z"
  };

  reference_interfaces_.resize(
    reference_interface_names_.size(),
    std::numeric_limits<double>::quiet_NaN());

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
    std::bind(&BodyVelocityController::parametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Configured BodyVelocityController");
  RCLCPP_INFO(get_node()->get_logger(), "setpoint topic: %s", setpoint_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "navigator topic: %s", navigator_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "feedforward topic: %s", feedforward_topic_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(),
    "body_force_controller_name: %s",
    body_force_controller_name_.c_str());
  logGains("Body velocity gains loaded");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyVelocityController::on_activate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = true;
  x_pid_ = AxisPidState{};
  y_pid_ = AxisPidState{};
  z_pid_ = AxisPidState{};
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};

  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  resetDebugStats();

  logGains("Body velocity controller activated with gains");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyVelocityController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = false;
  x_pid_ = AxisPidState{};
  y_pid_ = AxisPidState{};
  z_pid_ = AxisPidState{};
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};

  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  if (feedforward_rt_pub_ && feedforward_rt_pub_->trylock()) {
    feedforward_rt_pub_->msg_ = WrenchMsg{};
    feedforward_rt_pub_->unlockAndPublish();
  }

  if (debug_enabled_ && debug_pub_) {
    publishDebugStats();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult BodyVelocityController::parametersCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : params) {
    const auto & name = param.get_name();

    if (name == "kp_x") {
      kp_x_ = param.as_double();
    } else if (name == "ki_x") {
      ki_x_ = param.as_double();
      x_pid_.integral = 0.0;
    } else if (name == "kd_x") {
      kd_x_ = param.as_double();
    } else if (name == "antiwindup_x") {
      antiwindup_x_ = std::abs(param.as_double());
    } else if (name == "kp_y") {
      kp_y_ = param.as_double();
    } else if (name == "ki_y") {
      ki_y_ = param.as_double();
      y_pid_.integral = 0.0;
    } else if (name == "kd_y") {
      kd_y_ = param.as_double();
    } else if (name == "antiwindup_y") {
      antiwindup_y_ = std::abs(param.as_double());
    } else if (name == "kp_z") {
      kp_z_ = param.as_double();
    } else if (name == "ki_z") {
      ki_z_ = param.as_double();
      z_pid_.integral = 0.0;
    } else if (name == "kd_z") {
      kd_z_ = param.as_double();
    } else if (name == "antiwindup_z") {
      antiwindup_z_ = std::abs(param.as_double());
    } else if (name == "kp_roll") {
      kp_roll_ = param.as_double();
    } else if (name == "ki_roll") {
      ki_roll_ = param.as_double();
      roll_pid_.integral = 0.0;
    } else if (name == "kd_roll") {
      kd_roll_ = param.as_double();
    } else if (name == "antiwindup_roll") {
      antiwindup_roll_ = std::abs(param.as_double());
    } else if (name == "kp_pitch") {
      kp_pitch_ = param.as_double();
    } else if (name == "ki_pitch") {
      ki_pitch_ = param.as_double();
      pitch_pid_.integral = 0.0;
    } else if (name == "kd_pitch") {
      kd_pitch_ = param.as_double();
    } else if (name == "antiwindup_pitch") {
      antiwindup_pitch_ = std::abs(param.as_double());
    } else if (name == "kp_yaw") {
      kp_yaw_ = param.as_double();
    } else if (name == "ki_yaw") {
      ki_yaw_ = param.as_double();
      yaw_pid_.integral = 0.0;
    } else if (name == "kd_yaw") {
      kd_yaw_ = param.as_double();
    } else if (name == "antiwindup_yaw") {
      antiwindup_yaw_ = std::abs(param.as_double());
    }
  }

  logGains("Body velocity gains updated");

  return result;
}

double BodyVelocityController::computePid(
  double error,
  double measured_acceleration,
  double dt,
  double kp,
  double ki,
  double kd,
  double antiwindup,
  AxisPidState & state)
{
  state.integral += error * dt;
  if (antiwindup > 0.0) {
    state.integral = std::clamp(state.integral, -antiwindup, antiwindup);
  }
  const double derivative = -measured_acceleration;
  return kp * error + ki * state.integral + kd * derivative;
}

BodyVelocityController::PidTerms BodyVelocityController::computePidTerms(
  double error,
  double measured_acceleration,
  double dt,
  double kp,
  double ki,
  double kd,
  double antiwindup,
  AxisPidState & state)
{
  state.integral += error * dt;
  if (antiwindup > 0.0) {
    state.integral = std::clamp(state.integral, -antiwindup, antiwindup);
  }

  const double derivative = -measured_acceleration;
  return {
    kp * error,
    ki * state.integral,
    kd * derivative};
}

void BodyVelocityController::publishTelemetry(
  double force_x,
  double force_y,
  double force_z,
  double torque_x,
  double torque_y,
  double torque_z,
  const std::array<PidTerms, 6> & pid_terms)
{
  if (output_rt_pub_ && output_rt_pub_->trylock()) {
    output_rt_pub_->msg_.force.x = force_x;
    output_rt_pub_->msg_.force.y = force_y;
    output_rt_pub_->msg_.force.z = force_z;
    output_rt_pub_->msg_.torque.x = torque_x;
    output_rt_pub_->msg_.torque.y = torque_y;
    output_rt_pub_->msg_.torque.z = torque_z;
    output_rt_pub_->unlockAndPublish();
  }

  if (pid_terms_rt_pub_ && pid_terms_rt_pub_->trylock()) {
    auto & data = pid_terms_rt_pub_->msg_.data;
    for (size_t axis = 0; axis < pid_terms.size(); ++axis) {
      const size_t offset = axis * 3;
      data[offset] = pid_terms[axis].proportional;
      data[offset + 1] = pid_terms[axis].integral;
      data[offset + 2] = pid_terms[axis].derivative;
    }
    pid_terms_rt_pub_->unlockAndPublish();
  }
}

void BodyVelocityController::logGains(const std::string & context) const
{
  RCLCPP_INFO(
    get_node()->get_logger(),
    "%s: x(kp=%.3f ki=%.3f kd=%.3f aw=%.3f) y(kp=%.3f ki=%.3f kd=%.3f aw=%.3f) z(kp=%.3f ki=%.3f kd=%.3f aw=%.3f) roll(kp=%.3f ki=%.3f kd=%.3f aw=%.3f) pitch(kp=%.3f ki=%.3f kd=%.3f aw=%.3f) yaw(kp=%.3f ki=%.3f kd=%.3f aw=%.3f)",
    context.c_str(),
    kp_x_,
    ki_x_,
    kd_x_,
    antiwindup_x_,
    kp_y_,
    ki_y_,
    kd_y_,
    antiwindup_y_,
    kp_z_,
    ki_z_,
    kd_z_,
    antiwindup_z_,
    kp_roll_,
    ki_roll_,
    kd_roll_,
    antiwindup_roll_,
    kp_pitch_,
    ki_pitch_,
    kd_pitch_,
    antiwindup_pitch_,
    kp_yaw_,
    ki_yaw_,
    kd_yaw_,
    antiwindup_yaw_);
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
    std::fill(
      reference_interfaces_.begin(),
      reference_interfaces_.end(),
      std::numeric_limits<double>::quiet_NaN());
  }

  return true;
}

controller_interface::return_type BodyVelocityController::update_reference_from_subscribers()
{
  auto setpoint_msg = setpoint_buffer_.readFromRT();

  if (!setpoint_msg || !(*setpoint_msg)) {
    return controller_interface::return_type::OK;
  }

  reference_interfaces_[0] = (*setpoint_msg)->linear.x;
  reference_interfaces_[1] = (*setpoint_msg)->linear.y;
  reference_interfaces_[2] = (*setpoint_msg)->linear.z;
  reference_interfaces_[3] = (*setpoint_msg)->angular.x;
  reference_interfaces_[4] = (*setpoint_msg)->angular.y;
  reference_interfaces_[5] = (*setpoint_msg)->angular.z;

  return controller_interface::return_type::OK;
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

  const double dt = period.seconds();

  const double setpoint_linear_x =
    std::isnan(reference_interfaces_[0]) ? 0.0 : reference_interfaces_[0];
  const double setpoint_linear_y =
    std::isnan(reference_interfaces_[1]) ? 0.0 : reference_interfaces_[1];
  const double setpoint_linear_z =
    std::isnan(reference_interfaces_[2]) ? 0.0 : reference_interfaces_[2];
  const double setpoint_angular_x =
    std::isnan(reference_interfaces_[3]) ? 0.0 : reference_interfaces_[3];
  const double setpoint_angular_y =
    std::isnan(reference_interfaces_[4]) ? 0.0 : reference_interfaces_[4];
  const double setpoint_angular_z =
    std::isnan(reference_interfaces_[5]) ? 0.0 : reference_interfaces_[5];

  std::array<PidTerms, 6> pid_terms;

  pid_terms[0] = computePidTerms(
    setpoint_linear_x - (*navigator_msg)->body_velocity.linear.x,
    (*navigator_msg)->body_acceleration.linear.x,
    dt,
    kp_x_,
    ki_x_,
    kd_x_,
    antiwindup_x_,
    x_pid_);
  const double force_x =
    pid_terms[0].proportional + pid_terms[0].integral + pid_terms[0].derivative;

  pid_terms[1] = computePidTerms(
    setpoint_linear_y - (*navigator_msg)->body_velocity.linear.y,
    (*navigator_msg)->body_acceleration.linear.y,
    dt,
    kp_y_,
    ki_y_,
    kd_y_,
    antiwindup_y_,
    y_pid_);
  const double force_y =
    pid_terms[1].proportional + pid_terms[1].integral + pid_terms[1].derivative;

  pid_terms[2] = computePidTerms(
    setpoint_linear_z - (*navigator_msg)->body_velocity.linear.z,
    (*navigator_msg)->body_acceleration.linear.z,
    dt,
    kp_z_,
    ki_z_,
    kd_z_,
    antiwindup_z_,
    z_pid_);
  const double force_z =
    pid_terms[2].proportional + pid_terms[2].integral + pid_terms[2].derivative;

  pid_terms[3] = computePidTerms(
    setpoint_angular_x - (*navigator_msg)->body_velocity.angular.x,
    (*navigator_msg)->body_acceleration.angular.x,
    dt,
    kp_roll_,
    ki_roll_,
    kd_roll_,
    antiwindup_roll_,
    roll_pid_);
  const double torque_x =
    pid_terms[3].proportional + pid_terms[3].integral + pid_terms[3].derivative;

  pid_terms[4] = computePidTerms(
    setpoint_angular_y - (*navigator_msg)->body_velocity.angular.y,
    (*navigator_msg)->body_acceleration.angular.y,
    dt,
    kp_pitch_,
    ki_pitch_,
    kd_pitch_,
    antiwindup_pitch_,
    pitch_pid_);
  const double torque_y =
    pid_terms[4].proportional + pid_terms[4].integral + pid_terms[4].derivative;

  pid_terms[5] = computePidTerms(
    setpoint_angular_z - (*navigator_msg)->body_velocity.angular.z,
    (*navigator_msg)->body_acceleration.angular.z,
    dt,
    kp_yaw_,
    ki_yaw_,
    kd_yaw_,
    antiwindup_yaw_,
    yaw_pid_);
  const double torque_z =
    pid_terms[5].proportional + pid_terms[5].integral + pid_terms[5].derivative;

  if (feedforward_rt_pub_ && feedforward_rt_pub_->trylock()) {
    feedforward_rt_pub_->msg_.force.x = force_x;
    feedforward_rt_pub_->msg_.force.y = force_y;
    feedforward_rt_pub_->msg_.force.z = force_z;
    feedforward_rt_pub_->msg_.torque.x = torque_x;
    feedforward_rt_pub_->msg_.torque.y = torque_y;
    feedforward_rt_pub_->msg_.torque.z = torque_z;
    feedforward_rt_pub_->unlockAndPublish();
  }

  publishTelemetry(force_x, force_y, force_z, torque_x, torque_y, torque_z, pid_terms);

  if (debug_enabled_) {
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

  return controller_interface::return_type::OK;
}

}  // namespace sura_controllers::auv

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::auv::BodyVelocityController,
  controller_interface::ChainableControllerInterface)
