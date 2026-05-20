#include "sura_controllers/auv/stabilize_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace sura_controllers::auv
{

void StabilizeController::resetDebugStats()
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

void StabilizeController::publishDebugStats()
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

controller_interface::CallbackReturn StabilizeController::on_init()
{
  try {
    auto_declare<std::string>(
      "feedforward_topic", "/cirtesub/controller/stabilize/feedforward");
    auto_declare<std::string>(
      "navigator_topic", "/cirtesub/navigator/navigation");
    auto_declare<std::string>(
      "setpoint_topic", "/cirtesub/controller/stabilize/set_point");
    auto_declare<std::string>(
      "enable_roll_pitch_service_name", "/cirtesub/controller/stabilize/enable_roll_pitch");
    auto_declare<std::string>(
      "disable_roll_pitch_service_name", "/cirtesub/controller/stabilize/disable_roll_pitch");
    auto_declare<std::string>(
      "body_force_controller_name", "body_force");

    auto_declare<bool>("allow_roll_pitch", false);
    auto_declare<double>("feedforward_gain_x", 1.0);
    auto_declare<double>("feedforward_gain_y", 1.0);
    auto_declare<double>("feedforward_gain_z", 1.0);
    auto_declare<double>("feedforward_gain_roll", 1.0);
    auto_declare<double>("feedforward_gain_pitch", 1.0);
    auto_declare<double>("feedforward_gain_yaw", 1.0);
    auto_declare<double>("roll_command_threshold", 1e-3);
    auto_declare<double>("pitch_command_threshold", 1e-3);
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

    auto_declare<double>("yaw_command_threshold", 1e-3);
    auto_declare<bool>("debug.enabled", false);
    auto_declare<std::string>("debug.topic", "debug");
  } catch (const std::exception &) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
StabilizeController::command_interface_configuration() const
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
StabilizeController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::CallbackReturn StabilizeController::on_configure(
  const rclcpp_lifecycle::State &)
{
  feedforward_topic_ = get_node()->get_parameter("feedforward_topic").as_string();
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  setpoint_topic_ = get_node()->get_parameter("setpoint_topic").as_string();
  enable_roll_pitch_service_name_ =
    get_node()->get_parameter("enable_roll_pitch_service_name").as_string();
  disable_roll_pitch_service_name_ =
    get_node()->get_parameter("disable_roll_pitch_service_name").as_string();
  body_force_controller_name_ =
    get_node()->get_parameter("body_force_controller_name").as_string();
  debug_enabled_ = get_node()->get_parameter("debug.enabled").as_bool();
  debug_topic_ = get_node()->get_parameter("debug.topic").as_string();
  allow_roll_pitch_ = get_node()->get_parameter("allow_roll_pitch").as_bool();
  feedforward_gain_x_ = get_node()->get_parameter("feedforward_gain_x").as_double();
  feedforward_gain_y_ = get_node()->get_parameter("feedforward_gain_y").as_double();
  feedforward_gain_z_ = get_node()->get_parameter("feedforward_gain_z").as_double();
  feedforward_gain_roll_ = get_node()->get_parameter("feedforward_gain_roll").as_double();
  feedforward_gain_pitch_ = get_node()->get_parameter("feedforward_gain_pitch").as_double();
  feedforward_gain_yaw_ = get_node()->get_parameter("feedforward_gain_yaw").as_double();
  roll_command_threshold_ =
    std::max(0.0, get_node()->get_parameter("roll_command_threshold").as_double());
  pitch_command_threshold_ =
    std::max(0.0, get_node()->get_parameter("pitch_command_threshold").as_double());

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

  yaw_command_threshold_ =
    std::max(0.0, get_node()->get_parameter("yaw_command_threshold").as_double());

  feedforward_sub_ = get_node()->create_subscription<WrenchMsg>(
    feedforward_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const WrenchMsg::SharedPtr msg)
    {
      feedforward_buffer_.writeFromNonRT(msg);
    });

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
    });

  enable_roll_pitch_srv_ = get_node()->create_service<TriggerSrv>(
    enable_roll_pitch_service_name_,
    [this](
      const std::shared_ptr<TriggerSrv::Request>,
      std::shared_ptr<TriggerSrv::Response> response)
    {
      setRollPitchEnabled(true, true);
      response->success = true;
      response->message = "Roll and pitch setpoints will be zeroed and unlocked in update.";
    });

  disable_roll_pitch_srv_ = get_node()->create_service<TriggerSrv>(
    disable_roll_pitch_service_name_,
    [this](
      const std::shared_ptr<TriggerSrv::Request>,
      std::shared_ptr<TriggerSrv::Response> response)
    {
      setRollPitchEnabled(false, true);
      response->success = true;
      response->message = "Roll and pitch setpoints will be zeroed and locked in update.";
    });

  setpoint_pub_ = get_node()->create_publisher<Vector3Msg>(
    setpoint_topic_,
    rclcpp::SystemDefaultsQoS());
  setpoint_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<Vector3Msg>>(setpoint_pub_);
  output_pub_ = get_node()->create_publisher<WrenchMsg>(
    "/cirtesub/controller/stabilize/output",
    rclcpp::SystemDefaultsQoS());
  output_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<WrenchMsg>>(output_pub_);
  pid_terms_pub_ = get_node()->create_publisher<Float64MultiArrayMsg>(
    "/cirtesub/controller/stabilize/pid_terms",
    rclcpp::SystemDefaultsQoS());
  pid_terms_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<Float64MultiArrayMsg>>(pid_terms_pub_);
  pid_terms_rt_pub_->msg_.data.resize(9, 0.0);
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
  allow_roll_pitch_buffer_.writeFromNonRT(allow_roll_pitch_);
  reference_interface_names_ = {
    "force.x",
    "force.y",
    "force.z",
    "torque.x",
    "torque.y",
    "torque.z"
  };
  reference_interfaces_.resize(
    reference_interface_names_.size(),
    std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Stabilize feedforward gains loaded: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
    feedforward_gain_x_,
    feedforward_gain_y_,
    feedforward_gain_z_,
    feedforward_gain_roll_,
    feedforward_gain_pitch_,
    feedforward_gain_yaw_);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Stabilize PID gains loaded: roll(kp=%.3f ki=%.3f kd=%.3f) pitch(kp=%.3f ki=%.3f kd=%.3f) yaw(kp=%.3f ki=%.3f kd=%.3f)",
    kp_roll_,
    ki_roll_,
    kd_roll_,
    kp_pitch_,
    ki_pitch_,
    kd_pitch_,
    kp_yaw_,
    ki_yaw_,
    kd_yaw_);

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
    std::bind(&StabilizeController::parametersCallback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StabilizeController::on_activate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = true;
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};
  roll_setpoint_ = 0.0;
  pitch_setpoint_ = 0.0;
  yaw_setpoint_ = 0.0;
  roll_setpoint_initialized_ = false;
  pitch_setpoint_initialized_ = false;
  yaw_setpoint_initialized_ = false;
  roll_feedforward_active_ = false;
  pitch_feedforward_active_ = false;
  yaw_feedforward_active_ = false;
  zero_roll_pitch_requested_.store(false);
  allow_roll_pitch_buffer_.writeFromNonRT(allow_roll_pitch_);
  first_update_ = true;
  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  resetDebugStats();

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Stabilize controller activated with feedforward gains: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
    feedforward_gain_x_,
    feedforward_gain_y_,
    feedforward_gain_z_,
    feedforward_gain_roll_,
    feedforward_gain_pitch_,
    feedforward_gain_yaw_);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Stabilize PID gains loaded: roll(kp=%.3f ki=%.3f kd=%.3f) pitch(kp=%.3f ki=%.3f kd=%.3f) yaw(kp=%.3f ki=%.3f kd=%.3f)",
    kp_roll_,
    ki_roll_,
    kd_roll_,
    kp_pitch_,
    ki_pitch_,
    kd_pitch_,
    kp_yaw_,
    ki_yaw_,
    kd_yaw_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StabilizeController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = false;
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};
  roll_setpoint_ = 0.0;
  pitch_setpoint_ = 0.0;
  yaw_setpoint_ = 0.0;
  roll_setpoint_initialized_ = false;
  pitch_setpoint_initialized_ = false;
  yaw_setpoint_initialized_ = false;
  roll_feedforward_active_ = false;
  pitch_feedforward_active_ = false;
  yaw_feedforward_active_ = false;
  zero_roll_pitch_requested_.store(false);
  allow_roll_pitch_buffer_.writeFromNonRT(allow_roll_pitch_);
  first_update_ = true;
  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  if (debug_enabled_ && debug_pub_) {
    publishDebugStats();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult StabilizeController::parametersCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  bool feedforward_gains_updated = false;

  for (const auto & param : params) {
    const auto & name = param.get_name();

    if (name == "kp_roll") {
      kp_roll_ = param.as_double();
    } else if (name == "allow_roll_pitch") {
      setRollPitchEnabled(param.as_bool(), !param.as_bool());
    } else if (name == "feedforward_gain_x") {
      feedforward_gain_x_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "feedforward_gain_y") {
      feedforward_gain_y_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "feedforward_gain_z") {
      feedforward_gain_z_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "feedforward_gain_roll") {
      feedforward_gain_roll_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "feedforward_gain_pitch") {
      feedforward_gain_pitch_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "feedforward_gain_yaw") {
      feedforward_gain_yaw_ = param.as_double();
      feedforward_gains_updated = true;
    } else if (name == "roll_command_threshold") {
      roll_command_threshold_ = std::max(0.0, param.as_double());
    } else if (name == "pitch_command_threshold") {
      pitch_command_threshold_ = std::max(0.0, param.as_double());
    } else if (name == "ki_roll") {
      ki_roll_ = param.as_double();
      roll_pid_.integral = 0.0;
    } else if (name == "antiwindup_roll") {
      antiwindup_roll_ = std::abs(param.as_double());
    } else if (name == "kd_roll") {
      kd_roll_ = param.as_double();
    } else if (name == "kp_pitch") {
      kp_pitch_ = param.as_double();
    } else if (name == "ki_pitch") {
      ki_pitch_ = param.as_double();
      pitch_pid_.integral = 0.0;
    } else if (name == "antiwindup_pitch") {
      antiwindup_pitch_ = std::abs(param.as_double());
    } else if (name == "kd_pitch") {
      kd_pitch_ = param.as_double();
    } else if (name == "kp_yaw") {
      kp_yaw_ = param.as_double();
    } else if (name == "ki_yaw") {
      ki_yaw_ = param.as_double();
      yaw_pid_.integral = 0.0;
    } else if (name == "antiwindup_yaw") {
      antiwindup_yaw_ = std::abs(param.as_double());
    } else if (name == "kd_yaw") {
      kd_yaw_ = param.as_double();
    } else if (name == "yaw_command_threshold") {
      yaw_command_threshold_ = std::max(0.0, param.as_double());
    }
  }

  if (feedforward_gains_updated) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Stabilize feedforward gains updated: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
      feedforward_gain_x_,
      feedforward_gain_y_,
      feedforward_gain_z_,
      feedforward_gain_roll_,
      feedforward_gain_pitch_,
      feedforward_gain_yaw_);
  }

  return result;
}

double StabilizeController::computePid(
  double error,
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

  double derivative = 0.0;
  if (!first_update_ && dt > 0.0) {
    derivative = (error - state.previous_error) / dt;
  }

  state.previous_error = error;

  return kp * error + ki * state.integral + kd * derivative;
}

StabilizeController::PidTerms StabilizeController::computePidTerms(
  double error,
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

  double derivative = 0.0;
  if (!first_update_ && dt > 0.0) {
    derivative = (error - state.previous_error) / dt;
  }

  state.previous_error = error;

  return {
    kp * error,
    ki * state.integral,
    kd * derivative};
}

StabilizeController::PidTerms StabilizeController::computePidTermsWithMeasuredRate(
  double error,
  double measured_rate,
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

  state.previous_error = error;

  return {
    kp * error,
    ki * state.integral,
    kd * -measured_rate};
}

double StabilizeController::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void StabilizeController::setRollPitchEnabled(bool enabled, bool request_zero_setpoint)
{
  allow_roll_pitch_ = enabled;
  allow_roll_pitch_buffer_.writeFromNonRT(enabled);
  if (request_zero_setpoint) {
    zero_roll_pitch_requested_.store(true);
  }
}

void StabilizeController::publishSetpoint()
{
  if (setpoint_rt_pub_ && setpoint_rt_pub_->trylock()) {
    setpoint_rt_pub_->msg_.x = roll_setpoint_;
    setpoint_rt_pub_->msg_.y = pitch_setpoint_;
    setpoint_rt_pub_->msg_.z = yaw_setpoint_;
    setpoint_rt_pub_->unlockAndPublish();
  }
}

void StabilizeController::publishTelemetry(
  double force_x,
  double force_y,
  double force_z,
  double torque_x,
  double torque_y,
  double torque_z,
  const std::array<PidTerms, 3> & pid_terms)
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

std::vector<hardware_interface::CommandInterface>
StabilizeController::on_export_reference_interfaces()
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

bool StabilizeController::on_set_chained_mode(bool chained_mode)
{
  chained_mode_ = chained_mode;
  if (chained_mode) {
    RCLCPP_INFO(get_node()->get_logger(), "StabilizeController switched to chained mode");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "StabilizeController switched to topic mode");
    std::fill(
      reference_interfaces_.begin(),
      reference_interfaces_.end(),
      std::numeric_limits<double>::quiet_NaN());
  }

  return true;
}

controller_interface::return_type StabilizeController::update_reference_from_subscribers()
{
  auto feedforward_msg = feedforward_buffer_.readFromRT();

  if (!feedforward_msg || !(*feedforward_msg)) {
    return controller_interface::return_type::OK;
  }

  reference_interfaces_[0] = (*feedforward_msg)->force.x;
  reference_interfaces_[1] = (*feedforward_msg)->force.y;
  reference_interfaces_[2] = (*feedforward_msg)->force.z;
  reference_interfaces_[3] = (*feedforward_msg)->torque.x;
  reference_interfaces_[4] = (*feedforward_msg)->torque.y;
  reference_interfaces_[5] = (*feedforward_msg)->torque.z;

  return controller_interface::return_type::OK;
}

controller_interface::return_type StabilizeController::update_and_write_commands(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const auto update_start = debug_enabled_ ? std::chrono::steady_clock::now() :
    std::chrono::steady_clock::time_point{};
  auto navigator_msg = navigator_buffer_.readFromRT();

  if (!navigator_msg || !(*navigator_msg)) {
    return controller_interface::return_type::OK;
  }

  if (command_interfaces_.size() != 6) {
    return controller_interface::return_type::ERROR;
  }

  const auto & orientation = (*navigator_msg)->position.orientation;
  tf2::Quaternion q(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  if (
    !roll_setpoint_initialized_ ||
    !pitch_setpoint_initialized_ ||
    !yaw_setpoint_initialized_)
  {
    roll_setpoint_ = roll;
    pitch_setpoint_ = pitch;
    yaw_setpoint_ = yaw;
    roll_setpoint_initialized_ = true;
    pitch_setpoint_initialized_ = true;
    yaw_setpoint_initialized_ = true;
  }

  const double force_x =
    std::isnan(reference_interfaces_[0]) ? 0.0 : reference_interfaces_[0] * feedforward_gain_x_;
  const double force_y =
    std::isnan(reference_interfaces_[1]) ? 0.0 : reference_interfaces_[1] * feedforward_gain_y_;
  const double force_z =
    std::isnan(reference_interfaces_[2]) ? 0.0 : reference_interfaces_[2] * feedforward_gain_z_;
  const double torque_ff_x =
    std::isnan(reference_interfaces_[3]) ? 0.0 : reference_interfaces_[3] * feedforward_gain_roll_;
  const double torque_ff_y =
    std::isnan(reference_interfaces_[4]) ? 0.0 : reference_interfaces_[4] * feedforward_gain_pitch_;
  const double yaw_feedforward =
    std::isnan(reference_interfaces_[5]) ? 0.0 : reference_interfaces_[5] * feedforward_gain_yaw_;
  const bool * allow_roll_pitch_ptr = allow_roll_pitch_buffer_.readFromRT();
  const bool allow_roll_pitch = allow_roll_pitch_ptr ? *allow_roll_pitch_ptr : false;
  const bool zero_roll_pitch_requested =
    zero_roll_pitch_requested_.exchange(false);
  const bool has_force_feedforward =
    std::abs(force_x) > yaw_command_threshold_ ||
    std::abs(force_y) > yaw_command_threshold_ ||
    std::abs(force_z) > yaw_command_threshold_;
  const bool has_roll_feedforward = std::abs(torque_ff_x) > roll_command_threshold_;
  const bool has_pitch_feedforward = std::abs(torque_ff_y) > pitch_command_threshold_;
  const bool has_yaw_feedforward = std::abs(yaw_feedforward) > yaw_command_threshold_;
  const bool has_input_command =
    has_force_feedforward ||
    has_roll_feedforward ||
    has_pitch_feedforward ||
    has_yaw_feedforward;
  if (zero_roll_pitch_requested || !allow_roll_pitch) {
    roll_setpoint_ = 0.0;
    pitch_setpoint_ = 0.0;
  }

  if (allow_roll_pitch && !zero_roll_pitch_requested && has_roll_feedforward) {
    roll_setpoint_ = roll;
    roll_pid_.integral = 0.0;
  } else if (allow_roll_pitch && !zero_roll_pitch_requested && roll_feedforward_active_) {
    roll_setpoint_ = roll;
    roll_pid_.integral = 0.0;
  }

  if (allow_roll_pitch && !zero_roll_pitch_requested && has_pitch_feedforward) {
    pitch_setpoint_ = pitch;
    pitch_pid_.integral = 0.0;
  } else if (allow_roll_pitch && !zero_roll_pitch_requested && pitch_feedforward_active_) {
    pitch_setpoint_ = pitch;
    pitch_pid_.integral = 0.0;
  }

  if (has_yaw_feedforward) {
    yaw_setpoint_ = yaw;
    yaw_pid_.integral = 0.0;
  } else if (yaw_feedforward_active_) {
    yaw_setpoint_ = yaw;
    yaw_pid_.integral = 0.0;
  }

  if (has_input_command) {
    publishSetpoint();
  }

  const double dt = period.seconds();

  const double roll_error = wrapAngle(roll_setpoint_ - roll);
  const double pitch_error = wrapAngle(pitch_setpoint_ - pitch);
  const double yaw_error = wrapAngle(yaw_setpoint_ - yaw);
  const auto & angular_velocity = (*navigator_msg)->body_velocity.angular;

  std::array<PidTerms, 3> pid_terms;
  pid_terms[0] = computePidTermsWithMeasuredRate(
    roll_error, angular_velocity.x, dt, kp_roll_, ki_roll_, kd_roll_, antiwindup_roll_, roll_pid_);
  pid_terms[1] = computePidTermsWithMeasuredRate(
    pitch_error, angular_velocity.y, dt, kp_pitch_, ki_pitch_, kd_pitch_, antiwindup_pitch_,
    pitch_pid_);
  pid_terms[2] = computePidTermsWithMeasuredRate(
    yaw_error, angular_velocity.z, dt, kp_yaw_, ki_yaw_, kd_yaw_, antiwindup_yaw_, yaw_pid_);

  const double torque_x =
    torque_ff_x +
    pid_terms[0].proportional + pid_terms[0].integral + pid_terms[0].derivative;

  const double torque_y =
    torque_ff_y +
    pid_terms[1].proportional + pid_terms[1].integral + pid_terms[1].derivative;

  const double torque_z =
    yaw_feedforward +
    pid_terms[2].proportional + pid_terms[2].integral + pid_terms[2].derivative;

  command_interfaces_[0].set_value(force_x);
  command_interfaces_[1].set_value(force_y);
  command_interfaces_[2].set_value(force_z);
  command_interfaces_[3].set_value(torque_x);
  command_interfaces_[4].set_value(torque_y);
  command_interfaces_[5].set_value(torque_z);

  publishTelemetry(force_x, force_y, force_z, torque_x, torque_y, torque_z, pid_terms);

  roll_feedforward_active_ = allow_roll_pitch && has_roll_feedforward;
  pitch_feedforward_active_ = allow_roll_pitch && has_pitch_feedforward;
  yaw_feedforward_active_ = has_yaw_feedforward;
  first_update_ = false;

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
  sura_controllers::auv::StabilizeController,
  controller_interface::ChainableControllerInterface)
