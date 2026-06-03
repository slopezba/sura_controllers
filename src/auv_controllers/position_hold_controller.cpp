#include "sura_controllers/auv/position_hold_controller.hpp"

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
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"

namespace sura_controllers::auv
{

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

controller_interface::CallbackReturn PositionHoldController::on_init()
{
  try {
    auto_declare<std::string>("setpoint_topic", "/cirtesub/controller/position_hold/setpoint");
    auto_declare<std::string>("feedforward_topic", "/cirtesub/controller/position_hold/feedforward");
    auto_declare<std::string>("navigator_topic", "/cirtesub/navigator/navigation");
    auto_declare<std::string>("output_topic", "/cirtesub/controller/position_hold/output");
    auto_declare<std::string>("pid_terms_topic", "/cirtesub/controller/position_hold/pid_terms");
    auto_declare<std::string>("body_velocity_controller_name", "body_velocity");
    auto_declare<std::string>("setpoint_frame_id", "world_ned");

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
  const std::string prefix = body_velocity_controller_name_;

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      prefix + "/linear.x",
      prefix + "/linear.y",
      prefix + "/linear.z",
      prefix + "/angular.x",
      prefix + "/angular.y",
      prefix + "/angular.z"
    }
  };
}

controller_interface::InterfaceConfiguration
PositionHoldController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::CallbackReturn PositionHoldController::on_configure(
  const rclcpp_lifecycle::State &)
{
  setpoint_topic_ = get_node()->get_parameter("setpoint_topic").as_string();
  feedforward_topic_ = get_node()->get_parameter("feedforward_topic").as_string();
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  output_topic_ = get_node()->get_parameter("output_topic").as_string();
  pid_terms_topic_ = get_node()->get_parameter("pid_terms_topic").as_string();
  body_velocity_controller_name_ =
    get_node()->get_parameter("body_velocity_controller_name").as_string();
  setpoint_frame_id_ = get_node()->get_parameter("setpoint_frame_id").as_string();
  debug_enabled_ = get_node()->get_parameter("debug.enabled").as_bool();
  debug_topic_ = get_node()->get_parameter("debug.topic").as_string();

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

  linear_feedforward_threshold_ = std::max(
    0.0,
    get_node()->get_parameter("linear_feedforward_threshold").as_double());
  angular_feedforward_threshold_ = std::max(
    0.0,
    get_node()->get_parameter("angular_feedforward_threshold").as_double());
  feedforward_timeout_ = std::max(0.0, get_node()->get_parameter("feedforward_timeout").as_double());

  rclcpp::SubscriptionOptions setpoint_subscription_options;
  setpoint_subscription_options.ignore_local_publications = true;

  setpoint_sub_ = get_node()->create_subscription<PoseStampedMsg>(
    setpoint_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const PoseStampedMsg::SharedPtr msg)
    {
      setpoint_buffer_.writeFromNonRT(msg);
      new_setpoint_requested_.store(true);
    },
    setpoint_subscription_options);

  feedforward_sub_ = get_node()->create_subscription<TwistMsg>(
    feedforward_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const TwistMsg::SharedPtr msg)
    {
      feedforward_buffer_.writeFromNonRT(msg);
      last_feedforward_time_ns_.store(get_node()->now().nanoseconds());
    });

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
    });

  setpoint_pub_ = get_node()->create_publisher<PoseStampedMsg>(
    setpoint_topic_,
    rclcpp::SystemDefaultsQoS());
  setpoint_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<PoseStampedMsg>>(setpoint_pub_);
  output_pub_ = get_node()->create_publisher<TwistMsg>(
    output_topic_,
    rclcpp::SystemDefaultsQoS());
  output_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<TwistMsg>>(output_pub_);
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
    "position.x",
    "position.y",
    "position.z",
    "orientation.roll",
    "orientation.pitch",
    "orientation.yaw"
  };
  reference_interfaces_.resize(
    reference_interface_names_.size(),
    std::numeric_limits<double>::quiet_NaN());

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
    std::bind(&PositionHoldController::parametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured PositionHoldController with setpoint topic '%s' and feedforward topic '%s'",
    setpoint_topic_.c_str(),
    feedforward_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionHoldController::on_activate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = true;
  resetPidStates();
  current_setpoint_ = PoseStampedMsg{};
  current_feedforward_ = TwistMsg{};
  setpoint_initialized_ = false;
  feedforward_active_ = false;
  new_setpoint_requested_.store(false);
  last_feedforward_time_ns_.store(0);
  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  resetDebugStats();

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionHoldController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = false;
  resetPidStates();
  current_feedforward_ = TwistMsg{};
  setpoint_initialized_ = false;
  feedforward_active_ = false;
  new_setpoint_requested_.store(false);
  last_feedforward_time_ns_.store(0);
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

rcl_interfaces::msg::SetParametersResult PositionHoldController::parametersCallback(
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
    } else if (name == "linear_feedforward_threshold") {
      linear_feedforward_threshold_ = std::max(0.0, param.as_double());
    } else if (name == "angular_feedforward_threshold") {
      angular_feedforward_threshold_ = std::max(0.0, param.as_double());
    } else if (name == "feedforward_timeout") {
      feedforward_timeout_ = std::max(0.0, param.as_double());
    }
  }

  return result;
}

double PositionHoldController::computePid(
  double error,
  double dt,
  double kp,
  double ki,
  double kd,
  double antiwindup,
  AxisPidState & state)
{
  const double safe_dt = dt > 0.0 ? dt : 1e-6;
  state.integral += error * safe_dt;

  if (antiwindup > 0.0) {
    state.integral = std::clamp(state.integral, -antiwindup, antiwindup);
  }

  const double derivative = (error - state.previous_error) / safe_dt;
  state.previous_error = error;

  return (kp * error) + (ki * state.integral) + (kd * derivative);
}

PositionHoldController::PidTerms PositionHoldController::computePidTerms(
  double error,
  double dt,
  double kp,
  double ki,
  double kd,
  double antiwindup,
  AxisPidState & state)
{
  const double safe_dt = dt > 0.0 ? dt : 1e-6;
  state.integral += error * safe_dt;

  if (antiwindup > 0.0) {
    state.integral = std::clamp(state.integral, -antiwindup, antiwindup);
  }

  const double derivative = (error - state.previous_error) / safe_dt;
  state.previous_error = error;

  return {
    kp * error,
    ki * state.integral,
    kd * derivative};
}

PositionHoldController::PidTerms PositionHoldController::computePidTermsWithMeasuredRate(
  double error,
  double measured_rate,
  double dt,
  double kp,
  double ki,
  double kd,
  double antiwindup,
  AxisPidState & state)
{
  const double safe_dt = dt > 0.0 ? dt : 1e-6;
  state.integral += error * safe_dt;

  if (antiwindup > 0.0) {
    state.integral = std::clamp(state.integral, -antiwindup, antiwindup);
  }

  state.previous_error = error;

  return {
    kp * error,
    ki * state.integral,
    kd * -measured_rate};
}

double PositionHoldController::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void PositionHoldController::resetPidStates()
{
  x_pid_ = AxisPidState{};
  y_pid_ = AxisPidState{};
  z_pid_ = AxisPidState{};
  roll_pid_ = AxisPidState{};
  pitch_pid_ = AxisPidState{};
  yaw_pid_ = AxisPidState{};
}

void PositionHoldController::setSetpointFromNavigator(const NavigatorMsg & navigator_msg)
{
  current_setpoint_.pose = navigator_msg.position;
  current_setpoint_.header.frame_id =
    setpoint_frame_id_.empty() ? "blueboat/map" : setpoint_frame_id_;
  setpoint_initialized_ = true;
  updateReferenceInterfacesFromSetpoint();
}

void PositionHoldController::applyExternalSetpoint(const PoseStampedMsg & setpoint_msg)
{
  current_setpoint_ = setpoint_msg;
  if (current_setpoint_.header.frame_id.empty()) {
    current_setpoint_.header.frame_id =
      setpoint_frame_id_.empty() ? "blueboat/map" : setpoint_frame_id_;
  }
  setpoint_initialized_ = true;
  resetPidStates();
  updateReferenceInterfacesFromSetpoint();
}

void PositionHoldController::publishCurrentSetpoint(const rclcpp::Time & stamp)
{
  if (!setpoint_initialized_ || !setpoint_rt_pub_) {
    return;
  }

  if (setpoint_rt_pub_->trylock()) {
    current_setpoint_.header.stamp = stamp;
    setpoint_rt_pub_->msg_ = current_setpoint_;
    setpoint_rt_pub_->unlockAndPublish();
  }
}

void PositionHoldController::publishTelemetry(
  double linear_x,
  double linear_y,
  double linear_z,
  double angular_x,
  double angular_y,
  double angular_z,
  const std::array<PidTerms, 6> & pid_terms)
{
  if (output_rt_pub_ && output_rt_pub_->trylock()) {
    output_rt_pub_->msg_.linear.x = linear_x;
    output_rt_pub_->msg_.linear.y = linear_y;
    output_rt_pub_->msg_.linear.z = linear_z;
    output_rt_pub_->msg_.angular.x = angular_x;
    output_rt_pub_->msg_.angular.y = angular_y;
    output_rt_pub_->msg_.angular.z = angular_z;
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

void PositionHoldController::updateReferenceInterfacesFromSetpoint()
{
  if (reference_interfaces_.size() != 6) {
    return;
  }

  tf2::Quaternion q_setpoint(
    current_setpoint_.pose.orientation.x,
    current_setpoint_.pose.orientation.y,
    current_setpoint_.pose.orientation.z,
    current_setpoint_.pose.orientation.w);
  q_setpoint.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q_setpoint).getRPY(roll, pitch, yaw);

  reference_interfaces_[0] = current_setpoint_.pose.position.x;
  reference_interfaces_[1] = current_setpoint_.pose.position.y;
  reference_interfaces_[2] = current_setpoint_.pose.position.z;
  reference_interfaces_[3] = roll;
  reference_interfaces_[4] = pitch;
  reference_interfaces_[5] = yaw;
}

std::vector<hardware_interface::CommandInterface>
PositionHoldController::on_export_reference_interfaces()
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

bool PositionHoldController::on_set_chained_mode(bool chained_mode)
{
  chained_mode_ = chained_mode;
  if (chained_mode) {
    RCLCPP_INFO(get_node()->get_logger(), "PositionHoldController switched to chained mode");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "PositionHoldController switched to topic mode");
    std::fill(
      reference_interfaces_.begin(),
      reference_interfaces_.end(),
      std::numeric_limits<double>::quiet_NaN());
  }
  return true;
}

controller_interface::return_type PositionHoldController::update_reference_from_subscribers()
{
  auto feedforward_msg = feedforward_buffer_.readFromRT();

  if (!feedforward_msg || !(*feedforward_msg)) {
    current_feedforward_ = TwistMsg{};
    return controller_interface::return_type::OK;
  }

  current_feedforward_ = *(*feedforward_msg);
  return controller_interface::return_type::OK;
}

controller_interface::return_type PositionHoldController::update_and_write_commands(
  const rclcpp::Time & time,
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

  if (!setpoint_initialized_) {
    setSetpointFromNavigator(*(*navigator_msg));
    publishCurrentSetpoint(time);
  }

  auto setpoint_msg = setpoint_buffer_.readFromRT();
  if (new_setpoint_requested_.exchange(false) && setpoint_msg && *setpoint_msg) {
    applyExternalSetpoint(*(*setpoint_msg));
    publishCurrentSetpoint(time);
  }

  const int64_t last_feedforward_time_ns = last_feedforward_time_ns_.load();
  const bool feedforward_is_fresh =
    last_feedforward_time_ns > 0 &&
    ((time.nanoseconds() - last_feedforward_time_ns) * 1e-9) <= feedforward_timeout_;

  TwistMsg effective_feedforward;
  if (feedforward_is_fresh) {
    effective_feedforward = current_feedforward_;
  }

  const bool has_linear_feedforward =
    std::abs(effective_feedforward.linear.x) > linear_feedforward_threshold_ ||
    std::abs(effective_feedforward.linear.y) > linear_feedforward_threshold_ ||
    std::abs(effective_feedforward.linear.z) > linear_feedforward_threshold_;
  const bool has_angular_feedforward =
    std::abs(effective_feedforward.angular.x) > angular_feedforward_threshold_ ||
    std::abs(effective_feedforward.angular.y) > angular_feedforward_threshold_ ||
    std::abs(effective_feedforward.angular.z) > angular_feedforward_threshold_;
  const bool has_feedforward = has_linear_feedforward || has_angular_feedforward;

  if (has_feedforward) {
    setSetpointFromNavigator(*(*navigator_msg));
    publishCurrentSetpoint(time);
    resetPidStates();
  } else if (feedforward_active_) {
    setSetpointFromNavigator(*(*navigator_msg));
    publishCurrentSetpoint(time);
    resetPidStates();
  }

  const auto & current_pose = (*navigator_msg)->position;
  const auto & setpoint_pose = current_setpoint_.pose;

  tf2::Quaternion q_current(
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);
  q_current.normalize();

  tf2::Quaternion q_setpoint(
    setpoint_pose.orientation.x,
    setpoint_pose.orientation.y,
    setpoint_pose.orientation.z,
    setpoint_pose.orientation.w);
  q_setpoint.normalize();

  const tf2::Vector3 position_error_world(
    setpoint_pose.position.x - current_pose.position.x,
    setpoint_pose.position.y - current_pose.position.y,
    setpoint_pose.position.z - current_pose.position.z);
  const tf2::Vector3 position_error_body =
    tf2::quatRotate(q_current.inverse(), position_error_world);

  tf2::Quaternion q_error = q_current.inverse() * q_setpoint;
  q_error.normalize();
  if (q_error.w() < 0.0) {
    q_error = tf2::Quaternion(-q_error.x(), -q_error.y(), -q_error.z(), -q_error.w());
  }

  double roll_error = 0.0;
  double pitch_error = 0.0;
  double yaw_error = 0.0;
  tf2::Matrix3x3(q_error).getRPY(roll_error, pitch_error, yaw_error);
  roll_error = wrapAngle(roll_error);
  pitch_error = wrapAngle(pitch_error);
  yaw_error = wrapAngle(yaw_error);

  const double dt = period.seconds();
  const auto & angular_velocity = (*navigator_msg)->body_velocity.angular;

  std::array<PidTerms, 6> pid_terms;
  pid_terms[0] = computePidTerms(
    position_error_body.x(), dt, kp_x_, ki_x_, kd_x_, antiwindup_x_, x_pid_);
  pid_terms[1] = computePidTerms(
    position_error_body.y(), dt, kp_y_, ki_y_, kd_y_, antiwindup_y_, y_pid_);
  pid_terms[2] = computePidTerms(
    position_error_body.z(), dt, kp_z_, ki_z_, kd_z_, antiwindup_z_, z_pid_);
  pid_terms[3] = computePidTermsWithMeasuredRate(
    roll_error, angular_velocity.x, dt, kp_roll_, ki_roll_, kd_roll_, antiwindup_roll_, roll_pid_);
  pid_terms[4] = computePidTermsWithMeasuredRate(
    pitch_error, angular_velocity.y, dt, kp_pitch_, ki_pitch_, kd_pitch_, antiwindup_pitch_,
    pitch_pid_);
  pid_terms[5] = computePidTermsWithMeasuredRate(
    yaw_error, angular_velocity.z, dt, kp_yaw_, ki_yaw_, kd_yaw_, antiwindup_yaw_, yaw_pid_);

  const double linear_x_command =
    effective_feedforward.linear.x +
    pid_terms[0].proportional + pid_terms[0].integral + pid_terms[0].derivative;
  const double linear_y_command =
    effective_feedforward.linear.y +
    pid_terms[1].proportional + pid_terms[1].integral + pid_terms[1].derivative;
  const double linear_z_command =
    effective_feedforward.linear.z +
    pid_terms[2].proportional + pid_terms[2].integral + pid_terms[2].derivative;
  const double angular_x_command =
    effective_feedforward.angular.x +
    pid_terms[3].proportional + pid_terms[3].integral + pid_terms[3].derivative;
  const double angular_y_command =
    effective_feedforward.angular.y +
    pid_terms[4].proportional + pid_terms[4].integral + pid_terms[4].derivative;
  const double angular_z_command =
    effective_feedforward.angular.z +
    pid_terms[5].proportional + pid_terms[5].integral + pid_terms[5].derivative;

  command_interfaces_[0].set_value(linear_x_command);
  command_interfaces_[1].set_value(linear_y_command);
  command_interfaces_[2].set_value(linear_z_command);
  command_interfaces_[3].set_value(angular_x_command);
  command_interfaces_[4].set_value(angular_y_command);
  command_interfaces_[5].set_value(angular_z_command);

  publishTelemetry(
    linear_x_command,
    linear_y_command,
    linear_z_command,
    angular_x_command,
    angular_y_command,
    angular_z_command,
    pid_terms);

  feedforward_active_ = has_feedforward;

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
  sura_controllers::auv::PositionHoldController,
  controller_interface::ChainableControllerInterface)
