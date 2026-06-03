#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_msgs/msg/controller_debug.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace sura_controllers::auv
{

class StabilizeController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_reference_from_subscribers() override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using WrenchMsg = geometry_msgs::msg::Wrench;
  using Vector3Msg = geometry_msgs::msg::Vector3;
  using Float64MultiArrayMsg = std_msgs::msg::Float64MultiArray;
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using TriggerSrv = std_srvs::srv::Trigger;

  struct AxisPidState
  {
    double integral{0.0};
    double previous_error{0.0};
  };

  struct PidTerms
  {
    double proportional{0.0};
    double integral{0.0};
    double derivative{0.0};
  };

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & params);

  double computePid(
    double error,
    double dt,
    double kp,
    double ki,
    double kd,
    double antiwindup,
    AxisPidState & state);

  PidTerms computePidTerms(
    double error,
    double dt,
    double kp,
    double ki,
    double kd,
    double antiwindup,
    AxisPidState & state);

  PidTerms computePidTermsWithMeasuredRate(
    double error,
    double measured_rate,
    double dt,
    double kp,
    double ki,
    double kd,
    double antiwindup,
    AxisPidState & state);

  static double wrapAngle(double angle);
  void publishSetpoint();
  void publishTelemetry(
    double force_x,
    double force_y,
    double force_z,
    double torque_x,
    double torque_y,
    double torque_z,
    const std::array<PidTerms, 3> & pid_terms);
  void setRollPitchEnabled(bool enabled, bool request_zero_setpoint);
  void resetDebugStats();
  void publishDebugStats();

  rclcpp::Subscription<WrenchMsg>::SharedPtr feedforward_sub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Service<TriggerSrv>::SharedPtr enable_roll_pitch_srv_;
  rclcpp::Service<TriggerSrv>::SharedPtr disable_roll_pitch_srv_;
  rclcpp::Publisher<Vector3Msg>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<WrenchMsg>::SharedPtr output_pub_;
  rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr pid_terms_pub_;
  rclcpp::Publisher<sura_msgs::msg::ControllerDebug>::SharedPtr debug_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<Vector3Msg>> setpoint_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<WrenchMsg>> output_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<Float64MultiArrayMsg>> pid_terms_rt_pub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<WrenchMsg>> feedforward_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;
  realtime_tools::RealtimeBuffer<bool> allow_roll_pitch_buffer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string feedforward_topic_;
  std::string navigator_topic_;
  std::string debug_topic_;
  std::string setpoint_topic_;
  std::string output_topic_;
  std::string pid_terms_topic_;
  std::string enable_roll_pitch_service_name_;
  std::string disable_roll_pitch_service_name_;
  std::string body_force_controller_name_;
  std::vector<std::string> reference_interface_names_;
  bool debug_enabled_{false};
  bool controller_active_{false};
  bool chained_mode_{false};

  double kp_roll_{0.0};
  double ki_roll_{0.0};
  double kd_roll_{0.0};
  double antiwindup_roll_{0.0};

  double kp_pitch_{0.0};
  double ki_pitch_{0.0};
  double kd_pitch_{0.0};
  double antiwindup_pitch_{0.0};

  double kp_yaw_{0.0};
  double ki_yaw_{0.0};
  double kd_yaw_{0.0};
  double antiwindup_yaw_{0.0};

  bool allow_roll_pitch_{false};
  double feedforward_gain_x_{1.0};
  double feedforward_gain_y_{1.0};
  double feedforward_gain_z_{1.0};
  double feedforward_gain_roll_{1.0};
  double feedforward_gain_pitch_{1.0};
  double feedforward_gain_yaw_{1.0};
  double roll_command_threshold_{1e-3};
  double pitch_command_threshold_{1e-3};
  double yaw_command_threshold_{1e-3};
  double roll_setpoint_{0.0};
  double pitch_setpoint_{0.0};
  double yaw_setpoint_{0.0};
  bool roll_setpoint_initialized_{false};
  bool pitch_setpoint_initialized_{false};
  bool yaw_setpoint_initialized_{false};
  bool roll_feedforward_active_{false};
  bool pitch_feedforward_active_{false};
  bool yaw_feedforward_active_{false};
  std::atomic<bool> zero_roll_pitch_requested_{false};

  AxisPidState roll_pid_;
  AxisPidState pitch_pid_;
  AxisPidState yaw_pid_;
  bool first_update_{true};
  std::atomic<uint64_t> debug_desired_period_us_{0};
  std::atomic<uint64_t> debug_cycle_count_{0};
  std::atomic<uint64_t> debug_deadline_miss_count_{0};
  std::atomic<uint64_t> debug_total_update_us_{0};
  std::atomic<uint64_t> debug_last_update_us_{0};
  std::atomic<uint64_t> debug_max_update_us_{0};
  std::atomic<uint64_t> debug_min_update_us_{std::numeric_limits<uint64_t>::max()};
};

}  // namespace sura_controllers::auv
