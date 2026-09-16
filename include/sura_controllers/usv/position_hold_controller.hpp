#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sura_msgs/msg/controller_debug.hpp"
#include "sura_msgs/msg/navigator.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace sura_controllers::usv
{

class PositionHoldController : public controller_interface::ControllerInterface
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

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
  using TwistMsg = geometry_msgs::msg::Twist;
  using Float64MultiArrayMsg = std_msgs::msg::Float64MultiArray;
  using DebugMsg = sura_msgs::msg::ControllerDebug;
  using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

  struct ActiveTarget
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  static double normalizeAngle(double angle);
  static double clampAbs(double value, double limit);
  static double yawFromPose(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);

  bool setActiveTarget(double x, double y, double yaw, bool hold_current_position);
  void publishCurrentSetpoint(const rclcpp::Time & stamp);
  void publishThresholdMarkers();
  void publishBodyVelocitySetpoint(const TwistMsg & command);
  void publishZeroBodyVelocitySetpoint();
  void publishTelemetry(
    const TwistMsg & command,
    const std::array<double, 6> & control_terms);
  void resetDebugStats();
  void publishDebugStats();
  void recordDebugCycle(
    const std::chrono::steady_clock::time_point & update_start,
    const rclcpp::Duration & period);

  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Subscription<PoseStampedMsg>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<TwistMsg>::SharedPtr feedforward_sub_;
  rclcpp::Subscription<TwistMsg>::SharedPtr reposition_feedforward_sub_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr body_velocity_setpoint_pub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr output_pub_;
  rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr pid_terms_pub_;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr threshold_marker_pub_;
  rclcpp::Publisher<DebugMsg>::SharedPtr debug_pub_;

  std::shared_ptr<realtime_tools::RealtimePublisher<PoseStampedMsg>> setpoint_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<TwistMsg>>
  body_velocity_setpoint_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<TwistMsg>> output_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<Float64MultiArrayMsg>> pid_terms_rt_pub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<PoseStampedMsg>> setpoint_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistMsg>> feedforward_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistMsg>> reposition_feedforward_buffer_;

  std::string navigator_topic_{"navigator/navigation"};
  std::string setpoint_topic_{"position_hold/setpoint"};
  std::string feedforward_topic_{"position_hold/feedforward"};
  std::string reposition_feedforward_topic_{"position_hold/reposition_feedforward"};
  std::string body_velocity_setpoint_topic_{"body_velocity/setpoint"};
  std::string output_topic_{"position_hold/output"};
  std::string pid_terms_topic_{"position_hold/pid_terms"};
  std::string threshold_markers_topic_{"position_hold/threshold_markers"};
  std::string body_velocity_controller_name_{"body_velocity"};
  std::string setpoint_frame_id_{"world_ned"};
  std::string debug_topic_{"debug"};

  double kp_position_{0.0};
  double kp_yaw_{0.0};
  double max_linear_speed_{0.0};
  double max_angular_speed_{0.0};
  double position_hold_radius_{0.0};
  double yaw_tolerance_{0.0};
  double slow_down_radius_{0.0};
  double heading_error_stop_{0.0};
  double reverse_distance_threshold_{0.0};
  double max_reverse_speed_{0.0};
  double position_release_radius_{0.60};
  double hold_position_deadband_{0.12};
  double hold_yaw_deadband_{0.12};
  double hold_kp_position_{0.25};
  double hold_kp_yaw_{0.6};
  double hold_max_linear_speed_{0.05};
  double hold_max_angular_speed_{0.10};
  double linear_feedforward_threshold_{1e-3};
  double angular_feedforward_threshold_{1e-3};
  double feedforward_timeout_{0.25};

  ActiveTarget active_target_{};
  std::string active_target_frame_id_{"world_ned"};
  bool debug_enabled_{false};
  bool controller_active_{false};
  bool has_active_target_{false};
  bool holding_current_position_{false};
  bool feedforward_active_{false};
  bool reposition_feedforward_active_{false};

  std::atomic<bool> new_setpoint_requested_{false};
  std::atomic<int64_t> last_feedforward_time_ns_{0};
  std::atomic<int64_t> last_reposition_feedforward_time_ns_{0};
  std::atomic<uint64_t> debug_desired_period_us_{0};
  std::atomic<uint64_t> debug_cycle_count_{0};
  std::atomic<uint64_t> debug_deadline_miss_count_{0};
  std::atomic<uint64_t> debug_total_update_us_{0};
  std::atomic<uint64_t> debug_last_update_us_{0};
  std::atomic<uint64_t> debug_max_update_us_{0};
  std::atomic<uint64_t> debug_min_update_us_{std::numeric_limits<uint64_t>::max()};
};

}  // namespace sura_controllers::usv
