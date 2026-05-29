#pragma once

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sura_msgs/msg/controller_debug.hpp"
#include "sura_msgs/msg/navigator.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace sura_controllers::usv
{

class BodyPositionController : public controller_interface::ControllerInterface
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
  using SetPointMsg = geometry_msgs::msg::PoseStamped;
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
  void publishThresholdMarkers();
  void resetDebugStats();
  void publishDebugStats();
  void recordDebugCycle(
    const std::chrono::steady_clock::time_point & update_start,
    const rclcpp::Duration & period);

  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Subscription<SetPointMsg>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr threshold_marker_pub_;
  rclcpp::Publisher<DebugMsg>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<SetPointMsg>> setpoint_buffer_;

  std::string navigator_topic_{"navigator/msg"};
  std::string setpoint_topic_{"body_position/setpoint"};
  std::string body_velocity_controller_name_{"body_velocity_controller"};
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
  double yaw_command_sign_{1.0};

  double position_release_radius_{0.60};

  double hold_position_deadband_{0.12};
  double hold_yaw_deadband_{0.12};

  double hold_kp_position_{0.25};
  double hold_kp_yaw_{0.6};

  double hold_max_linear_speed_{0.05};
  double hold_max_angular_speed_{0.10};
  
  const SetPointMsg * last_setpoint_msg_ptr_{nullptr};
  ActiveTarget active_target_{};
  bool debug_enabled_{false};
  bool controller_active_{false};
  bool has_active_target_{false};
  bool holding_current_position_{false};

  std::atomic<uint64_t> debug_desired_period_us_{0};
  std::atomic<uint64_t> debug_cycle_count_{0};
  std::atomic<uint64_t> debug_deadline_miss_count_{0};
  std::atomic<uint64_t> debug_total_update_us_{0};
  std::atomic<uint64_t> debug_last_update_us_{0};
  std::atomic<uint64_t> debug_max_update_us_{0};
  std::atomic<uint64_t> debug_min_update_us_{std::numeric_limits<uint64_t>::max()};
};

}  // namespace sura_controllers::usv
