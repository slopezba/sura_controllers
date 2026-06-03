#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tinyxml2.h>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sura_msgs/msg/controller_debug.hpp"
#include "urdf/model.h"

namespace sura_controllers::common
{

class BodyForceController : public controller_interface::ChainableControllerInterface
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
  using BodyWrenchVector = Eigen::Matrix<double, 6, 1>;
  using ThrusterAllocationMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;
  using ThrusterAllocationPseudoInverse = Eigen::Matrix<double, Eigen::Dynamic, 6>;
  using ThrusterForceVector = Eigen::Matrix<double, Eigen::Dynamic, 1>;

  static Eigen::Isometry3d urdfPoseToEigen(const urdf::Pose & pose);

  std::vector<std::string> extractThrusterJointsFromRos2Control(
    const std::string & robot_description) const;

  std::string resolveBaseLink(
    const urdf::Model & model,
    const std::string & configured_base_link) const;

  Eigen::Isometry3d jointPoseInBase(
    const urdf::Model & model,
    const std::string & joint_name,
    const std::string & base_link) const;

  bool buildThrusterAllocationMatrix(
    const urdf::Model & model,
    const std::string & base_link,
    const std::vector<std::string> & thruster_joints);

  Eigen::MatrixXd pseudoInverse(
    const Eigen::MatrixXd & matrix,
    double tolerance = 1e-6) const;

  void resetDebugStats();
  void publishDebugStats();

  using WrenchMsg = geometry_msgs::msg::Wrench;
  using Float64MultiArrayMsg = std_msgs::msg::Float64MultiArray;
  using DebugMsg = sura_msgs::msg::ControllerDebug;

  rclcpp::Subscription<WrenchMsg>::SharedPtr body_force_sub_;
  rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr output_pub_;
  rclcpp::Publisher<WrenchMsg>::SharedPtr wrench_output_pub_;
  rclcpp::Publisher<DebugMsg>::SharedPtr debug_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<Float64MultiArrayMsg>> output_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<WrenchMsg>> wrench_output_rt_pub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<WrenchMsg>> rt_buffer_ptr_;

  std::string input_topic_;
  std::string output_topic_;
  std::string wrench_output_topic_;
  std::string debug_topic_;
  std::string base_link_;
  std::vector<std::string> thruster_joints_;
  bool debug_enabled_{false};
  bool controller_active_{false};
  bool chained_mode_{false};

  std::vector<std::string> reference_interface_names_;

  ThrusterAllocationMatrix thruster_allocation_matrix_;
  ThrusterAllocationPseudoInverse thruster_allocation_matrix_pinv_;
  ThrusterForceVector thruster_forces_;

  std::atomic<uint64_t> debug_desired_period_us_{0};
  std::atomic<uint64_t> debug_cycle_count_{0};
  std::atomic<uint64_t> debug_deadline_miss_count_{0};
  std::atomic<uint64_t> debug_total_update_us_{0};
  std::atomic<uint64_t> debug_last_update_us_{0};
  std::atomic<uint64_t> debug_max_update_us_{0};
  std::atomic<uint64_t> debug_min_update_us_{std::numeric_limits<uint64_t>::max()};
};

}  // namespace sura_controllers::common
