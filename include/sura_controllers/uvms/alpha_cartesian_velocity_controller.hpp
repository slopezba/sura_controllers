#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "Eigen/Dense"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace sura_controllers::uvms
{

class AlphaCartesianVelocityController : public controller_interface::ControllerInterface
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
  using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

  struct Command
  {
    TwistStampedMsg msg;
    rclcpp::Time received_time;
  };

  struct ChainData
  {
    KDL::Chain chain;
    std::vector<std::string> joint_names;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
  };

  std::string resolveRobotDescription();
  bool configureKinematics(const std::string & robot_description);
  bool buildChain(const std::string & tip_frame, ChainData & data) const;
  bool fillChainPositions(const ChainData & data, KDL::JntArray & q) const;
  bool computeLinearVelocityInBase(
    const TwistStampedMsg & command,
    Eigen::Vector3d & linear_velocity_base) const;
  bool computeTipFrame(KDL::Frame & tip_frame) const;
  bool computeTipPosition(Eigen::Vector3d & tip_position_base) const;
  Eigen::Vector3d applyLinearPoseHold(
    const Eigen::Vector3d & feedforward_velocity,
    const Eigen::Vector3d & current_tip_position,
    double period_sec);
  bool hasActiveCartesianCommand(
    const TwistStampedMsg & command,
    const Eigen::Vector3d & linear_velocity_base) const;
  void captureLinearPoseHoldTarget(const Eigen::Vector3d & current_tip_position);
  void publishEndEffectorPose(const rclcpp::Time & time, const KDL::Frame & tip_frame);
  Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd & jacobian) const;
  bool validateParameters() const;
  void resetLinearPoseHold();
  void writeZeroCommands();
  double clampJointVelocity(const std::string & joint_name, double value) const;
  int jointIndex(const std::string & joint_name) const;
  int chainJointIndex(const std::string & joint_name) const;

  rclcpp::Subscription<TwistStampedMsg>::SharedPtr command_sub_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr end_effector_pose_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<PoseStampedMsg>>
  end_effector_pose_rt_pub_;
  rclcpp::SyncParametersClient::SharedPtr remote_param_client_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<Command>> command_buffer_;

  std::string command_topic_{"~/twist"};
  std::string robot_description_;
  std::string robot_description_source_node_{"/robot_state_publisher"};
  std::string base_frame_;
  std::string tip_frame_;
  std::string end_effector_pose_topic_{"~/end_effector_pose"};
  std::string angular_y_joint_;
  std::string angular_z_joint_;

  std::vector<std::string> joints_;
  std::vector<std::string> ik_joints_;
  std::vector<double> velocity_limits_;
  std::unordered_map<std::string, double> velocity_limit_by_joint_;

  KDL::Tree tree_;
  ChainData tip_chain_;
  mutable std::unordered_map<std::string, ChainData> frame_chains_;

  double dls_lambda_{0.05};
  double command_timeout_sec_{0.5};
  double robot_description_wait_timeout_sec_{3.0};
  bool linear_pose_hold_enabled_{true};
  bool linear_hold_active_command_only_{false};
  double linear_hold_kp_{1.0};
  double linear_hold_ki_{0.0};
  double linear_hold_kd_{0.0};
  double linear_hold_integral_limit_{0.05};
  double linear_hold_max_velocity_{0.03};
  double linear_command_threshold_{1e-4};
  double angular_command_threshold_{1e-4};
  double end_effector_pose_publish_rate_{20.0};
  int64_t last_end_effector_pose_publish_time_ns_{0};
  bool linear_hold_initialized_{false};
  Eigen::Vector3d linear_hold_target_base_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_hold_integral_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_hold_last_error_{Eigen::Vector3d::Zero()};
  bool controller_active_{false};
};

}  // namespace sura_controllers::uvms
