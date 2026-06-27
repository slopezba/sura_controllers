#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "Eigen/Dense"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

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
  Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd & jacobian) const;
  bool validateParameters() const;
  void writeZeroCommands();
  double clampJointVelocity(const std::string & joint_name, double value) const;
  int jointIndex(const std::string & joint_name) const;
  int chainJointIndex(const std::string & joint_name) const;

  rclcpp::Subscription<TwistStampedMsg>::SharedPtr command_sub_;
  rclcpp::SyncParametersClient::SharedPtr remote_param_client_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<Command>> command_buffer_;

  std::string command_topic_{"~/twist"};
  std::string robot_description_;
  std::string robot_description_source_node_{"/robot_state_publisher"};
  std::string base_frame_;
  std::string tip_frame_;
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
  bool controller_active_{false};
};

}  // namespace sura_controllers::uvms
