#include "sura_controllers/uvms/alpha_cartesian_velocity_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kdl/jacobian.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace sura_controllers::uvms
{

namespace
{

bool isFinite(const Eigen::VectorXd & vector)
{
  return vector.allFinite();
}

bool containsName(const std::vector<std::string> & names, const std::string & name)
{
  return std::find(names.begin(), names.end(), name) != names.end();
}

}  // namespace

controller_interface::CallbackReturn AlphaCartesianVelocityController::on_init()
{
  try {
    auto_declare<std::string>("command_topic", "~/twist");
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_description_source_node", "/robot_state_publisher");
    auto_declare<double>("robot_description_wait_timeout_sec", 3.0);
    auto_declare<std::string>("base_frame", "");
    auto_declare<std::string>("tip_frame", "");
    auto_declare<std::string>("end_effector_pose_topic", "~/end_effector_pose");
    auto_declare<double>("end_effector_pose_publish_rate", 20.0);
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<std::string>>("ik_joints", {});
    auto_declare<std::string>("angular_y_joint", "");
    auto_declare<std::string>("angular_z_joint", "");
    auto_declare<std::vector<double>>("velocity_limits", {});
    auto_declare<double>("dls_lambda", 0.05);
    auto_declare<double>("command_timeout_sec", 0.5);
    auto_declare<bool>("linear_pose_hold_enabled", true);
    auto_declare<bool>("linear_hold_active_command_only", false);
    auto_declare<double>("linear_hold_kp", 1.0);
    auto_declare<double>("linear_hold_ki", 0.0);
    auto_declare<double>("linear_hold_kd", 0.0);
    auto_declare<double>("linear_hold_integral_limit", 0.05);
    auto_declare<double>("linear_hold_max_velocity", 0.03);
    auto_declare<double>("linear_command_threshold", 1e-4);
    auto_declare<double>("angular_command_threshold", 1e-4);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AlphaCartesianVelocityController::command_interface_configuration() const
{
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  std::vector<std::string> names;
  names.reserve(joints.size());
  for (const auto & joint_name : joints) {
    names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::InterfaceConfiguration
AlphaCartesianVelocityController::state_interface_configuration() const
{
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  std::vector<std::string> names;
  names.reserve(joints.size());
  for (const auto & joint_name : joints) {
    names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn AlphaCartesianVelocityController::on_configure(
  const rclcpp_lifecycle::State &)
{
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  robot_description_ = get_node()->get_parameter("robot_description").as_string();
  robot_description_source_node_ =
    get_node()->get_parameter("robot_description_source_node").as_string();
  robot_description_wait_timeout_sec_ =
    get_node()->get_parameter("robot_description_wait_timeout_sec").as_double();
  base_frame_ = get_node()->get_parameter("base_frame").as_string();
  tip_frame_ = get_node()->get_parameter("tip_frame").as_string();
  end_effector_pose_topic_ = get_node()->get_parameter("end_effector_pose_topic").as_string();
  end_effector_pose_publish_rate_ =
    get_node()->get_parameter("end_effector_pose_publish_rate").as_double();
  joints_ = get_node()->get_parameter("joints").as_string_array();
  ik_joints_ = get_node()->get_parameter("ik_joints").as_string_array();
  angular_y_joint_ = get_node()->get_parameter("angular_y_joint").as_string();
  angular_z_joint_ = get_node()->get_parameter("angular_z_joint").as_string();
  velocity_limits_ = get_node()->get_parameter("velocity_limits").as_double_array();
  dls_lambda_ = get_node()->get_parameter("dls_lambda").as_double();
  command_timeout_sec_ = get_node()->get_parameter("command_timeout_sec").as_double();
  linear_pose_hold_enabled_ =
    get_node()->get_parameter("linear_pose_hold_enabled").as_bool();
  linear_hold_active_command_only_ =
    get_node()->get_parameter("linear_hold_active_command_only").as_bool();
  linear_hold_kp_ = get_node()->get_parameter("linear_hold_kp").as_double();
  linear_hold_ki_ = get_node()->get_parameter("linear_hold_ki").as_double();
  linear_hold_kd_ = get_node()->get_parameter("linear_hold_kd").as_double();
  linear_hold_integral_limit_ =
    std::abs(get_node()->get_parameter("linear_hold_integral_limit").as_double());
  linear_hold_max_velocity_ =
    std::abs(get_node()->get_parameter("linear_hold_max_velocity").as_double());
  linear_command_threshold_ =
    std::max(0.0, get_node()->get_parameter("linear_command_threshold").as_double());
  angular_command_threshold_ =
    std::max(0.0, get_node()->get_parameter("angular_command_threshold").as_double());

  if (!validateParameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  velocity_limit_by_joint_.clear();
  if (!velocity_limits_.empty()) {
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      velocity_limit_by_joint_[joints_[i]] = std::abs(velocity_limits_[i]);
    }
  }

  try {
    if (!configureKinematics(resolveRobotDescription())) {
      return controller_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure kinematics: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  command_sub_ = get_node()->create_subscription<TwistStampedMsg>(
    command_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const TwistStampedMsg::SharedPtr msg)
    {
      auto command = std::make_shared<Command>();
      command->msg = *msg;
      command->received_time = get_node()->now();
      command_buffer_.writeFromNonRT(command);
    });

  end_effector_pose_pub_ = get_node()->create_publisher<PoseStampedMsg>(
    end_effector_pose_topic_,
    rclcpp::SystemDefaultsQoS());
  end_effector_pose_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<PoseStampedMsg>>(
    end_effector_pose_pub_);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured AlphaCartesianVelocityController for %s -> %s, publishing tip pose on '%s'",
    base_frame_.c_str(),
    tip_frame_.c_str(),
    end_effector_pose_topic_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AlphaCartesianVelocityController::on_activate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = true;
  resetLinearPoseHold();
  last_end_effector_pose_publish_time_ns_ = 0;
  command_buffer_.writeFromNonRT(nullptr);
  writeZeroCommands();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AlphaCartesianVelocityController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  controller_active_ = false;
  resetLinearPoseHold();
  last_end_effector_pose_publish_time_ns_ = 0;
  writeZeroCommands();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AlphaCartesianVelocityController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  if (!controller_active_) {
    return controller_interface::return_type::OK;
  }

  KDL::JntArray q(tip_chain_.joint_names.size());
  if (!fillChainPositions(tip_chain_, q)) {
    writeZeroCommands();
    return controller_interface::return_type::OK;
  }

  KDL::Frame tip_frame;
  if (tip_chain_.fk_solver->JntToCart(q, tip_frame) < 0) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Failed to compute Alpha tip pose; writing zero velocities");
    writeZeroCommands();
    return controller_interface::return_type::OK;
  }
  publishEndEffectorPose(time, tip_frame);

  const auto command_handle = command_buffer_.readFromRT();
  const std::shared_ptr<Command> command = command_handle ? *command_handle : nullptr;
  if (!command) {
    resetLinearPoseHold();
    writeZeroCommands();
    return controller_interface::return_type::OK;
  }

  if ((time - command->received_time).seconds() > command_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Alpha cartesian velocity command timed out; writing zero velocities");
    resetLinearPoseHold();
    writeZeroCommands();
    return controller_interface::return_type::OK;
  }

  Eigen::Vector3d linear_velocity_base = Eigen::Vector3d::Zero();
  if (!computeLinearVelocityInBase(command->msg, linear_velocity_base)) {
    writeZeroCommands();
    return controller_interface::return_type::OK;
  }

  if (linear_pose_hold_enabled_) {
    const Eigen::Vector3d current_tip_position_base(
      tip_frame.p.x(),
      tip_frame.p.y(),
      tip_frame.p.z());

    if (linear_hold_active_command_only_ &&
      !hasActiveCartesianCommand(command->msg, linear_velocity_base))
    {
      captureLinearPoseHoldTarget(current_tip_position_base);
      writeZeroCommands();
      return controller_interface::return_type::OK;
    }

    linear_velocity_base = applyLinearPoseHold(
      linear_velocity_base,
      current_tip_position_base,
      std::max(0.0, period.seconds()));
  } else {
    resetLinearPoseHold();
  }

  KDL::Jacobian jacobian(tip_chain_.joint_names.size());
  if (tip_chain_.jacobian_solver->JntToJac(q, jacobian) < 0) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Failed to compute Alpha KDL Jacobian; writing zero velocities");
    writeZeroCommands();
    return controller_interface::return_type::OK;
  }

  Eigen::MatrixXd linear_jacobian(3, static_cast<Eigen::Index>(ik_joints_.size()));
  for (std::size_t col = 0; col < ik_joints_.size(); ++col) {
    const int kdl_col = chainJointIndex(ik_joints_[col]);
    if (kdl_col < 0) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "IK joint '%s' is not in the KDL chain; writing zero velocities",
        ik_joints_[col].c_str());
      writeZeroCommands();
      return controller_interface::return_type::OK;
    }
    for (int row = 0; row < 3; ++row) {
      linear_jacobian(row, static_cast<Eigen::Index>(col)) =
        jacobian(row, static_cast<unsigned int>(kdl_col));
    }
  }

  const Eigen::VectorXd ik_velocity =
    dampedPseudoInverse(linear_jacobian) * linear_velocity_base;
  if (!isFinite(ik_velocity)) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Alpha IK produced non-finite velocities; writing zero velocities");
    writeZeroCommands();
    return controller_interface::return_type::OK;
  }

  std::vector<double> command_by_joint(joints_.size(), 0.0);
  for (std::size_t i = 0; i < ik_joints_.size(); ++i) {
    const int idx = jointIndex(ik_joints_[i]);
    if (idx >= 0) {
      command_by_joint[static_cast<std::size_t>(idx)] =
        ik_velocity(static_cast<Eigen::Index>(i));
    }
  }

  const int angular_y_idx = jointIndex(angular_y_joint_);
  const int angular_z_idx = jointIndex(angular_z_joint_);
  if (angular_y_idx >= 0) {
    command_by_joint[static_cast<std::size_t>(angular_y_idx)] = command->msg.twist.angular.y;
  }
  if (angular_z_idx >= 0) {
    command_by_joint[static_cast<std::size_t>(angular_z_idx)] = command->msg.twist.angular.z;
  }

  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(clampJointVelocity(joints_[i], command_by_joint[i]));
  }

  return controller_interface::return_type::OK;
}

std::string AlphaCartesianVelocityController::resolveRobotDescription()
{
  if (!robot_description_.empty()) {
    return robot_description_;
  }

  const double timeout_sec = std::max(0.1, robot_description_wait_timeout_sec_);
  remote_param_client_ =
    std::make_shared<rclcpp::SyncParametersClient>(get_node(), robot_description_source_node_);
  if (!remote_param_client_->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    throw std::runtime_error(
      "robot_description is empty locally and parameter service for '" +
      robot_description_source_node_ + "' is not available");
  }

  const auto parameters = remote_param_client_->get_parameters({"robot_description"});
  if (parameters.empty()) {
    throw std::runtime_error(
      "Failed to read robot_description from '" + robot_description_source_node_ + "'");
  }

  const std::string value = parameters.front().as_string();
  if (value.empty()) {
    throw std::runtime_error(
      "Node '" + robot_description_source_node_ + "' returned an empty robot_description");
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Loaded robot_description at runtime from '%s'",
    robot_description_source_node_.c_str());
  return value;
}

bool AlphaCartesianVelocityController::configureKinematics(
  const std::string & robot_description)
{
  if (!kdl_parser::treeFromString(robot_description, tree_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse robot_description into a KDL tree");
    return false;
  }

  frame_chains_.clear();
  if (!buildChain(tip_frame_, tip_chain_)) {
    return false;
  }

  for (const auto & ik_joint : ik_joints_) {
    if (chainJointIndex(ik_joint) < 0) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "IK joint '%s' is not part of the chain %s -> %s",
        ik_joint.c_str(),
        base_frame_.c_str(),
        tip_frame_.c_str());
      return false;
    }
  }

  return true;
}

bool AlphaCartesianVelocityController::buildChain(
  const std::string & tip_frame,
  ChainData & data) const
{
  if (!tree_.getChain(base_frame_, tip_frame, data.chain)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to extract KDL chain from '%s' to '%s'",
      base_frame_.c_str(),
      tip_frame.c_str());
    return false;
  }

  data.joint_names.clear();
  for (unsigned int i = 0; i < data.chain.getNrOfSegments(); ++i) {
    const auto & joint = data.chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None) {
      data.joint_names.push_back(joint.getName());
    }
  }
  data.fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(data.chain);
  data.jacobian_solver = std::make_unique<KDL::ChainJntToJacSolver>(data.chain);
  return true;
}

bool AlphaCartesianVelocityController::fillChainPositions(
  const ChainData & data,
  KDL::JntArray & q) const
{
  for (std::size_t i = 0; i < data.joint_names.size(); ++i) {
    const int idx = jointIndex(data.joint_names[i]);
    if (idx < 0 || static_cast<std::size_t>(idx) >= state_interfaces_.size()) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "No position state interface found for joint '%s'",
        data.joint_names[i].c_str());
      return false;
    }

    const double value = state_interfaces_[static_cast<std::size_t>(idx)].get_value();
    if (!std::isfinite(value)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "Non-finite position state for joint '%s'",
        data.joint_names[i].c_str());
      return false;
    }
    q(static_cast<unsigned int>(i)) = value;
  }
  return true;
}

bool AlphaCartesianVelocityController::computeLinearVelocityInBase(
  const TwistStampedMsg & command,
  Eigen::Vector3d & linear_velocity_base) const
{
  const Eigen::Vector3d input_velocity(
    command.twist.linear.x,
    command.twist.linear.y,
    command.twist.linear.z);
  if (!input_velocity.allFinite()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Received non-finite linear velocity; writing zero velocities");
    return false;
  }

  const std::string frame_id = command.header.frame_id.empty() ? base_frame_ : command.header.frame_id;
  if (frame_id == base_frame_) {
    linear_velocity_base = input_velocity;
    return true;
  }

  auto chain_it = frame_chains_.find(frame_id);
  if (chain_it == frame_chains_.end()) {
    ChainData frame_chain;
    if (!buildChain(frame_id, frame_chain)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "Frame '%s' is not reachable from base frame '%s'; writing zero velocities",
        frame_id.c_str(),
        base_frame_.c_str());
      return false;
    }
    chain_it = frame_chains_.emplace(frame_id, std::move(frame_chain)).first;
  }

  const ChainData & frame_chain = chain_it->second;
  KDL::JntArray q(frame_chain.joint_names.size());
  if (!fillChainPositions(frame_chain, q)) {
    return false;
  }

  KDL::Frame frame;
  if (frame_chain.fk_solver->JntToCart(q, frame) < 0) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Failed to compute transform from '%s' to '%s'; writing zero velocities",
      base_frame_.c_str(),
      frame_id.c_str());
    return false;
  }

  Eigen::Matrix3d rotation_base_frame;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation_base_frame(row, col) = frame.M(row, col);
    }
  }
  linear_velocity_base = rotation_base_frame * input_velocity;
  return true;
}

bool AlphaCartesianVelocityController::computeTipFrame(KDL::Frame & tip_frame) const
{
  KDL::JntArray q(tip_chain_.joint_names.size());
  if (!fillChainPositions(tip_chain_, q)) {
    return false;
  }

  if (tip_chain_.fk_solver->JntToCart(q, tip_frame) < 0) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Failed to compute Alpha tip pose; writing zero velocities");
    return false;
  }

  return true;
}

bool AlphaCartesianVelocityController::computeTipPosition(
  Eigen::Vector3d & tip_position_base) const
{
  KDL::Frame tip_frame;
  if (!computeTipFrame(tip_frame)) {
    return false;
  }

  tip_position_base = Eigen::Vector3d(
    tip_frame.p.x(),
    tip_frame.p.y(),
    tip_frame.p.z());
  return true;
}

Eigen::Vector3d AlphaCartesianVelocityController::applyLinearPoseHold(
  const Eigen::Vector3d & feedforward_velocity,
  const Eigen::Vector3d & current_tip_position,
  double period_sec)
{
  if (!linear_hold_initialized_) {
    linear_hold_target_base_ = current_tip_position;
    linear_hold_integral_.setZero();
    linear_hold_last_error_.setZero();
    linear_hold_initialized_ = true;
  }

  Eigen::Vector3d output_velocity = feedforward_velocity;
  const double safe_period = std::max(0.0, period_sec);

  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    const bool axis_has_feedforward =
      std::abs(feedforward_velocity(axis)) > linear_command_threshold_;

    if (axis_has_feedforward) {
      linear_hold_target_base_(axis) = current_tip_position(axis);
      linear_hold_integral_(axis) = 0.0;
      linear_hold_last_error_(axis) = 0.0;
      continue;
    }

    const double error = linear_hold_target_base_(axis) - current_tip_position(axis);
    if (safe_period > 0.0) {
      linear_hold_integral_(axis) += error * safe_period;
      linear_hold_integral_(axis) = std::clamp(
        linear_hold_integral_(axis),
        -linear_hold_integral_limit_,
        linear_hold_integral_limit_);
    }

    const double derivative =
      safe_period > 0.0 ? (error - linear_hold_last_error_(axis)) / safe_period : 0.0;
    linear_hold_last_error_(axis) = error;

    const double feedback =
      linear_hold_kp_ * error +
      linear_hold_ki_ * linear_hold_integral_(axis) +
      linear_hold_kd_ * derivative;

    output_velocity(axis) += std::clamp(
      feedback,
      -linear_hold_max_velocity_,
      linear_hold_max_velocity_);
  }

  return output_velocity;
}

bool AlphaCartesianVelocityController::hasActiveCartesianCommand(
  const TwistStampedMsg & command,
  const Eigen::Vector3d & linear_velocity_base) const
{
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if (std::abs(linear_velocity_base(axis)) > linear_command_threshold_) {
      return true;
    }
  }

  return std::abs(command.twist.angular.y) > angular_command_threshold_ ||
         std::abs(command.twist.angular.z) > angular_command_threshold_;
}

void AlphaCartesianVelocityController::captureLinearPoseHoldTarget(
  const Eigen::Vector3d & current_tip_position)
{
  linear_hold_target_base_ = current_tip_position;
  linear_hold_integral_.setZero();
  linear_hold_last_error_.setZero();
  linear_hold_initialized_ = true;
}

void AlphaCartesianVelocityController::publishEndEffectorPose(
  const rclcpp::Time & time,
  const KDL::Frame & tip_frame)
{
  if (!end_effector_pose_rt_pub_) {
    return;
  }

  const int64_t now_ns = time.nanoseconds();
  if (
    end_effector_pose_publish_rate_ > 0.0 &&
    last_end_effector_pose_publish_time_ns_ != 0)
  {
    const double elapsed =
      static_cast<double>(now_ns - last_end_effector_pose_publish_time_ns_) * 1e-9;
    if (elapsed < (1.0 / end_effector_pose_publish_rate_)) {
      return;
    }
  }

  if (!end_effector_pose_rt_pub_->trylock()) {
    return;
  }

  auto & msg = end_effector_pose_rt_pub_->msg_;
  msg.header.stamp = time;
  msg.header.frame_id = base_frame_;
  msg.pose.position.x = tip_frame.p.x();
  msg.pose.position.y = tip_frame.p.y();
  msg.pose.position.z = tip_frame.p.z();

  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  tip_frame.M.GetQuaternion(qx, qy, qz, qw);
  msg.pose.orientation.x = qx;
  msg.pose.orientation.y = qy;
  msg.pose.orientation.z = qz;
  msg.pose.orientation.w = qw;

  last_end_effector_pose_publish_time_ns_ = now_ns;
  end_effector_pose_rt_pub_->unlockAndPublish();
}

Eigen::MatrixXd AlphaCartesianVelocityController::dampedPseudoInverse(
  const Eigen::MatrixXd & jacobian) const
{
  const double lambda = std::max(0.0, dls_lambda_);
  const Eigen::MatrixXd metric =
    jacobian * jacobian.transpose() +
    (lambda * lambda) * Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows());
  return jacobian.transpose() * metric.completeOrthogonalDecomposition().pseudoInverse();
}

bool AlphaCartesianVelocityController::validateParameters() const
{
  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty");
    return false;
  }
  if (ik_joints_.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'ik_joints' must contain exactly 3 joints");
    return false;
  }
  if (base_frame_.empty() || tip_frame_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameters 'base_frame' and 'tip_frame' are required");
    return false;
  }
  if (!velocity_limits_.empty() && velocity_limits_.size() != joints_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'velocity_limits' must be empty or have the same length as 'joints'");
    return false;
  }
  if (command_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'command_timeout_sec' must be positive");
    return false;
  }
  if (end_effector_pose_topic_.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'end_effector_pose_topic' must not be empty");
    return false;
  }
  if (end_effector_pose_publish_rate_ < 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'end_effector_pose_publish_rate' must be zero or positive");
    return false;
  }
  if (angular_y_joint_.empty() || angular_z_joint_.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameters 'angular_y_joint' and 'angular_z_joint' are required");
    return false;
  }
  if (!containsName(joints_, angular_y_joint_) || !containsName(joints_, angular_z_joint_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Angular mapping joints must be included in parameter 'joints'");
    return false;
  }
  for (const auto & ik_joint : ik_joints_) {
    if (!containsName(joints_, ik_joint)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "IK joint '%s' must be included in parameter 'joints'",
        ik_joint.c_str());
      return false;
    }
  }
  return true;
}

void AlphaCartesianVelocityController::resetLinearPoseHold()
{
  linear_hold_initialized_ = false;
  linear_hold_target_base_.setZero();
  linear_hold_integral_.setZero();
  linear_hold_last_error_.setZero();
}

void AlphaCartesianVelocityController::writeZeroCommands()
{
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
}

double AlphaCartesianVelocityController::clampJointVelocity(
  const std::string & joint_name,
  double value) const
{
  const auto limit_it = velocity_limit_by_joint_.find(joint_name);
  if (limit_it == velocity_limit_by_joint_.end() || limit_it->second <= 0.0) {
    return value;
  }
  return std::clamp(value, -limit_it->second, limit_it->second);
}

int AlphaCartesianVelocityController::jointIndex(const std::string & joint_name) const
{
  const auto it = std::find(joints_.begin(), joints_.end(), joint_name);
  if (it == joints_.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(joints_.begin(), it));
}

int AlphaCartesianVelocityController::chainJointIndex(const std::string & joint_name) const
{
  const auto it = std::find(tip_chain_.joint_names.begin(), tip_chain_.joint_names.end(), joint_name);
  if (it == tip_chain_.joint_names.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(tip_chain_.joint_names.begin(), it));
}

}  // namespace sura_controllers::uvms

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::uvms::AlphaCartesianVelocityController,
  controller_interface::ControllerInterface)
