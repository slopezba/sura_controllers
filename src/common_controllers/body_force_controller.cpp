#include "sura_controllers/common/body_force_controller.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace sura_controllers::common
{

Eigen::Isometry3d BodyForceController::urdfPoseToEigen(const urdf::Pose & pose)
{
  Eigen::Quaterniond q(
    pose.rotation.w,
    pose.rotation.x,
    pose.rotation.y,
    pose.rotation.z);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.normalized().toRotationMatrix();
  T.translation() = Eigen::Vector3d(
    pose.position.x,
    pose.position.y,
    pose.position.z);

  return T;
}

std::vector<std::string> BodyForceController::extractThrusterJointsFromRos2Control(
  const std::string & robot_description) const
{
  tinyxml2::XMLDocument doc;
  if (doc.Parse(robot_description.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to parse robot_description XML");
  }

  const tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("robot_description does not contain a <robot> element");
  }

  std::vector<std::string> joint_names;

  for (
    const tinyxml2::XMLElement * ros2_control = robot->FirstChildElement("ros2_control");
    ros2_control != nullptr;
    ros2_control = ros2_control->NextSiblingElement("ros2_control"))
  {
    for (
      const tinyxml2::XMLElement * joint = ros2_control->FirstChildElement("joint");
      joint != nullptr;
      joint = joint->NextSiblingElement("joint"))
    {
      const char * joint_name = joint->Attribute("name");
      if (joint_name == nullptr) {
        continue;
      }

      bool has_effort_command_interface = false;
      for (
        const tinyxml2::XMLElement * command_interface = joint->FirstChildElement("command_interface");
        command_interface != nullptr;
        command_interface = command_interface->NextSiblingElement("command_interface"))
      {
        const char * interface_name = command_interface->Attribute("name");
        if (interface_name != nullptr && std::string(interface_name) == "effort") {
          has_effort_command_interface = true;
          break;
        }
      }

      if (has_effort_command_interface) {
        joint_names.emplace_back(joint_name);
      }
    }
  }

  if (joint_names.empty()) {
    throw std::runtime_error("No effort command joints found under <ros2_control>");
  }

  return joint_names;
}

std::string BodyForceController::resolveBaseLink(
  const urdf::Model & model,
  const std::string & configured_base_link) const
{
  if (!configured_base_link.empty() && model.getLink(configured_base_link)) {
    return configured_base_link;
  }

  const auto root_link = model.getRoot();
  if (!root_link) {
    throw std::runtime_error("Failed to resolve the robot root link");
  }

  if (!configured_base_link.empty() && configured_base_link != root_link->name) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Configured base_link '%s' was not found. Falling back to root link '%s'.",
      configured_base_link.c_str(),
      root_link->name.c_str());
  }

  return root_link->name;
}

Eigen::Isometry3d BodyForceController::jointPoseInBase(
  const urdf::Model & model,
  const std::string & joint_name,
  const std::string & base_link) const
{
  auto joint = model.getJoint(joint_name);
  if (!joint) {
    throw std::runtime_error("Joint not found: " + joint_name);
  }

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  while (joint) {
    T = urdfPoseToEigen(joint->parent_to_joint_origin_transform) * T;

    if (joint->parent_link_name == base_link) {
      return T;
    }

    auto parent_link = model.getLink(joint->parent_link_name);
    if (!parent_link) {
      throw std::runtime_error(
        "Parent link not found while resolving chain for joint: " + joint_name);
    }

    joint = parent_link->parent_joint;
  }

  throw std::runtime_error(
    "Joint " + joint_name + " is not connected to base link " + base_link);
}

bool BodyForceController::buildThrusterAllocationMatrix(
  const urdf::Model & model,
  const std::string & base_link,
  const std::vector<std::string> & thruster_joints)
{
  const std::size_t n = thruster_joints.size();
  if (n == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "No thruster joints provided");
    return false;
  }

  thruster_allocation_matrix_.resize(6, static_cast<int>(n));

  for (std::size_t i = 0; i < n; ++i) {
    const auto & joint_name = thruster_joints[i];

    Eigen::Isometry3d T_base_thruster;
    try {
      T_base_thruster = jointPoseInBase(model, joint_name, base_link);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Error resolving joint '%s': %s",
        joint_name.c_str(), e.what());
      return false;
    }

    const Eigen::Vector3d r = T_base_thruster.translation();
    const Eigen::Vector3d d = T_base_thruster.rotation() * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d m = r.cross(d);

    thruster_allocation_matrix_.block<3, 1>(0, static_cast<int>(i)) = d;
    thruster_allocation_matrix_.block<3, 1>(3, static_cast<int>(i)) = m;

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Thruster '%s': r = [%.3f %.3f %.3f], d = [%.3f %.3f %.3f], rxd = [%.3f %.3f %.3f]",
      joint_name.c_str(),
      r.x(), r.y(), r.z(),
      d.x(), d.y(), d.z(),
      m.x(), m.y(), m.z());
  }

  std::ostringstream oss;
  oss << "\nThruster Allocation Matrix B (6x" << n << "):\n" << thruster_allocation_matrix_;
  RCLCPP_INFO(get_node()->get_logger(), "%s", oss.str().c_str());

  return true;
}

Eigen::MatrixXd BodyForceController::pseudoInverse(
  const Eigen::MatrixXd & matrix,
  double tolerance) const
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const auto & singular_values = svd.singularValues();
  Eigen::MatrixXd singular_values_inv =
    Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());

  for (int i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) > tolerance) {
      singular_values_inv(i, i) = 1.0 / singular_values(i);
    }
  }

  return svd.matrixV() * singular_values_inv * svd.matrixU().transpose();
}

void BodyForceController::resetDebugStats()
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

void BodyForceController::publishDebugStats()
{
  if (!debug_enabled_ || !debug_pub_) {
    return;
  }

  const uint64_t cycle_count = debug_cycle_count_.load(std::memory_order_relaxed);
  const uint64_t total_update_us = debug_total_update_us_.load(std::memory_order_relaxed);
  const uint64_t min_update_us = debug_min_update_us_.load(std::memory_order_relaxed);

  DebugMsg msg;
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

controller_interface::CallbackReturn BodyForceController::on_init()
{
  try {
    auto_declare<std::string>("input_topic", "/cirtesub/controller/body_force/command");
    auto_declare<std::string>("output_topic", "/cirtesub/controller/body_force/output");
    auto_declare<std::string>("wrench_output_topic", "/cirtesub/controller/body_force/wrench");
    auto_declare<std::string>("base_link", "base_link");
    auto_declare<bool>("debug.enabled", false);
    auto_declare<std::string>("debug.topic", "debug");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BodyForceController::command_interface_configuration() const
{
  std::vector<std::string> names;

  std::vector<std::string> thruster_joints;
  const auto robot_description_parameter = get_node()->get_parameter("robot_description");
  if (robot_description_parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    try {
      thruster_joints = extractThrusterJointsFromRos2Control(
        robot_description_parameter.as_string());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to extract thruster joints from ros2_control: %s",
        e.what());
    }
  }

  names.reserve(thruster_joints.size());
  for (const auto & joint_name : thruster_joints) {
    names.push_back(joint_name + "/effort");
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    names};
}

controller_interface::InterfaceConfiguration
BodyForceController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn BodyForceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  input_topic_ = get_node()->get_parameter("input_topic").as_string();
  output_topic_ = get_node()->get_parameter("output_topic").as_string();
  wrench_output_topic_ = get_node()->get_parameter("wrench_output_topic").as_string();
  base_link_ = get_node()->get_parameter("base_link").as_string();
  debug_enabled_ = get_node()->get_parameter("debug.enabled").as_bool();
  debug_topic_ = get_node()->get_parameter("debug.topic").as_string();

  const std::string robot_description =
    get_node()->get_parameter("robot_description").as_string();

  if (robot_description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  urdf::Model model;
  if (!model.initString(robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse robot_description");
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    thruster_joints_ = extractThrusterJointsFromRos2Control(robot_description);
    base_link_ = resolveBaseLink(model, base_link_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure thruster model: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!buildThrusterAllocationMatrix(model, base_link_, thruster_joints_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  thruster_allocation_matrix_pinv_ = pseudoInverse(thruster_allocation_matrix_);
  thruster_forces_.resize(static_cast<Eigen::Index>(thruster_joints_.size()));
  thruster_forces_.setZero();

  body_force_sub_ = this->get_node()->create_subscription<WrenchMsg>(
    input_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const WrenchMsg::SharedPtr msg)
    {
      rt_buffer_ptr_.writeFromNonRT(msg);
    });

  output_pub_ = this->get_node()->create_publisher<Float64MultiArrayMsg>(
    output_topic_,
    rclcpp::SystemDefaultsQoS());
  output_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<Float64MultiArrayMsg>>(output_pub_);
  output_rt_pub_->msg_.data.resize(thruster_joints_.size(), 0.0);

  wrench_output_pub_ = this->get_node()->create_publisher<WrenchMsg>(
    wrench_output_topic_,
    rclcpp::SystemDefaultsQoS());
  wrench_output_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<WrenchMsg>>(wrench_output_pub_);

  debug_pub_.reset();
  debug_timer_.reset();
  resetDebugStats();

  if (debug_enabled_) {
    debug_pub_ = this->get_node()->create_publisher<DebugMsg>(debug_topic_, 10);
    debug_timer_ = this->get_node()->create_wall_timer(
      std::chrono::seconds(1),
      [this]()
      {
        publishDebugStats();
      });

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Debug profiling enabled. Publishing ControllerDebug messages to '%s'.",
      debug_topic_.c_str());
  }

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

  command_interfaces_.reserve(thruster_joints_.size());

  RCLCPP_INFO(get_node()->get_logger(), "Configured BodyForceController");
  RCLCPP_INFO(get_node()->get_logger(), "Input topic: %s", input_topic_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(), "Wrench output topic: %s", wrench_output_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Base link: %s", base_link_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Thruster joints discovered: %zu", thruster_joints_.size());
  RCLCPP_INFO(get_node()->get_logger(), "Precomputed thruster allocation pseudoinverse");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyForceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<WrenchMsg>>(nullptr);
  controller_active_ = true;

  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  resetDebugStats();

  RCLCPP_INFO(get_node()->get_logger(), "Activated BodyForceController");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyForceController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<WrenchMsg>>(nullptr);
  controller_active_ = false;
  if (debug_enabled_ && debug_pub_) {
    publishDebugStats();
  }

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
BodyForceController::on_export_reference_interfaces()
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

bool BodyForceController::on_set_chained_mode(bool chained_mode)
{
  chained_mode_ = chained_mode;
  if (chained_mode) {
    RCLCPP_INFO(get_node()->get_logger(), "BodyForceController switched to chained mode");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "BodyForceController switched to topic mode");
  }
  return true;
}

controller_interface::return_type BodyForceController::update_reference_from_subscribers()
{
  auto wrench_msg = rt_buffer_ptr_.readFromRT();

  if (!(!wrench_msg || !(*wrench_msg))) {
    reference_interfaces_[0] = (*wrench_msg)->force.x;
    reference_interfaces_[1] = (*wrench_msg)->force.y;
    reference_interfaces_[2] = (*wrench_msg)->force.z;
    reference_interfaces_[3] = (*wrench_msg)->torque.x;
    reference_interfaces_[4] = (*wrench_msg)->torque.y;
    reference_interfaces_[5] = (*wrench_msg)->torque.z;
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type BodyForceController::update_and_write_commands(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const auto update_start = debug_enabled_ ? std::chrono::steady_clock::now() :
    std::chrono::steady_clock::time_point{};

  BodyWrenchVector desired_wrench;
  desired_wrench.setZero();

  for (size_t i = 0; i < 6; ++i) {
    if (!std::isnan(reference_interfaces_[i])) {
      desired_wrench(static_cast<Eigen::Index>(i)) = reference_interfaces_[i];
    }
  }

  thruster_forces_.noalias() = thruster_allocation_matrix_pinv_ * desired_wrench;

  if (thruster_forces_.size() != static_cast<int>(command_interfaces_.size())) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Thruster force vector size does not match number of command interfaces");
    return controller_interface::return_type::ERROR;
  }

  for (int i = 0; i < thruster_forces_.size(); ++i) {
    command_interfaces_[i].set_value(thruster_forces_(i));
  }

  if (output_rt_pub_ && output_rt_pub_->trylock()) {
    auto & data = output_rt_pub_->msg_.data;
    for (int i = 0; i < thruster_forces_.size(); ++i) {
      data[static_cast<size_t>(i)] = thruster_forces_(i);
    }
    output_rt_pub_->unlockAndPublish();
  }

  if (wrench_output_rt_pub_ && wrench_output_rt_pub_->trylock()) {
    wrench_output_rt_pub_->msg_.force.x = desired_wrench(0);
    wrench_output_rt_pub_->msg_.force.y = desired_wrench(1);
    wrench_output_rt_pub_->msg_.force.z = desired_wrench(2);
    wrench_output_rt_pub_->msg_.torque.x = desired_wrench(3);
    wrench_output_rt_pub_->msg_.torque.y = desired_wrench(4);
    wrench_output_rt_pub_->msg_.torque.z = desired_wrench(5);
    wrench_output_rt_pub_->unlockAndPublish();
  }

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

}  // namespace sura_controllers::common

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::common::BodyForceController,
  controller_interface::ChainableControllerInterface)
