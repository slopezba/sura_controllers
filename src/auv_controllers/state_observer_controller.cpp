#include "sura_controllers/auv/state_observer_controller.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>

#include "pluginlib/class_list_macros.hpp"
#include "yaml-cpp/yaml.h"

namespace sura_controllers::auv
{

namespace
{

bool readYamlVector6(
  const YAML::Node & node,
  const std::string & key,
  StateObserverModel::Vector6 & vector)
{
  const YAML::Node values = node[key];
  if (!values || !values.IsSequence() || values.size() != StateObserverModel::kAxisCount) {
    return false;
  }

  for (std::size_t axis = 0; axis < StateObserverModel::kAxisCount; ++axis) {
    vector(static_cast<Eigen::Index>(axis)) = values[axis].as<double>();
  }
  return true;
}

YAML::Node vectorToYaml(const StateObserverModel::Vector6 & vector)
{
  YAML::Node node(YAML::NodeType::Sequence);
  for (std::size_t axis = 0; axis < StateObserverModel::kAxisCount; ++axis) {
    node.push_back(vector(static_cast<Eigen::Index>(axis)));
  }
  return node;
}

}  // namespace

controller_interface::CallbackReturn StateObserverController::on_init()
{
  try {
    auto_declare<std::string>("navigator_topic", "");
    auto_declare<std::string>("force_topic", "");
    auto_declare<std::string>("imu_topic", "sensors/imu");
    auto_declare<std::string>("aruco_pose_topic", "sensors/aruco/pose_enu");
    auto_declare<std::string>("depth_pose_topic", "sensors/pressure/pose");
    auto_declare<std::string>("predicted_pose_topic", "state_observer/predicted_pose");
    auto_declare<std::string>("predicted_odometry_topic", "state_observer/predicted_odometry");
    auto_declare<std::string>("model_topic", "state_observer/model");
    auto_declare<std::string>("freeze_service_name", "state_observer/freeze");
    auto_declare<std::string>("reset_service_name", "state_observer/reset");
    auto_declare<std::string>("save_service_name", "state_observer/save");
    auto_declare<std::string>("load_service_name", "state_observer/load");
    auto_declare<std::string>("learn_service_name", "state_observer/learn");
    auto_declare<std::string>("predict_service_name", "state_observer/predict");
    auto_declare<std::string>("world_frame_id", "world_ned");
    auto_declare<std::string>("base_frame_id", "");
    auto_declare<std::string>(
      "model_file",
      "/home/cirtesu/cirtesub_ws/src/cirtesub_description/config/"
      "cirtesub_state_observer_model.yaml");
    auto_declare<std::string>("mode", "learn");

    auto_declare<double>("navigator_timeout", 0.5);
    auto_declare<double>("force_timeout", 0.5);
    auto_declare<double>("imu_timeout", 0.5);
    auto_declare<double>("aruco_pose_timeout", 0.5);
    auto_declare<double>("depth_pose_timeout", 0.5);
    auto_declare<double>("forgetting_factor", 0.995);
    auto_declare<double>("min_excitation", 1e-3);
    auto_declare<double>("initial_estimator_covariance", 1e3);
    auto_declare<double>("model_publish_period", 1.0);
    auto_declare<double>("max_prediction_dt", 0.1);
    auto_declare<double>("weight", 112.8);
    auto_declare<double>("buoyancy", 114.8);
    auto_declare<bool>("use_imu_linear_acceleration", false);

    auto_declare<std::vector<double>>(
      "rigid_body_inertia", {11.5, 11.5, 11.5, 0.16, 0.16, 0.16});
    auto_declare<std::vector<double>>(
      "effective_inertia", {17.0, 24.2, 26.07, 0.28, 0.28, 0.28});
    auto_declare<std::vector<double>>(
      "linear_damping", {4.03, 6.22, 5.18, 0.07, 0.07, 0.07});
    auto_declare<std::vector<double>>(
      "quadratic_damping", {18.18, 21.66, 36.99, 1.55, 1.55, 1.55});
    auto_declare<std::vector<double>>(
      "static_wrench", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    auto_declare<std::vector<double>>(
      "navigation_pose_covariance", {0.02, 0.02, 0.02, 0.01, 0.01, 0.01});
    auto_declare<std::vector<double>>(
      "process_noise", {0.02, 0.02, 0.02, 0.01, 0.01, 0.01});
    auto_declare<std::vector<bool>>(
      "navigation_feedback_axes", {false, false, true, true, true, true});

    auto_declare<double>("min_effective_inertia", 1e-3);
    auto_declare<double>("max_effective_inertia", 1e6);
    auto_declare<double>("min_linear_damping", 0.0);
    auto_declare<double>("max_linear_damping", 1e6);
    auto_declare<double>("min_quadratic_damping", 0.0);
    auto_declare<double>("max_quadratic_damping", 1e6);
    auto_declare<double>("min_static_wrench", -1e6);
    auto_declare<double>("max_static_wrench", 1e6);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
StateObserverController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
StateObserverController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn StateObserverController::on_configure(
  const rclcpp_lifecycle::State &)
{
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  force_topic_ = get_node()->get_parameter("force_topic").as_string();
  imu_topic_ = get_node()->get_parameter("imu_topic").as_string();
  aruco_pose_topic_ = get_node()->get_parameter("aruco_pose_topic").as_string();
  depth_pose_topic_ = get_node()->get_parameter("depth_pose_topic").as_string();
  predicted_pose_topic_ = get_node()->get_parameter("predicted_pose_topic").as_string();
  predicted_odometry_topic_ = get_node()->get_parameter("predicted_odometry_topic").as_string();
  model_topic_ = get_node()->get_parameter("model_topic").as_string();
  freeze_service_name_ = get_node()->get_parameter("freeze_service_name").as_string();
  reset_service_name_ = get_node()->get_parameter("reset_service_name").as_string();
  save_service_name_ = get_node()->get_parameter("save_service_name").as_string();
  load_service_name_ = get_node()->get_parameter("load_service_name").as_string();
  learn_service_name_ = get_node()->get_parameter("learn_service_name").as_string();
  predict_service_name_ = get_node()->get_parameter("predict_service_name").as_string();
  world_frame_id_ = get_node()->get_parameter("world_frame_id").as_string();
  base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
  model_file_ = get_node()->get_parameter("model_file").as_string();

  navigator_timeout_ = std::max(0.0, get_node()->get_parameter("navigator_timeout").as_double());
  force_timeout_ = std::max(0.0, get_node()->get_parameter("force_timeout").as_double());
  imu_timeout_ = std::max(0.0, get_node()->get_parameter("imu_timeout").as_double());
  aruco_pose_timeout_ =
    std::max(0.0, get_node()->get_parameter("aruco_pose_timeout").as_double());
  depth_pose_timeout_ =
    std::max(0.0, get_node()->get_parameter("depth_pose_timeout").as_double());
  forgetting_factor_ =
    std::clamp(get_node()->get_parameter("forgetting_factor").as_double(), 0.90, 1.0);
  min_excitation_ = std::max(0.0, get_node()->get_parameter("min_excitation").as_double());
  initial_estimator_covariance_ =
    std::max(1e-9, get_node()->get_parameter("initial_estimator_covariance").as_double());
  model_publish_period_ =
    std::max(0.0, get_node()->get_parameter("model_publish_period").as_double());
  max_prediction_dt_ = std::max(1e-4, get_node()->get_parameter("max_prediction_dt").as_double());
  weight_ = get_node()->get_parameter("weight").as_double();
  buoyancy_ = get_node()->get_parameter("buoyancy").as_double();
  use_imu_linear_acceleration_ =
    get_node()->get_parameter("use_imu_linear_acceleration").as_bool();

  if (navigator_topic_.empty() || force_topic_.empty() || imu_topic_.empty() ||
    aruco_pose_topic_.empty() || depth_pose_topic_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "StateObserverController requires navigator_topic, force_topic, imu_topic, and "
      "aruco_pose_topic, and depth_pose_topic parameters");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (base_frame_id_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "StateObserverController requires base_frame_id");
    return controller_interface::CallbackReturn::ERROR;
  }

  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    StateObserverModel::Limits limits;
    limits.min_effective_inertia = get_node()->get_parameter("min_effective_inertia").as_double();
    limits.max_effective_inertia = get_node()->get_parameter("max_effective_inertia").as_double();
    limits.min_linear_damping = get_node()->get_parameter("min_linear_damping").as_double();
    limits.max_linear_damping = get_node()->get_parameter("max_linear_damping").as_double();
    limits.min_quadratic_damping =
      get_node()->get_parameter("min_quadratic_damping").as_double();
    limits.max_quadratic_damping =
      get_node()->get_parameter("max_quadratic_damping").as_double();
    limits.min_static_wrench = get_node()->get_parameter("min_static_wrench").as_double();
    limits.max_static_wrench = get_node()->get_parameter("max_static_wrench").as_double();
    model_.setLimits(limits);
    model_.setRigidBodyInertia(
      readVector6Parameter("rigid_body_inertia", model_.rigidBodyInertia()));
    model_.setEffectiveInertia(
      readVector6Parameter("effective_inertia", model_.effectiveInertia()));
    model_.setLinearDamping(readVector6Parameter("linear_damping", model_.linearDamping()));
    model_.setQuadraticDamping(
      readVector6Parameter("quadratic_damping", model_.quadraticDamping()));
    model_.setStaticWrench(readVector6Parameter("static_wrench", model_.staticWrench()));
    model_.resetEstimator(initial_estimator_covariance_);
    model_.resetStatistics();
  }

  navigation_pose_covariance_ =
    readVector6Parameter("navigation_pose_covariance", navigation_pose_covariance_);
  process_noise_ = readVector6Parameter("process_noise", process_noise_);
  navigation_feedback_axes_ =
    readAxisFeedbackMaskParameter("navigation_feedback_axes", navigation_feedback_axes_);

  const std::string configured_mode = get_node()->get_parameter("mode").as_string();
  if (!setModeFromString(configured_mode)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid mode. Use 'learn' or 'predict'.");
    return controller_interface::CallbackReturn::ERROR;
  }

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
      last_navigator_time_ns_.store(get_node()->now().nanoseconds(), std::memory_order_relaxed);
    });

  force_sub_ = get_node()->create_subscription<WrenchMsg>(
    force_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const WrenchMsg::SharedPtr msg)
    {
      force_buffer_.writeFromNonRT(msg);
      last_force_time_ns_.store(get_node()->now().nanoseconds(), std::memory_order_relaxed);
    });

  imu_sub_ = get_node()->create_subscription<ImuMsg>(
    imu_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const ImuMsg::SharedPtr msg)
    {
      imu_buffer_.writeFromNonRT(msg);
      last_imu_time_ns_.store(get_node()->now().nanoseconds(), std::memory_order_relaxed);
    });

  aruco_pose_sub_ = get_node()->create_subscription<PoseWithCovarianceStampedMsg>(
    aruco_pose_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const PoseWithCovarianceStampedMsg::SharedPtr msg)
    {
      aruco_pose_buffer_.writeFromNonRT(msg);
      last_aruco_pose_time_ns_.store(get_node()->now().nanoseconds(), std::memory_order_relaxed);
    });

  depth_pose_sub_ = get_node()->create_subscription<PoseWithCovarianceStampedMsg>(
    depth_pose_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const PoseWithCovarianceStampedMsg::SharedPtr msg)
    {
      depth_pose_buffer_.writeFromNonRT(msg);
      last_depth_pose_time_ns_.store(get_node()->now().nanoseconds(), std::memory_order_relaxed);
    });

  predicted_pose_pub_ =
    get_node()->create_publisher<PoseWithCovarianceStampedMsg>(
    predicted_pose_topic_, rclcpp::SystemDefaultsQoS());
  predicted_pose_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<PoseWithCovarianceStampedMsg>>(
    predicted_pose_pub_);

  predicted_odometry_pub_ =
    get_node()->create_publisher<OdometryMsg>(
    predicted_odometry_topic_, rclcpp::SystemDefaultsQoS());
  predicted_odometry_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<OdometryMsg>>(
    predicted_odometry_pub_);

  model_pub_ = get_node()->create_publisher<ModelMsg>(model_topic_, rclcpp::SystemDefaultsQoS());
  model_rt_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<ModelMsg>>(model_pub_);

  freeze_service_ = get_node()->create_service<Trigger>(
    freeze_service_name_,
    [this](const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
    {
      handleFreeze(request, response);
    });
  reset_service_ = get_node()->create_service<Trigger>(
    reset_service_name_,
    [this](const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
    {
      handleReset(request, response);
    });
  save_service_ = get_node()->create_service<Trigger>(
    save_service_name_,
    [this](const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
    {
      handleSave(request, response);
    });
  load_service_ = get_node()->create_service<Trigger>(
    load_service_name_,
    [this](const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
    {
      handleLoad(request, response);
    });
  learn_mode_service_ = get_node()->create_service<Trigger>(
    learn_service_name_,
    [this](const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
    {
      handleSetLearnMode(request, response);
    });
  predict_mode_service_ = get_node()->create_service<Trigger>(
    predict_service_name_,
    [this](const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
    {
      handleSetPredictMode(request, response);
    });

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
    std::bind(&StateObserverController::parametersCallback, this, std::placeholders::_1));

  resetState();

  RCLCPP_INFO(get_node()->get_logger(), "Configured StateObserverController");
  RCLCPP_INFO(get_node()->get_logger(), "navigator_topic: %s", navigator_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "force_topic: %s", force_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "imu_topic: %s", imu_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "aruco_pose_topic: %s", aruco_pose_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "depth_pose_topic: %s", depth_pose_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "world_frame_id: %s", world_frame_id_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "mode: %s", modeString().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StateObserverController::on_activate(
  const rclcpp_lifecycle::State &)
{
  navigator_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>>(nullptr);
  force_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<WrenchMsg>>(nullptr);
  imu_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ImuMsg>>(nullptr);
  aruco_pose_buffer_ =
    realtime_tools::RealtimeBuffer<std::shared_ptr<PoseWithCovarianceStampedMsg>>(nullptr);
  depth_pose_buffer_ =
    realtime_tools::RealtimeBuffer<std::shared_ptr<PoseWithCovarianceStampedMsg>>(nullptr);
  last_navigator_time_ns_.store(0, std::memory_order_relaxed);
  last_force_time_ns_.store(0, std::memory_order_relaxed);
  last_imu_time_ns_.store(0, std::memory_order_relaxed);
  last_aruco_pose_time_ns_.store(0, std::memory_order_relaxed);
  last_depth_pose_time_ns_.store(0, std::memory_order_relaxed);
  last_state_time_ns_.store(0, std::memory_order_relaxed);
  last_model_publish_time_ns_.store(0, std::memory_order_relaxed);
  resetState();
  controller_active_.store(true, std::memory_order_relaxed);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StateObserverController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  controller_active_.store(false, std::memory_order_relaxed);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type StateObserverController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  if (!controller_active_.load(std::memory_order_relaxed)) {
    return controller_interface::return_type::OK;
  }

  const rclcpp::Time stamp = time.nanoseconds() > 0 ? time : get_node()->now();
  const int64_t now_ns = stamp.nanoseconds();
  const int64_t last_navigator_ns =
    last_navigator_time_ns_.load(std::memory_order_relaxed);
  const int64_t last_force_ns = last_force_time_ns_.load(std::memory_order_relaxed);
  const int64_t last_imu_ns = last_imu_time_ns_.load(std::memory_order_relaxed);
  const int64_t last_aruco_pose_ns =
    last_aruco_pose_time_ns_.load(std::memory_order_relaxed);
  const int64_t last_depth_pose_ns =
    last_depth_pose_time_ns_.load(std::memory_order_relaxed);
  const bool navigator_fresh =
    last_navigator_ns > 0 && ((now_ns - last_navigator_ns) * 1e-9) <= navigator_timeout_;
  const bool force_fresh =
    last_force_ns > 0 && ((now_ns - last_force_ns) * 1e-9) <= force_timeout_;
  const bool imu_fresh =
    last_imu_ns > 0 && ((now_ns - last_imu_ns) * 1e-9) <= imu_timeout_;
  const bool aruco_pose_fresh =
    last_aruco_pose_ns > 0 &&
    ((now_ns - last_aruco_pose_ns) * 1e-9) <= aruco_pose_timeout_;
  const bool depth_pose_fresh =
    last_depth_pose_ns > 0 &&
    ((now_ns - last_depth_pose_ns) * 1e-9) <= depth_pose_timeout_;

  auto navigator_msg = navigator_buffer_.readFromRT();
  auto force_msg = force_buffer_.readFromRT();
  auto imu_msg = imu_buffer_.readFromRT();
  auto aruco_pose_msg = aruco_pose_buffer_.readFromRT();
  auto depth_pose_msg = depth_pose_buffer_.readFromRT();
  const auto mode = static_cast<ObserverMode>(mode_.load(std::memory_order_relaxed));

  const bool has_fresh_navigation = navigator_fresh && navigator_msg && *navigator_msg;
  const bool should_correct_from_navigation = has_fresh_navigation && mode == ObserverMode::LEARN;

  if (should_correct_from_navigation) {
    correctStateFromNavigator(*(*navigator_msg));
    last_state_time_ns_.store(now_ns, std::memory_order_relaxed);
  }

  if (!state_.initialized && mode == ObserverMode::PREDICT) {
    state_.initialized = true;
    if (imu_fresh && imu_msg && *imu_msg) {
      correctPredictFeedbackFromImu(*(*imu_msg));
    }
    if (aruco_pose_fresh && aruco_pose_msg && *aruco_pose_msg) {
      correctPositionFromAruco(*(*aruco_pose_msg));
    } else if (depth_pose_fresh && depth_pose_msg && *depth_pose_msg) {
      correctDepthFromPose(*(*depth_pose_msg));
    }
    last_state_time_ns_.store(now_ns, std::memory_order_relaxed);
  }

  if (has_fresh_navigation) {
    if (
      mode == ObserverMode::LEARN &&
      !parameters_frozen_.load(std::memory_order_relaxed) &&
      force_fresh && force_msg && *force_msg)
    {
      const Vector6 body_wrench = wrenchToVector(*(*force_msg));
      const Vector6 body_velocity = navigatorBodyVelocityToVector(*(*navigator_msg));
      static Vector6 filtered_acceleration = Vector6::Zero();

      const Vector6 raw_acceleration =
        navigatorBodyAccelerationToVector(*(*navigator_msg));

      constexpr double alpha = 0.05;

      filtered_acceleration =
        alpha * raw_acceleration +
        (1.0 - alpha) * filtered_acceleration;

      const Vector6 body_acceleration = filtered_acceleration;
      const Vector6 restoring = restoringWrenchBody();
      std::lock_guard<std::mutex> lock(model_mutex_);
      model_.update(
        body_wrench,
        body_velocity,
        body_acceleration,
        restoring,
        forgetting_factor_,
        min_excitation_);
    }
  }

  if (!should_correct_from_navigation && state_.initialized) {
    const int64_t last_state_ns = last_state_time_ns_.load(std::memory_order_relaxed);
    const double raw_dt =
      last_state_ns > 0 ? (now_ns - last_state_ns) * 1e-9 : period.seconds();
    const double dt = std::clamp(raw_dt, 0.0, max_prediction_dt_);
    const WrenchMsg * prediction_wrench = force_msg && *force_msg ? force_msg->get() : nullptr;
    const ImuMsg * prediction_imu = imu_fresh && imu_msg && *imu_msg ? imu_msg->get() : nullptr;
    const PoseWithCovarianceStampedMsg * prediction_aruco =
      aruco_pose_fresh && aruco_pose_msg && *aruco_pose_msg ? aruco_pose_msg->get() : nullptr;
    const PoseWithCovarianceStampedMsg * prediction_depth =
      depth_pose_fresh && depth_pose_msg && *depth_pose_msg ? depth_pose_msg->get() : nullptr;
    predictState(prediction_wrench, prediction_imu, prediction_aruco, prediction_depth, dt);
    last_state_time_ns_.store(now_ns, std::memory_order_relaxed);
  }

  if (state_.initialized) {
    publishOutputs(stamp);
  }

  return controller_interface::return_type::OK;
}

rcl_interfaces::msg::SetParametersResult StateObserverController::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();

    if (name == "mode") {
      if (!setModeFromString(parameter.as_string())) {
        result.successful = false;
        result.reason = "mode must be 'learn' or 'predict'";
        return result;
      }
    } else if (name == "navigator_timeout") {
      navigator_timeout_ = std::max(0.0, parameter.as_double());
    } else if (name == "force_timeout") {
      force_timeout_ = std::max(0.0, parameter.as_double());
    } else if (name == "imu_timeout") {
      imu_timeout_ = std::max(0.0, parameter.as_double());
    } else if (name == "aruco_pose_timeout") {
      aruco_pose_timeout_ = std::max(0.0, parameter.as_double());
    } else if (name == "depth_pose_timeout") {
      depth_pose_timeout_ = std::max(0.0, parameter.as_double());
    } else if (name == "forgetting_factor") {
      forgetting_factor_ = std::clamp(parameter.as_double(), 0.90, 1.0);
    } else if (name == "min_excitation") {
      min_excitation_ = std::max(0.0, parameter.as_double());
    } else if (name == "model_publish_period") {
      model_publish_period_ = std::max(0.0, parameter.as_double());
    } else if (name == "max_prediction_dt") {
      max_prediction_dt_ = std::max(1e-4, parameter.as_double());
    } else if (name == "weight") {
      weight_ = parameter.as_double();
    } else if (name == "buoyancy") {
      buoyancy_ = parameter.as_double();
    } else if (name == "use_imu_linear_acceleration") {
      use_imu_linear_acceleration_ = parameter.as_bool();
    } else if (name == "navigation_feedback_axes") {
      const auto values = parameter.as_bool_array();
      if (values.size() != StateObserverModel::kAxisCount) {
        result.successful = false;
        result.reason = "navigation_feedback_axes must contain 6 boolean values";
        return result;
      }
      for (std::size_t axis = 0; axis < StateObserverModel::kAxisCount; ++axis) {
        navigation_feedback_axes_[axis] = values[axis];
      }
    }
  }

  return result;
}

Eigen::Quaterniond StateObserverController::normalizeQuaternion(
  const Eigen::Quaterniond & quaternion)
{
  if (!std::isfinite(quaternion.norm()) || quaternion.squaredNorm() < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  Eigen::Quaterniond normalized = quaternion;
  normalized.normalize();
  return normalized;
}

Eigen::Vector3d StateObserverController::quaternionToRpy(const Eigen::Quaterniond & quaternion)
{
  return quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
}

Eigen::Quaterniond StateObserverController::rpyToQuaternion(const Eigen::Vector3d & rpy)
{
  const Eigen::AngleAxisd roll(rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy.z(), Eigen::Vector3d::UnitZ());
  return normalizeQuaternion(yaw * pitch * roll);
}

Eigen::Vector3d StateObserverController::enuPositionToNed(
  const geometry_msgs::msg::Point & position)
{
  return Eigen::Vector3d(position.y, position.x, -position.z);
}

StateObserverController::Vector6 StateObserverController::wrenchToVector(
  const WrenchMsg & wrench_msg)
{
  Vector6 wrench;
  wrench <<
    wrench_msg.force.x,
    wrench_msg.force.y,
    wrench_msg.force.z,
    wrench_msg.torque.x,
    wrench_msg.torque.y,
    wrench_msg.torque.z;
  return wrench;
}

StateObserverController::Vector6 StateObserverController::navigatorBodyVelocityToVector(
  const NavigatorMsg & navigator_msg)
{
  Vector6 velocity;
  velocity <<
    navigator_msg.body_velocity.linear.x,
    navigator_msg.body_velocity.linear.y,
    navigator_msg.body_velocity.linear.z,
    navigator_msg.body_velocity.angular.x,
    navigator_msg.body_velocity.angular.y,
    navigator_msg.body_velocity.angular.z;
  return velocity;
}

StateObserverController::Vector6 StateObserverController::navigatorBodyAccelerationToVector(
  const NavigatorMsg & navigator_msg)
{
  Vector6 acceleration;
  acceleration <<
    navigator_msg.body_acceleration.linear.x,
    navigator_msg.body_acceleration.linear.y,
    navigator_msg.body_acceleration.linear.z,
    navigator_msg.body_acceleration.angular.x,
    navigator_msg.body_acceleration.angular.y,
    navigator_msg.body_acceleration.angular.z;
  return acceleration;
}

StateObserverController::Vector6 StateObserverController::imuBodyAccelerationToVector(
  const ImuMsg & imu_msg)
{
  Vector6 acceleration;
  acceleration <<
    imu_msg.linear_acceleration.x,
    imu_msg.linear_acceleration.y,
    imu_msg.linear_acceleration.z,
    0.0,
    0.0,
    0.0;
  return acceleration;
}

void StateObserverController::copyVectorToArray(
  const Vector6 & vector,
  std::array<double, 6> & array)
{
  for (std::size_t axis = 0; axis < StateObserverModel::kAxisCount; ++axis) {
    array[axis] = vector(static_cast<Eigen::Index>(axis));
  }
}

StateObserverController::Vector6 StateObserverController::readVector6Parameter(
  const std::string & name,
  const Vector6 & fallback) const
{
  const auto values = get_node()->get_parameter(name).as_double_array();
  if (values.size() != StateObserverModel::kAxisCount) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Parameter '%s' must contain 6 values. Keeping fallback.",
      name.c_str());
    return fallback;
  }

  Vector6 vector;
  for (std::size_t axis = 0; axis < StateObserverModel::kAxisCount; ++axis) {
    vector(static_cast<Eigen::Index>(axis)) = values[axis];
  }
  return vector;
}

StateObserverController::AxisFeedbackMask StateObserverController::readAxisFeedbackMaskParameter(
  const std::string & name,
  const AxisFeedbackMask & fallback) const
{
  const auto values = get_node()->get_parameter(name).as_bool_array();
  if (values.size() != StateObserverModel::kAxisCount) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Parameter '%s' must contain 6 boolean values. Keeping fallback.",
      name.c_str());
    return fallback;
  }

  AxisFeedbackMask mask = fallback;
  for (std::size_t axis = 0; axis < StateObserverModel::kAxisCount; ++axis) {
    mask[axis] = values[axis];
  }
  return mask;
}

bool StateObserverController::setModeFromString(const std::string & mode)
{
  if (mode == "learn") {
    mode_.store(static_cast<int>(ObserverMode::LEARN), std::memory_order_relaxed);
    parameters_frozen_.store(false, std::memory_order_relaxed);
    return true;
  }
  if (mode == "predict" || mode == "frozen") {
    mode_.store(static_cast<int>(ObserverMode::PREDICT), std::memory_order_relaxed);
    return true;
  }
  return false;
}

std::string StateObserverController::modeString() const
{
  const auto mode = static_cast<ObserverMode>(mode_.load(std::memory_order_relaxed));
  if (mode == ObserverMode::LEARN) {
    return "learn";
  }
  return "predict";
}

void StateObserverController::resetState()
{
  state_ = ObserverState{};
}

void StateObserverController::correctStateFromNavigator(const NavigatorMsg & navigator_msg)
{
  state_.position_ned <<
    navigator_msg.position.position.x,
    navigator_msg.position.position.y,
    navigator_msg.position.position.z;

  state_.orientation_body_to_ned = normalizeQuaternion(
    Eigen::Quaterniond(
      navigator_msg.position.orientation.w,
      navigator_msg.position.orientation.x,
      navigator_msg.position.orientation.y,
      navigator_msg.position.orientation.z));

  state_.body_velocity = navigatorBodyVelocityToVector(navigator_msg);
  state_.pose_covariance = navigation_pose_covariance_;
  state_.initialized = true;
}

void StateObserverController::correctPredictFeedbackFromNavigator(const NavigatorMsg & navigator_msg)
{
  if (navigation_feedback_axes_[0]) {
    state_.position_ned.x() = navigator_msg.position.position.x;
  }
  if (navigation_feedback_axes_[1]) {
    state_.position_ned.y() = navigator_msg.position.position.y;
  }
  if (navigation_feedback_axes_[2]) {
    state_.position_ned.z() = navigator_msg.position.position.z;
  }

  Eigen::Vector3d rpy = quaternionToRpy(state_.orientation_body_to_ned);
  if (navigation_feedback_axes_[3]) {
    rpy.x() = navigator_msg.rpy.x;
  }
  if (navigation_feedback_axes_[4]) {
    rpy.y() = navigator_msg.rpy.y;
  }
  if (navigation_feedback_axes_[5]) {
    rpy.z() = navigator_msg.rpy.z;
  }
  state_.orientation_body_to_ned = rpyToQuaternion(rpy);

  const Vector6 measured_velocity = navigatorBodyVelocityToVector(navigator_msg);
  for (std::size_t axis = 0; axis < StateObserverModel::kAxisCount; ++axis) {
    if (navigation_feedback_axes_[axis]) {
      state_.body_velocity(static_cast<Eigen::Index>(axis)) =
        measured_velocity(static_cast<Eigen::Index>(axis));
      state_.pose_covariance(static_cast<Eigen::Index>(axis)) =
        navigation_pose_covariance_(static_cast<Eigen::Index>(axis));
    }
  }
}

void StateObserverController::correctPredictFeedbackFromImu(const ImuMsg & imu_msg)
{
  state_.orientation_body_to_ned = normalizeQuaternion(
    Eigen::Quaterniond(
      imu_msg.orientation.w,
      imu_msg.orientation.x,
      imu_msg.orientation.y,
      imu_msg.orientation.z));

  state_.body_velocity(3) = imu_msg.angular_velocity.x;
  state_.body_velocity(4) = imu_msg.angular_velocity.y;
  state_.body_velocity(5) = imu_msg.angular_velocity.z;

  state_.pose_covariance(3) = navigation_pose_covariance_(3);
  state_.pose_covariance(4) = navigation_pose_covariance_(4);
  state_.pose_covariance(5) = navigation_pose_covariance_(5);
}

void StateObserverController::correctPositionFromAruco(
  const PoseWithCovarianceStampedMsg & aruco_msg)
{
  state_.position_ned = enuPositionToNed(aruco_msg.pose.pose.position);
  state_.pose_covariance(0) = aruco_msg.pose.covariance[7];
  state_.pose_covariance(1) = aruco_msg.pose.covariance[0];
  state_.pose_covariance(2) = aruco_msg.pose.covariance[14];
}

void StateObserverController::correctDepthFromPose(
  const PoseWithCovarianceStampedMsg & depth_msg)
{
  state_.position_ned.z() = -depth_msg.pose.pose.position.z;
  state_.body_velocity(2) = 0.0;
  state_.pose_covariance(2) = depth_msg.pose.covariance[14];
}

void StateObserverController::predictState(
  const WrenchMsg * wrench_msg,
  const ImuMsg * imu_msg,
  const PoseWithCovarianceStampedMsg * aruco_msg,
  const PoseWithCovarianceStampedMsg * depth_msg,
  double dt)
{
  if (dt <= 0.0) {
    return;
  }

  const Vector6 body_wrench = wrench_msg == nullptr ? Vector6::Zero() : wrenchToVector(*wrench_msg);
  const Vector6 restoring = restoringWrenchBody();
  Vector6 acceleration;
  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    acceleration = model_.predictAcceleration(body_wrench, state_.body_velocity, restoring);
  }

  if (imu_msg != nullptr && use_imu_linear_acceleration_) {
    const Vector6 measured_acceleration = imuBodyAccelerationToVector(*imu_msg);
    acceleration.template head<3>() = measured_acceleration.template head<3>();
  }

  state_.body_velocity += acceleration * dt;
  state_.position_ned +=
    state_.orientation_body_to_ned * state_.body_velocity.template head<3>() * dt;

  const Eigen::Vector3d angular_velocity = state_.body_velocity.template tail<3>();
  const double angular_speed = angular_velocity.norm();
  if (angular_speed > 1e-9) {
    const Eigen::AngleAxisd delta_rotation(angular_speed * dt, angular_velocity / angular_speed);
    state_.orientation_body_to_ned =
      normalizeQuaternion(state_.orientation_body_to_ned * Eigen::Quaterniond(delta_rotation));
  }

  state_.pose_covariance += process_noise_ * dt;

  if (imu_msg != nullptr) {
    correctPredictFeedbackFromImu(*imu_msg);
  }

  if (aruco_msg != nullptr) {
    correctPositionFromAruco(*aruco_msg);
  } else if (depth_msg != nullptr) {
    correctDepthFromPose(*depth_msg);
  }
}

StateObserverController::Vector6 StateObserverController::restoringWrenchBody() const
{
  Vector6 restoring;
  restoring.setZero();

  const Eigen::Vector3d net_weight_ned(0.0, 0.0, weight_ - buoyancy_);
  const Eigen::Vector3d net_weight_body =
    state_.orientation_body_to_ned.inverse() * net_weight_ned;
  restoring.template head<3>() = net_weight_body;
  return restoring;
}

void StateObserverController::publishState(const rclcpp::Time & stamp)
{
  if (predicted_pose_rt_pub_ && predicted_pose_rt_pub_->trylock()) {
    auto & msg = predicted_pose_rt_pub_->msg_;
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_id_;
    msg.pose.pose.position.x = state_.position_ned.x();
    msg.pose.pose.position.y = state_.position_ned.y();
    msg.pose.pose.position.z = state_.position_ned.z();
    msg.pose.pose.orientation.x = state_.orientation_body_to_ned.x();
    msg.pose.pose.orientation.y = state_.orientation_body_to_ned.y();
    msg.pose.pose.orientation.z = state_.orientation_body_to_ned.z();
    msg.pose.pose.orientation.w = state_.orientation_body_to_ned.w();
    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
    msg.pose.covariance[0] = state_.pose_covariance(0);
    msg.pose.covariance[7] = state_.pose_covariance(1);
    msg.pose.covariance[14] = state_.pose_covariance(2);
    msg.pose.covariance[21] = state_.pose_covariance(3);
    msg.pose.covariance[28] = state_.pose_covariance(4);
    msg.pose.covariance[35] = state_.pose_covariance(5);
    predicted_pose_rt_pub_->unlockAndPublish();
  }

  if (predicted_odometry_rt_pub_ && predicted_odometry_rt_pub_->trylock()) {
    auto & msg = predicted_odometry_rt_pub_->msg_;
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_id_;
    msg.child_frame_id = base_frame_id_;
    msg.pose.pose.position.x = state_.position_ned.x();
    msg.pose.pose.position.y = state_.position_ned.y();
    msg.pose.pose.position.z = state_.position_ned.z();
    msg.pose.pose.orientation.x = state_.orientation_body_to_ned.x();
    msg.pose.pose.orientation.y = state_.orientation_body_to_ned.y();
    msg.pose.pose.orientation.z = state_.orientation_body_to_ned.z();
    msg.pose.pose.orientation.w = state_.orientation_body_to_ned.w();
    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
    msg.pose.covariance[0] = state_.pose_covariance(0);
    msg.pose.covariance[7] = state_.pose_covariance(1);
    msg.pose.covariance[14] = state_.pose_covariance(2);
    msg.pose.covariance[21] = state_.pose_covariance(3);
    msg.pose.covariance[28] = state_.pose_covariance(4);
    msg.pose.covariance[35] = state_.pose_covariance(5);
    msg.twist.twist.linear.x = state_.body_velocity(0);
    msg.twist.twist.linear.y = state_.body_velocity(1);
    msg.twist.twist.linear.z = state_.body_velocity(2);
    msg.twist.twist.angular.x = state_.body_velocity(3);
    msg.twist.twist.angular.y = state_.body_velocity(4);
    msg.twist.twist.angular.z = state_.body_velocity(5);
    predicted_odometry_rt_pub_->unlockAndPublish();
  }
}

void StateObserverController::publishModel(const rclcpp::Time & stamp)
{
  if (!model_rt_pub_ || !model_rt_pub_->trylock()) {
    return;
  }

  auto & msg = model_rt_pub_->msg_;
  msg.header.stamp = stamp;
  msg.header.frame_id = world_frame_id_;
  msg.mode = modeString();
  msg.body_frame_id = base_frame_id_;

  std::lock_guard<std::mutex> lock(model_mutex_);
  msg.sample_count = model_.sampleCount();
  copyVectorToArray(model_.rigidBodyInertia(), msg.rigid_body_inertia);
  copyVectorToArray(model_.effectiveInertia(), msg.effective_inertia);
  copyVectorToArray(model_.addedMass(), msg.added_mass);
  copyVectorToArray(model_.linearDamping(), msg.linear_damping);
  copyVectorToArray(model_.quadraticDamping(), msg.quadratic_damping);
  copyVectorToArray(model_.staticWrench(), msg.static_wrench);
  copyVectorToArray(model_.effectiveInertiaVariance(), msg.effective_inertia_variance);
  copyVectorToArray(model_.linearDampingVariance(), msg.linear_damping_variance);
  copyVectorToArray(model_.quadraticDampingVariance(), msg.quadratic_damping_variance);
  copyVectorToArray(model_.staticWrenchVariance(), msg.static_wrench_variance);
  copyVectorToArray(model_.residualRms(), msg.residual_rms);
  model_rt_pub_->unlockAndPublish();
}

void StateObserverController::publishOutputs(const rclcpp::Time & stamp)
{
  publishState(stamp);

  const int64_t now_ns = stamp.nanoseconds();
  const int64_t last_publish_ns = last_model_publish_time_ns_.load(std::memory_order_relaxed);
  const bool should_publish_model =
    model_publish_period_ <= 0.0 ||
    last_publish_ns == 0 ||
    ((now_ns - last_publish_ns) * 1e-9) >= model_publish_period_;

  if (should_publish_model) {
    publishModel(stamp);
    last_model_publish_time_ns_.store(now_ns, std::memory_order_relaxed);
  }
}

bool StateObserverController::saveModel(std::string & message) const
{
  if (model_file_.empty()) {
    message = "model_file parameter is empty";
    return false;
  }

  YAML::Node root;
  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    root["mode"] = modeString();
    root["body_frame_id"] = base_frame_id_;
    root["rigid_body_inertia"] = vectorToYaml(model_.rigidBodyInertia());
    root["effective_inertia"] = vectorToYaml(model_.effectiveInertia());
    root["linear_damping"] = vectorToYaml(model_.linearDamping());
    root["quadratic_damping"] = vectorToYaml(model_.quadraticDamping());
    root["static_wrench"] = vectorToYaml(model_.staticWrench());
  }

  std::ofstream file(model_file_);
  if (!file.is_open()) {
    message = "failed to open " + model_file_;
    return false;
  }

  file << root;
  message = "saved model to " + model_file_;
  return true;
}

bool StateObserverController::loadModel(std::string & message)
{
  if (model_file_.empty()) {
    message = "model_file parameter is empty";
    return false;
  }

  try {
    const YAML::Node root = YAML::LoadFile(model_file_);
    Vector6 rigid_body_inertia;
    Vector6 effective_inertia;
    Vector6 linear_damping;
    Vector6 quadratic_damping;
    Vector6 static_wrench = Vector6::Zero();

    if (!readYamlVector6(root, "rigid_body_inertia", rigid_body_inertia) ||
      !readYamlVector6(root, "effective_inertia", effective_inertia) ||
      !readYamlVector6(root, "linear_damping", linear_damping) ||
      !readYamlVector6(root, "quadratic_damping", quadratic_damping))
    {
      message = "model file does not contain all required 6-axis arrays";
      return false;
    }
    readYamlVector6(root, "static_wrench", static_wrench);

    std::lock_guard<std::mutex> lock(model_mutex_);
    model_.setRigidBodyInertia(rigid_body_inertia);
    model_.setEffectiveInertia(effective_inertia);
    model_.setLinearDamping(linear_damping);
    model_.setQuadraticDamping(quadratic_damping);
    model_.setStaticWrench(static_wrench);
    model_.resetEstimator(initial_estimator_covariance_);
    message = "loaded model from " + model_file_;
    return true;
  } catch (const std::exception & e) {
    message = std::string("failed to load model: ") + e.what();
    return false;
  }
}

bool StateObserverController::handleFreeze(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  parameters_frozen_.store(true, std::memory_order_relaxed);
  response->success = true;
  response->message = "state observer parameters frozen";
  return true;
}

bool StateObserverController::handleReset(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  resetState();
  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    model_.resetEstimator(initial_estimator_covariance_);
    model_.resetStatistics();
  }
  response->success = true;
  response->message = "state observer reset";
  return true;
}

bool StateObserverController::handleSave(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = saveModel(response->message);
  return true;
}

bool StateObserverController::handleLoad(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = loadModel(response->message);
  return true;
}

bool StateObserverController::handleSetLearnMode(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  setModeFromString("learn");
  response->success = true;
  response->message = "state observer switched to learn mode";
  return true;
}

bool StateObserverController::handleSetPredictMode(
  const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  setModeFromString("predict");
  response->success = true;
  response->message = "state observer switched to predict mode";
  return true;
}

}  // namespace sura_controllers::auv

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::auv::StateObserverController,
  controller_interface::ControllerInterface)
