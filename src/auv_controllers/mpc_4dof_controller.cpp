#include "sura_controllers/auv/mpc_4dof_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "osqp.h"
#include "pluginlib/class_list_macros.hpp"

namespace sura_controllers::auv
{

namespace
{

constexpr double kMinInertia = 1e-6;
constexpr double kEpsilon = 1e-9;

template<typename Derived>
void clampInPlace(
  Eigen::MatrixBase<Derived> & value,
  const Eigen::Vector4d & lower,
  const Eigen::Vector4d & upper)
{
  for (Eigen::Index i = 0; i < value.rows(); ++i) {
    value(i) = std::clamp(value(i), lower(i), upper(i));
  }
}

void denseUpperToCsc(
  const Eigen::MatrixXd & matrix,
  std::vector<c_float> & values,
  std::vector<c_int> & row_indices,
  std::vector<c_int> & col_pointers)
{
  values.clear();
  row_indices.clear();
  col_pointers.assign(static_cast<size_t>(matrix.cols() + 1), 0);

  for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
    col_pointers[static_cast<size_t>(col)] = static_cast<c_int>(values.size());
    for (Eigen::Index row = 0; row <= col && row < matrix.rows(); ++row) {
      const double value = matrix(row, col);
      if (std::abs(value) > 1e-12) {
        values.push_back(static_cast<c_float>(value));
        row_indices.push_back(static_cast<c_int>(row));
      }
    }
  }
  col_pointers[static_cast<size_t>(matrix.cols())] = static_cast<c_int>(values.size());
}

void denseToCsc(
  const Eigen::MatrixXd & matrix,
  std::vector<c_float> & values,
  std::vector<c_int> & row_indices,
  std::vector<c_int> & col_pointers)
{
  values.clear();
  row_indices.clear();
  col_pointers.assign(static_cast<size_t>(matrix.cols() + 1), 0);

  for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
    col_pointers[static_cast<size_t>(col)] = static_cast<c_int>(values.size());
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      const double value = matrix(row, col);
      if (std::abs(value) > 1e-12) {
        values.push_back(static_cast<c_float>(value));
        row_indices.push_back(static_cast<c_int>(row));
      }
    }
  }
  col_pointers[static_cast<size_t>(matrix.cols())] = static_cast<c_int>(values.size());
}

}  // namespace

bool Mpc4dofController::MpcQpSolver::configure(
  int configured_horizon,
  double configured_dt,
  const Eigen::Vector4d & configured_wrench_min,
  const Eigen::Vector4d & configured_wrench_max,
  const Eigen::Vector4d & configured_wrench_rate_limit,
  const StateVector & configured_q_state,
  const InputVector & configured_r_input,
  const InputVector & configured_s_delta_input,
  double configured_terminal_weight_scale,
  int configured_max_iterations,
  double configured_time_limit_ms)
{
  horizon = std::max(1, configured_horizon);
  dt = std::max(1e-4, configured_dt);
  wrench_min = configured_wrench_min;
  wrench_max = configured_wrench_max;
  wrench_rate_limit = configured_wrench_rate_limit.cwiseMax(Eigen::Vector4d::Zero());
  q_state = configured_q_state.cwiseMax(StateVector::Zero());
  r_input = configured_r_input.cwiseMax(InputVector::Constant(kEpsilon));
  s_delta_input = configured_s_delta_input.cwiseMax(InputVector::Zero());
  terminal_weight_scale = std::max(0.0, configured_terminal_weight_scale);
  max_iterations = std::max(1, configured_max_iterations);
  time_limit_ms = std::max(0.0, configured_time_limit_ms);
  warm_start = Eigen::VectorXd::Zero(kInputSize * horizon);
  return true;
}

bool Mpc4dofController::MpcQpSolver::solve(
  const StateVector & x0,
  const std::vector<StateMatrix> & A_seq,
  const std::vector<InputMatrix> & B_seq,
  const std::vector<StateVector> & c_seq,
  const std::vector<StateVector> & x_ref_seq,
  const MpcModel &,
  const InputVector & previous_tau,
  InputVector & tau0_solution)
{
  if (
    static_cast<int>(A_seq.size()) != horizon ||
    static_cast<int>(B_seq.size()) != horizon ||
    static_cast<int>(c_seq.size()) != horizon ||
    static_cast<int>(x_ref_seq.size()) < horizon + 1)
  {
    return false;
  }

  const int variable_count = kInputSize * horizon;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(variable_count, variable_count);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(variable_count);
  Eigen::MatrixXd state_sensitivity = Eigen::MatrixXd::Zero(kStateSize, variable_count);
  StateVector affine_state = x0;
  const StateVector q_terminal = terminal_weight_scale * q_state;

  for (int step = 0; step < horizon; ++step) {
    Eigen::MatrixXd next_sensitivity = A_seq[static_cast<size_t>(step)] * state_sensitivity;
    next_sensitivity.block<kStateSize, kInputSize>(0, kInputSize * step) +=
      B_seq[static_cast<size_t>(step)];
    affine_state =
      A_seq[static_cast<size_t>(step)] * affine_state + c_seq[static_cast<size_t>(step)];

    const StateVector weights = step == horizon - 1 ? q_terminal : q_state;
    const Eigen::Matrix<double, kStateSize, kStateSize> Q = weights.asDiagonal();
    const StateVector error = affine_state - x_ref_seq[static_cast<size_t>(step + 1)];

    H.noalias() += 2.0 * next_sensitivity.transpose() * Q * next_sensitivity;
    g.noalias() += 2.0 * next_sensitivity.transpose() * Q * error;
    state_sensitivity = next_sensitivity;
  }

  const Eigen::Matrix4d R = r_input.asDiagonal();
  const Eigen::Matrix4d S = s_delta_input.asDiagonal();
  for (int step = 0; step < horizon; ++step) {
    const int offset = kInputSize * step;
    H.block<kInputSize, kInputSize>(offset, offset) += 2.0 * R;

    if (step == 0) {
      H.block<kInputSize, kInputSize>(offset, offset) += 2.0 * S;
      g.segment<kInputSize>(offset).noalias() -= 2.0 * S * previous_tau;
    } else {
      const int previous_offset = kInputSize * (step - 1);
      H.block<kInputSize, kInputSize>(offset, offset) += 2.0 * S;
      H.block<kInputSize, kInputSize>(previous_offset, previous_offset) += 2.0 * S;
      H.block<kInputSize, kInputSize>(offset, previous_offset) -= 2.0 * S;
      H.block<kInputSize, kInputSize>(previous_offset, offset) -= 2.0 * S;
    }
  }

  H.diagonal().array() += kEpsilon;

  if (warm_start.size() != variable_count) {
    warm_start = Eigen::VectorXd::Zero(variable_count);
    for (int step = 0; step < horizon; ++step) {
      warm_start.segment<kInputSize>(kInputSize * step) = previous_tau;
    }
  } else if (horizon > 1) {
    for (int step = 0; step < horizon - 1; ++step) {
      warm_start.segment<kInputSize>(kInputSize * step) =
        warm_start.segment<kInputSize>(kInputSize * (step + 1));
    }
    warm_start.segment<kInputSize>(kInputSize * (horizon - 1)) =
      warm_start.segment<kInputSize>(kInputSize * (horizon - 2));
  }

  auto project = [this, &previous_tau](Eigen::VectorXd & u_sequence)
  {
    for (int pass = 0; pass < 3; ++pass) {
      InputVector previous = previous_tau;
      for (int step = 0; step < horizon; ++step) {
        InputVector current = u_sequence.segment<kInputSize>(kInputSize * step);
        clampInPlace(current, wrench_min, wrench_max);
        InputVector delta = current - previous;
        clampInPlace(delta, -wrench_rate_limit, wrench_rate_limit);
        current = previous + delta;
        clampInPlace(current, wrench_min, wrench_max);
        u_sequence.segment<kInputSize>(kInputSize * step) = current;
        previous = current;
      }

      for (int step = horizon - 2; step >= 0; --step) {
        const InputVector next = u_sequence.segment<kInputSize>(kInputSize * (step + 1));
        InputVector current = u_sequence.segment<kInputSize>(kInputSize * step);
        clampInPlace(current, next - wrench_rate_limit, next + wrench_rate_limit);
        clampInPlace(current, wrench_min, wrench_max);
        u_sequence.segment<kInputSize>(kInputSize * step) = current;
      }
    }
  };

  Eigen::VectorXd u_sequence = warm_start;
  project(u_sequence);

  Eigen::MatrixXd A_constraints = Eigen::MatrixXd::Zero(2 * variable_count, variable_count);
  Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(2 * variable_count);
  Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(2 * variable_count);

  for (int step = 0; step < horizon; ++step) {
    for (int axis = 0; axis < kInputSize; ++axis) {
      const int variable = kInputSize * step + axis;
      A_constraints(variable, variable) = 1.0;
      lower_bound(variable) = wrench_min(axis);
      upper_bound(variable) = wrench_max(axis);

      const int rate_row = variable_count + variable;
      A_constraints(rate_row, variable) = 1.0;
      if (step == 0) {
        lower_bound(rate_row) = previous_tau(axis) - wrench_rate_limit(axis);
        upper_bound(rate_row) = previous_tau(axis) + wrench_rate_limit(axis);
      } else {
        A_constraints(rate_row, variable - kInputSize) = -1.0;
        lower_bound(rate_row) = -wrench_rate_limit(axis);
        upper_bound(rate_row) = wrench_rate_limit(axis);
      }
    }
  }

  std::vector<c_float> p_values;
  std::vector<c_int> p_rows;
  std::vector<c_int> p_cols;
  std::vector<c_float> a_values;
  std::vector<c_int> a_rows;
  std::vector<c_int> a_cols;
  denseUpperToCsc(0.5 * (H + H.transpose()), p_values, p_rows, p_cols);
  denseToCsc(A_constraints, a_values, a_rows, a_cols);

  std::vector<c_float> q_values(static_cast<size_t>(variable_count), 0.0);
  std::vector<c_float> l_values(static_cast<size_t>(2 * variable_count), 0.0);
  std::vector<c_float> u_values(static_cast<size_t>(2 * variable_count), 0.0);
  for (int i = 0; i < variable_count; ++i) {
    q_values[static_cast<size_t>(i)] = static_cast<c_float>(g(i));
  }
  for (int i = 0; i < 2 * variable_count; ++i) {
    l_values[static_cast<size_t>(i)] = static_cast<c_float>(lower_bound(i));
    u_values[static_cast<size_t>(i)] = static_cast<c_float>(upper_bound(i));
  }

  csc p_matrix;
  p_matrix.nzmax = static_cast<c_int>(p_values.size());
  p_matrix.m = static_cast<c_int>(variable_count);
  p_matrix.n = static_cast<c_int>(variable_count);
  p_matrix.p = p_cols.data();
  p_matrix.i = p_rows.data();
  p_matrix.x = p_values.data();
  p_matrix.nz = -1;

  csc a_matrix;
  a_matrix.nzmax = static_cast<c_int>(a_values.size());
  a_matrix.m = static_cast<c_int>(2 * variable_count);
  a_matrix.n = static_cast<c_int>(variable_count);
  a_matrix.p = a_cols.data();
  a_matrix.i = a_rows.data();
  a_matrix.x = a_values.data();
  a_matrix.nz = -1;

  OSQPData data;
  data.n = static_cast<c_int>(variable_count);
  data.m = static_cast<c_int>(2 * variable_count);
  data.P = &p_matrix;
  data.A = &a_matrix;
  data.q = q_values.data();
  data.l = l_values.data();
  data.u = u_values.data();

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polish = 0;
  settings.warm_start = 1;
  settings.max_iter = static_cast<c_int>(max_iterations);
  settings.eps_abs = 1e-4;
  settings.eps_rel = 1e-4;
#ifdef PROFILING
  settings.time_limit = static_cast<c_float>(time_limit_ms * 1e-3);
#endif

  OSQPWorkspace * workspace = nullptr;
  const c_int setup_status = osqp_setup(&workspace, &data, &settings);
  if (setup_status != 0 || workspace == nullptr) {
    return false;
  }

  std::vector<c_float> warm_start_values(static_cast<size_t>(variable_count), 0.0);
  for (int i = 0; i < variable_count; ++i) {
    warm_start_values[static_cast<size_t>(i)] = static_cast<c_float>(u_sequence(i));
  }
  osqp_warm_start_x(workspace, warm_start_values.data());

  const c_int solve_status = osqp_solve(workspace);
  const bool solved =
    solve_status == 0 &&
    workspace->info &&
    (workspace->info->status_val == OSQP_SOLVED ||
    workspace->info->status_val == OSQP_SOLVED_INACCURATE) &&
    workspace->solution &&
    workspace->solution->x;

  if (!solved) {
    osqp_cleanup(workspace);
    return false;
  }

  for (int i = 0; i < variable_count; ++i) {
    u_sequence(i) = static_cast<double>(workspace->solution->x[i]);
  }
  osqp_cleanup(workspace);

  if (!u_sequence.allFinite()) {
    return false;
  }

  project(u_sequence);
  warm_start = u_sequence;
  tau0_solution = u_sequence.segment<kInputSize>(0);
  return true;
}

controller_interface::CallbackReturn Mpc4dofController::on_init()
{
  try {
    auto_declare<std::string>("navigator_topic", "/cirtesub/navigator/navigation");
    auto_declare<std::string>("setpoint_topic", "/cirtesub/controller/mpc_4dof/setpoint");
    auto_declare<std::string>("body_force_controller_name", "body_force");
    auto_declare<std::string>("world_frame_id", "world_ned");
    auto_declare<std::string>("base_frame_id", "");
    auto_declare<int>("prediction_horizon", 10);
    auto_declare<double>("prediction_dt", 0.05);
    auto_declare<double>("mpc_rate", 20.0);
    auto_declare<std::vector<double>>("q_state", {5.0, 5.0, 10.0, 4.0, 0.5, 0.5, 1.0, 0.4});
    auto_declare<std::vector<double>>("r_input", {0.05, 0.05, 0.05, 0.03});
    auto_declare<std::vector<double>>("s_delta_input", {0.2, 0.2, 0.2, 0.1});
    auto_declare<double>("terminal_weight_scale", 5.0);
    auto_declare<std::vector<double>>("wrench_min", {-40.0, -40.0, -40.0, -8.0});
    auto_declare<std::vector<double>>("wrench_max", {40.0, 40.0, 40.0, 8.0});
    auto_declare<std::vector<double>>("wrench_rate_limit", {5.0, 5.0, 5.0, 1.0});
    auto_declare<double>("navigator_timeout", 0.5);
    auto_declare<double>("setpoint_timeout", 1.0);
    auto_declare<int>("solver_max_iterations", 50);
    auto_declare<double>("solver_time_limit_ms", 8.0);
    auto_declare<std::string>("fallback_mode", "hold_previous");
    auto_declare<std::vector<bool>>("mpc_control_axes", {true, true, true, true});
    auto_declare<std::vector<double>>("inertia", {17.0, 24.2, 26.07, 0.28});
    auto_declare<std::vector<double>>("linear_damping", {4.03, 6.22, 5.18, 0.07});
    auto_declare<std::vector<double>>("quadratic_damping", {18.18, 21.66, 36.99, 1.55});
    auto_declare<std::vector<double>>("static_wrench", {0.0, 0.0, 0.0, 0.0});
    auto_declare<double>("restoring_z", 0.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Mpc4dofController::command_interface_configuration() const
{
  const std::string body_force_name = get_node()->has_parameter("body_force_controller_name") ?
    get_node()->get_parameter("body_force_controller_name").as_string() :
    std::string("body_force");

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      body_force_name + "/force.x",
      body_force_name + "/force.y",
      body_force_name + "/force.z",
      body_force_name + "/torque.x",
      body_force_name + "/torque.y",
      body_force_name + "/torque.z"
    }
  };
}

controller_interface::InterfaceConfiguration
Mpc4dofController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn Mpc4dofController::on_configure(
  const rclcpp_lifecycle::State &)
{
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  setpoint_topic_ = get_node()->get_parameter("setpoint_topic").as_string();
  body_force_controller_name_ = get_node()->get_parameter("body_force_controller_name").as_string();
  world_frame_id_ = get_node()->get_parameter("world_frame_id").as_string();
  base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
  prediction_horizon_ = std::max(
    1,
    static_cast<int>(get_node()->get_parameter("prediction_horizon").as_int()));
  prediction_dt_ = std::max(1e-4, get_node()->get_parameter("prediction_dt").as_double());
  mpc_rate_ = std::max(1.0, get_node()->get_parameter("mpc_rate").as_double());
  navigator_timeout_ = std::max(0.0, get_node()->get_parameter("navigator_timeout").as_double());
  setpoint_timeout_ = std::max(0.0, get_node()->get_parameter("setpoint_timeout").as_double());
  fallback_mode_ = get_node()->get_parameter("fallback_mode").as_string();
  mpc_control_axes_ = readControlAxesParameter();

  if (navigator_topic_.empty() || setpoint_topic_.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Mpc4dofController requires non-empty navigator_topic and setpoint_topic");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (body_force_controller_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "body_force_controller_name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (fallback_mode_ != "hold_previous" && fallback_mode_ != "zero" &&
    fallback_mode_ != "decay_to_zero")
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Unknown fallback_mode '%s'. Falling back to hold_previous.",
      fallback_mode_.c_str());
    fallback_mode_ = "hold_previous";
  }

  model_.inertia = readVector4Parameter(get_node(), "inertia", model_.inertia)
    .cwiseMax(Eigen::Vector4d::Constant(kMinInertia));
  model_.linear_damping = readVector4Parameter(get_node(), "linear_damping", model_.linear_damping);
  model_.quadratic_damping =
    readVector4Parameter(get_node(), "quadratic_damping", model_.quadratic_damping);
  model_.static_wrench = readVector4Parameter(get_node(), "static_wrench", model_.static_wrench);
  model_.restoring_z = get_node()->get_parameter("restoring_z").as_double();

  const StateVector q_state = readStateWeightsParameter(
    get_node(),
    "q_state",
    (StateVector() << 5.0, 5.0, 10.0, 4.0, 0.5, 0.5, 1.0, 0.4).finished());
  const InputVector r_input = readVector4Parameter(
    get_node(), "r_input", (InputVector() << 0.05, 0.05, 0.05, 0.03).finished());
  const InputVector s_delta_input = readVector4Parameter(
    get_node(), "s_delta_input", (InputVector() << 0.2, 0.2, 0.2, 0.1).finished());
  const InputVector wrench_min = readVector4Parameter(
    get_node(), "wrench_min", (InputVector() << -40.0, -40.0, -40.0, -8.0).finished());
  const InputVector wrench_max = readVector4Parameter(
    get_node(), "wrench_max", (InputVector() << 40.0, 40.0, 40.0, 8.0).finished());
  const InputVector wrench_rate_limit = readVector4Parameter(
    get_node(), "wrench_rate_limit", (InputVector() << 5.0, 5.0, 5.0, 1.0).finished());

  solver_.configure(
    prediction_horizon_,
    prediction_dt_,
    wrench_min,
    wrench_max,
    wrench_rate_limit,
    q_state,
    r_input,
    s_delta_input,
    get_node()->get_parameter("terminal_weight_scale").as_double(),
    static_cast<int>(get_node()->get_parameter("solver_max_iterations").as_int()),
    get_node()->get_parameter("solver_time_limit_ms").as_double());

  previous_solution_.assign(static_cast<size_t>(prediction_horizon_), InputVector::Zero());
  previous_tau_.setZero();

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
      last_navigator_time_ns_.store(get_node()->now().nanoseconds(), std::memory_order_relaxed);
    });

  setpoint_sub_ = get_node()->create_subscription<PoseStampedMsg>(
    setpoint_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const PoseStampedMsg::SharedPtr msg)
    {
      setpoint_buffer_.writeFromNonRT(msg);
      last_setpoint_time_ns_.store(get_node()->now().nanoseconds(), std::memory_order_relaxed);
    });

  RCLCPP_INFO(get_node()->get_logger(), "Configured Mpc4dofController");
  RCLCPP_INFO(get_node()->get_logger(), "Navigator topic: %s", navigator_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Setpoint topic: %s", setpoint_topic_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Writing chained wrench references to controller '%s'",
    body_force_controller_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Mpc4dofController::on_activate(
  const rclcpp_lifecycle::State &)
{
  navigator_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>>(nullptr);
  setpoint_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<PoseStampedMsg>>(nullptr);

  Command zero_command;
  command_buffer_.writeFromNonRT(zero_command);
  previous_tau_.setZero();
  previous_solution_.assign(static_cast<size_t>(prediction_horizon_), InputVector::Zero());
  last_navigator_time_ns_.store(0, std::memory_order_relaxed);
  last_setpoint_time_ns_.store(0, std::memory_order_relaxed);

  if (command_interfaces_.size() != 6U) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Mpc4dofController expected 6 command interfaces, got %zu",
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  solver_running_.store(true, std::memory_order_release);
  solver_thread_ = std::thread(&Mpc4dofController::solverLoop, this);

  RCLCPP_INFO(get_node()->get_logger(), "Activated Mpc4dofController");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Mpc4dofController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stopSolverThread();
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "Deactivated Mpc4dofController");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Mpc4dofController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  const Command * command = command_buffer_.readFromRT();
  if (command == nullptr) {
    return controller_interface::return_type::OK;
  }

  command_interfaces_[0].set_value(command->tau[0]);
  command_interfaces_[1].set_value(command->tau[1]);
  command_interfaces_[2].set_value(command->tau[2]);
  command_interfaces_[3].set_value(0.0);
  command_interfaces_[4].set_value(0.0);
  command_interfaces_[5].set_value(command->tau[3]);

  return controller_interface::return_type::OK;
}

double Mpc4dofController::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double Mpc4dofController::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

Mpc4dofController::StateVector Mpc4dofController::stateToVector(const MpcState & state)
{
  StateVector vector;
  vector << state.x, state.y, state.z, state.yaw, state.u, state.v, state.w, state.r;
  return vector;
}

Mpc4dofController::MpcState Mpc4dofController::vectorToState(const StateVector & vector)
{
  MpcState state;
  state.x = vector(0);
  state.y = vector(1);
  state.z = vector(2);
  state.yaw = vector(3);
  state.u = vector(4);
  state.v = vector(5);
  state.w = vector(6);
  state.r = vector(7);
  return state;
}

Mpc4dofController::StateVector Mpc4dofController::navigatorToState(
  const NavigatorMsg & navigator_msg)
{
  MpcState state;
  state.x = navigator_msg.position.position.x;
  state.y = navigator_msg.position.position.y;
  state.z = navigator_msg.position.position.z;
  state.yaw = navigator_msg.rpy.z;
  state.u = navigator_msg.body_velocity.linear.x;
  state.v = navigator_msg.body_velocity.linear.y;
  state.w = navigator_msg.body_velocity.linear.z;
  state.r = navigator_msg.body_velocity.angular.z;
  return stateToVector(state);
}

Mpc4dofController::StateVector Mpc4dofController::setpointToReference(
  const PoseStampedMsg & setpoint_msg)
{
  StateVector reference;
  reference.setZero();
  reference(0) = setpoint_msg.pose.position.x;
  reference(1) = setpoint_msg.pose.position.y;
  reference(2) = setpoint_msg.pose.position.z;
  reference(3) = quaternionToYaw(setpoint_msg.pose.orientation);
  return reference;
}

Mpc4dofController::Command Mpc4dofController::inputToCommand(const InputVector & tau)
{
  Command command;
  for (Eigen::Index i = 0; i < kInputSize; ++i) {
    command.tau[static_cast<size_t>(i)] = tau(i);
  }
  return command;
}

Mpc4dofController::InputVector Mpc4dofController::commandToInput(const Command & command)
{
  InputVector tau;
  for (Eigen::Index i = 0; i < kInputSize; ++i) {
    tau(i) = command.tau[static_cast<size_t>(i)];
  }
  return tau;
}

Eigen::Vector4d Mpc4dofController::readVector4Parameter(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const Eigen::Vector4d & fallback)
{
  const auto values = node->get_parameter(name).as_double_array();
  if (values.size() != kInputSize) {
    RCLCPP_WARN(
      node->get_logger(),
      "Parameter '%s' must have length 4. Using fallback.",
      name.c_str());
    return fallback;
  }

  Eigen::Vector4d vector;
  for (Eigen::Index i = 0; i < kInputSize; ++i) {
    vector(i) = values[static_cast<size_t>(i)];
  }
  return vector;
}

Mpc4dofController::StateVector Mpc4dofController::readStateWeightsParameter(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const StateVector & fallback)
{
  const auto values = node->get_parameter(name).as_double_array();
  if (values.size() != kStateSize) {
    RCLCPP_WARN(
      node->get_logger(),
      "Parameter '%s' must have length 8. Using fallback.",
      name.c_str());
    return fallback;
  }

  StateVector vector;
  for (Eigen::Index i = 0; i < kStateSize; ++i) {
    vector(i) = values[static_cast<size_t>(i)];
  }
  return vector;
}

Mpc4dofController::InputVector Mpc4dofController::applyControlAxes(
  const InputVector & tau,
  const std::array<bool, kInputSize> & axes)
{
  InputVector masked = tau;
  for (Eigen::Index i = 0; i < kInputSize; ++i) {
    if (!axes[static_cast<size_t>(i)]) {
      masked(i) = 0.0;
    }
  }
  return masked;
}

Mpc4dofController::StateVector Mpc4dofController::continuousDynamics(
  const StateVector & x,
  const InputVector & tau,
  const MpcModel & model)
{
  StateVector x_dot;
  x_dot.setZero();

  const double yaw = x(3);
  const double u = x(4);
  const double v = x(5);
  const double w = x(6);
  const double r = x(7);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  x_dot(0) = u * cos_yaw - v * sin_yaw;
  x_dot(1) = u * sin_yaw + v * cos_yaw;
  x_dot(2) = w;
  x_dot(3) = r;

  const Eigen::Vector4d velocity(u, v, w, r);
  Eigen::Vector4d inertia = model.inertia.cwiseMax(Eigen::Vector4d::Constant(kMinInertia));
  Eigen::Vector4d dynamics =
    tau -
    model.linear_damping.cwiseProduct(velocity) -
    model.quadratic_damping.cwiseProduct(velocity.cwiseAbs().cwiseProduct(velocity)) -
    model.static_wrench;
  dynamics(2) -= model.restoring_z;

  x_dot.template segment<4>(4) = dynamics.cwiseQuotient(inertia);
  return x_dot;
}

void Mpc4dofController::linearizeDiscreteModel(
  const StateVector & x_bar,
  const InputVector & tau_bar,
  const MpcModel & model,
  double dt,
  StateMatrix & A,
  InputMatrix & B,
  StateVector & c)
{
  StateMatrix A_c = StateMatrix::Zero();
  InputMatrix B_c = InputMatrix::Zero();

  const double yaw = x_bar(3);
  const double u = x_bar(4);
  const double v = x_bar(5);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  A_c(0, 3) = -u * sin_yaw - v * cos_yaw;
  A_c(0, 4) = cos_yaw;
  A_c(0, 5) = -sin_yaw;
  A_c(1, 3) = u * cos_yaw - v * sin_yaw;
  A_c(1, 4) = sin_yaw;
  A_c(1, 5) = cos_yaw;
  A_c(2, 6) = 1.0;
  A_c(3, 7) = 1.0;

  const Eigen::Vector4d velocity = x_bar.template segment<4>(4);
  const Eigen::Vector4d inertia =
    model.inertia.cwiseMax(Eigen::Vector4d::Constant(kMinInertia));
  for (Eigen::Index i = 0; i < kInputSize; ++i) {
    A_c(i + 4, i + 4) =
      -(model.linear_damping(i) + 2.0 * model.quadratic_damping(i) * std::abs(velocity(i))) /
      inertia(i);
    B_c(i + 4, i) = 1.0 / inertia(i);
  }

  A = StateMatrix::Identity() + dt * A_c;
  B = dt * B_c;
  c = dt * (continuousDynamics(x_bar, tau_bar, model) - A_c * x_bar - B_c * tau_bar);
}

std::array<bool, Mpc4dofController::kInputSize> Mpc4dofController::readControlAxesParameter() const
{
  const auto values = get_node()->get_parameter("mpc_control_axes").as_bool_array();
  if (values.size() != kInputSize) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "mpc_control_axes must have length 4 in order [x, y, z, yaw]. Using all axes enabled.");
    return {true, true, true, true};
  }

  return {values[0], values[1], values[2], values[3]};
}

std::vector<Mpc4dofController::InputVector> Mpc4dofController::shiftedPreviousSolution() const
{
  std::vector<InputVector> shifted(static_cast<size_t>(prediction_horizon_), InputVector::Zero());
  if (previous_solution_.empty()) {
    return shifted;
  }

  for (int i = 0; i < prediction_horizon_ - 1; ++i) {
    shifted[static_cast<size_t>(i)] = previous_solution_[static_cast<size_t>(i + 1)];
  }
  shifted.back() = previous_solution_.back();
  return shifted;
}

void Mpc4dofController::solverLoop()
{
  const auto period = std::chrono::duration<double>(1.0 / mpc_rate_);

  while (solver_running_.load(std::memory_order_acquire)) {
    const auto loop_start = std::chrono::steady_clock::now();
    InputVector tau = previous_tau_;

    const auto now_ns = get_node()->now().nanoseconds();
    const int64_t navigator_age_ns =
      now_ns - last_navigator_time_ns_.load(std::memory_order_relaxed);
    const int64_t setpoint_age_ns =
      now_ns - last_setpoint_time_ns_.load(std::memory_order_relaxed);

    auto navigator_msg = navigator_buffer_.readFromNonRT();
    auto setpoint_msg = setpoint_buffer_.readFromNonRT();
    const bool navigator_fresh =
      navigator_msg && *navigator_msg &&
      navigator_age_ns >= 0 &&
      static_cast<double>(navigator_age_ns) * 1e-9 <= navigator_timeout_;
    const bool setpoint_fresh =
      setpoint_msg && *setpoint_msg &&
      setpoint_age_ns >= 0 &&
      static_cast<double>(setpoint_age_ns) * 1e-9 <= setpoint_timeout_;

    if (navigator_fresh && setpoint_fresh) {
      const StateVector x0 = navigatorToState(*(*navigator_msg));
      StateVector x_ref = setpointToReference(*(*setpoint_msg));
      x_ref(3) = x0(3) + wrapAngle(x_ref(3) - x0(3));

      std::vector<StateMatrix> A_seq;
      std::vector<InputMatrix> B_seq;
      std::vector<StateVector> c_seq;
      std::vector<StateVector> x_ref_seq;

      const bool problem_ready = buildLinearizedProblem(x0, x_ref, A_seq, B_seq, c_seq, x_ref_seq);
      const bool solved = problem_ready &&
        solver_.solve(x0, A_seq, B_seq, c_seq, x_ref_seq, model_, previous_tau_, tau);

      if (!solved) {
        tau = fallbackCommand(previous_tau_);
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(),
          2000,
          "MPC solve failed. Applying fallback mode '%s'.",
          fallback_mode_.c_str());
      }
    } else {
      tau = fallbackCommand(previous_tau_);
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "MPC waiting for fresh inputs. navigator_fresh=%s setpoint_fresh=%s",
        navigator_fresh ? "true" : "false",
        setpoint_fresh ? "true" : "false");
    }

    tau = applyControlAxes(tau, mpc_control_axes_);
    clampInPlace(tau, solver_.wrench_min, solver_.wrench_max);
    writeCommand(tau);
    previous_tau_ = tau;

    if (!previous_solution_.empty()) {
      for (int i = 0; i < prediction_horizon_ - 1; ++i) {
        previous_solution_[static_cast<size_t>(i)] =
          previous_solution_[static_cast<size_t>(i + 1)];
      }
      previous_solution_.back() = tau;
    }

    const auto elapsed = std::chrono::steady_clock::now() - loop_start;
    if (elapsed < period) {
      std::this_thread::sleep_for(period - elapsed);
    }
  }
}

bool Mpc4dofController::buildLinearizedProblem(
  const StateVector & x0,
  const StateVector & x_ref,
  std::vector<StateMatrix> & A_seq,
  std::vector<InputMatrix> & B_seq,
  std::vector<StateVector> & c_seq,
  std::vector<StateVector> & x_ref_seq)
{
  const std::vector<InputVector> tau_bar = shiftedPreviousSolution();
  A_seq.assign(static_cast<size_t>(prediction_horizon_), StateMatrix::Identity());
  B_seq.assign(static_cast<size_t>(prediction_horizon_), InputMatrix::Zero());
  c_seq.assign(static_cast<size_t>(prediction_horizon_), StateVector::Zero());
  x_ref_seq.assign(static_cast<size_t>(prediction_horizon_ + 1), x_ref);

  StateVector x_bar = x0;
  for (int i = 0; i < prediction_horizon_; ++i) {
    const auto index = static_cast<size_t>(i);
    linearizeDiscreteModel(
      x_bar,
      tau_bar[index],
      model_,
      prediction_dt_,
      A_seq[index],
      B_seq[index],
      c_seq[index]);

    x_bar += prediction_dt_ * continuousDynamics(x_bar, tau_bar[index], model_);
    x_bar(3) = wrapAngle(x_bar(3));
  }

  return true;
}

Mpc4dofController::InputVector Mpc4dofController::fallbackCommand(
  const InputVector & previous_tau) const
{
  if (fallback_mode_ == "zero") {
    return InputVector::Zero();
  }
  if (fallback_mode_ == "decay_to_zero") {
    return 0.95 * previous_tau;
  }
  return previous_tau;
}

void Mpc4dofController::writeCommand(const InputVector & tau)
{
  command_buffer_.writeFromNonRT(inputToCommand(tau));
}

void Mpc4dofController::stopSolverThread()
{
  solver_running_.store(false, std::memory_order_release);
  if (solver_thread_.joinable()) {
    solver_thread_.join();
  }
}

}  // namespace sura_controllers::auv

PLUGINLIB_EXPORT_CLASS(
  sura_controllers::auv::Mpc4dofController,
  controller_interface::ControllerInterface)
