#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace sura_controllers::auv
{

class Mpc4dofController : public controller_interface::ControllerInterface
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
  static constexpr int kStateSize = 8;
  static constexpr int kInputSize = 4;

  using StateVector = Eigen::Matrix<double, kStateSize, 1>;
  using InputVector = Eigen::Matrix<double, kInputSize, 1>;
  using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;
  using InputMatrix = Eigen::Matrix<double, kStateSize, kInputSize>;
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

  struct MpcState
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    double u{0.0};
    double v{0.0};
    double w{0.0};
    double r{0.0};
  };

  struct MpcInput
  {
    double X{0.0};
    double Y{0.0};
    double Z{0.0};
    double N{0.0};
  };

  struct MpcModel
  {
    Eigen::Vector4d inertia{Eigen::Vector4d::Ones()};
    Eigen::Vector4d linear_damping{Eigen::Vector4d::Zero()};
    Eigen::Vector4d quadratic_damping{Eigen::Vector4d::Zero()};
    Eigen::Vector4d static_wrench{Eigen::Vector4d::Zero()};
    double restoring_z{0.0};
  };

  struct Command
  {
    std::array<double, kInputSize> tau{{0.0, 0.0, 0.0, 0.0}};
  };

  struct MpcQpSolver
  {
    bool configure(
      int horizon,
      double dt,
      const Eigen::Vector4d & wrench_min,
      const Eigen::Vector4d & wrench_max,
      const Eigen::Vector4d & wrench_rate_limit,
      const Eigen::Matrix<double, kStateSize, 1> & q_state,
      const Eigen::Matrix<double, kInputSize, 1> & r_input,
      const Eigen::Matrix<double, kInputSize, 1> & s_delta_input,
      double terminal_weight_scale,
      int max_iterations,
      double time_limit_ms);

    bool solve(
      const StateVector & x0,
      const std::vector<StateMatrix> & A_seq,
      const std::vector<InputMatrix> & B_seq,
      const std::vector<StateVector> & c_seq,
      const std::vector<StateVector> & x_ref_seq,
      const MpcModel & model,
      const InputVector & previous_tau,
      InputVector & tau0_solution);

    int horizon{10};
    double dt{0.05};
    Eigen::Vector4d wrench_min{Eigen::Vector4d::Constant(-40.0)};
    Eigen::Vector4d wrench_max{Eigen::Vector4d::Constant(40.0)};
    Eigen::Vector4d wrench_rate_limit{Eigen::Vector4d::Constant(5.0)};
    StateVector q_state{StateVector::Ones()};
    InputVector r_input{InputVector::Ones()};
    InputVector s_delta_input{InputVector::Ones()};
    double terminal_weight_scale{5.0};
    int max_iterations{50};
    double time_limit_ms{8.0};
    Eigen::VectorXd warm_start;
  };

  static double wrapAngle(double angle);
  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q);
  static StateVector stateToVector(const MpcState & state);
  static MpcState vectorToState(const StateVector & vector);
  static StateVector navigatorToState(const NavigatorMsg & navigator_msg);
  static StateVector setpointToReference(const PoseStampedMsg & setpoint_msg);
  static Command inputToCommand(const InputVector & tau);
  static InputVector commandToInput(const Command & command);
  static Eigen::Vector4d readVector4Parameter(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name,
    const Eigen::Vector4d & fallback);
  static StateVector readStateWeightsParameter(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name,
    const StateVector & fallback);
  static InputVector applyControlAxes(
    const InputVector & tau,
    const std::array<bool, kInputSize> & axes);

  static StateVector continuousDynamics(
    const StateVector & x,
    const InputVector & tau,
    const MpcModel & model);

  static void linearizeDiscreteModel(
    const StateVector & x_bar,
    const InputVector & tau_bar,
    const MpcModel & model,
    double dt,
    StateMatrix & A,
    InputMatrix & B,
    StateVector & c);

  std::array<bool, kInputSize> readControlAxesParameter() const;
  std::vector<InputVector> shiftedPreviousSolution() const;
  void solverLoop();
  bool buildLinearizedProblem(
    const StateVector & x0,
    const StateVector & x_ref,
    std::vector<StateMatrix> & A_seq,
    std::vector<InputMatrix> & B_seq,
    std::vector<StateVector> & c_seq,
    std::vector<StateVector> & x_ref_seq);
  InputVector fallbackCommand(const InputVector & previous_tau) const;
  void writeCommand(const InputVector & tau);
  void stopSolverThread();

  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Subscription<PoseStampedMsg>::SharedPtr setpoint_sub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<PoseStampedMsg>> setpoint_buffer_;
  realtime_tools::RealtimeBuffer<Command> command_buffer_;

  MpcQpSolver solver_;
  MpcModel model_;
  std::vector<InputVector> previous_solution_;
  InputVector previous_tau_{InputVector::Zero()};

  std::string navigator_topic_;
  std::string setpoint_topic_;
  std::string body_force_controller_name_;
  std::string world_frame_id_;
  std::string base_frame_id_;
  std::string fallback_mode_;
  std::array<bool, kInputSize> mpc_control_axes_{{true, true, true, true}};

  int prediction_horizon_{10};
  double prediction_dt_{0.05};
  double mpc_rate_{20.0};
  double navigator_timeout_{0.5};
  double setpoint_timeout_{1.0};

  std::thread solver_thread_;
  std::atomic<bool> solver_running_{false};
  std::atomic<int64_t> last_navigator_time_ns_{0};
  std::atomic<int64_t> last_setpoint_time_ns_{0};
};

}  // namespace sura_controllers::auv
