#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "sura_controllers/auv/state_observer_model.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_msgs/msg/auv_dynamic_model.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace sura_controllers::auv
{

class StateObserverController : public controller_interface::ControllerInterface
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
  using WrenchMsg = geometry_msgs::msg::Wrench;
  using PoseWithCovarianceStampedMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
  using OdometryMsg = nav_msgs::msg::Odometry;
  using ImuMsg = sensor_msgs::msg::Imu;
  using ModelMsg = sura_msgs::msg::AuvDynamicModel;
  using Trigger = std_srvs::srv::Trigger;
  using Vector6 = StateObserverModel::Vector6;
  using AxisFeedbackMask = std::array<bool, 6>;

  enum class ObserverMode : int
  {
    LEARN = 0,
    PREDICT = 1
  };

  struct ObserverState
  {
    Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation_body_to_ned{Eigen::Quaterniond::Identity()};
    Vector6 body_velocity{Vector6::Zero()};
    Vector6 pose_covariance{Vector6::Zero()};
    bool initialized{false};
  };

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  static Eigen::Quaterniond normalizeQuaternion(const Eigen::Quaterniond & quaternion);
  static Eigen::Vector3d quaternionToRpy(const Eigen::Quaterniond & quaternion);
  static Eigen::Quaterniond rpyToQuaternion(const Eigen::Vector3d & rpy);
  static Eigen::Vector3d enuPositionToNed(const geometry_msgs::msg::Point & position);
  static Vector6 wrenchToVector(const WrenchMsg & wrench_msg);
  static Vector6 navigatorBodyVelocityToVector(const NavigatorMsg & navigator_msg);
  static Vector6 navigatorBodyAccelerationToVector(const NavigatorMsg & navigator_msg);
  static Vector6 imuBodyAccelerationToVector(const ImuMsg & imu_msg);
  static void copyVectorToArray(
    const Vector6 & vector,
    std::array<double, 6> & array);

  Vector6 readVector6Parameter(const std::string & name, const Vector6 & fallback) const;
  AxisFeedbackMask readAxisFeedbackMaskParameter(
    const std::string & name,
    const AxisFeedbackMask & fallback) const;
  bool setModeFromString(const std::string & mode);
  std::string modeString() const;

  void resetState();
  void correctStateFromNavigator(const NavigatorMsg & navigator_msg);
  void correctPredictFeedbackFromNavigator(const NavigatorMsg & navigator_msg);
  void correctPredictFeedbackFromImu(const ImuMsg & imu_msg);
  void correctPositionFromAruco(const PoseWithCovarianceStampedMsg & aruco_msg);
  void correctDepthFromPose(const PoseWithCovarianceStampedMsg & depth_msg);
  void predictState(
    const WrenchMsg * wrench_msg,
    const ImuMsg * imu_msg,
    const PoseWithCovarianceStampedMsg * aruco_msg,
    const PoseWithCovarianceStampedMsg * depth_msg,
    double dt);
  Vector6 restoringWrenchBody() const;
  void publishState(const rclcpp::Time & stamp);
  void publishModel(const rclcpp::Time & stamp);
  void publishOutputs(const rclcpp::Time & stamp);

  bool saveModel(std::string & message) const;
  bool loadModel(std::string & message);

  bool handleFreeze(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  bool handleReset(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  bool handleSave(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  bool handleLoad(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  bool handleSetLearnMode(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  bool handleSetPredictMode(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Subscription<WrenchMsg>::SharedPtr force_sub_;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<PoseWithCovarianceStampedMsg>::SharedPtr aruco_pose_sub_;
  rclcpp::Subscription<PoseWithCovarianceStampedMsg>::SharedPtr depth_pose_sub_;
  rclcpp::Publisher<PoseWithCovarianceStampedMsg>::SharedPtr predicted_pose_pub_;
  rclcpp::Publisher<OdometryMsg>::SharedPtr predicted_odometry_pub_;
  rclcpp::Publisher<ModelMsg>::SharedPtr model_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<PoseWithCovarianceStampedMsg>>
  predicted_pose_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<OdometryMsg>> predicted_odometry_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<ModelMsg>> model_rt_pub_;

  rclcpp::Service<Trigger>::SharedPtr freeze_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
  rclcpp::Service<Trigger>::SharedPtr load_service_;
  rclcpp::Service<Trigger>::SharedPtr learn_mode_service_;
  rclcpp::Service<Trigger>::SharedPtr predict_mode_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<WrenchMsg>> force_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ImuMsg>> imu_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<PoseWithCovarianceStampedMsg>> aruco_pose_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<PoseWithCovarianceStampedMsg>> depth_pose_buffer_;

  mutable std::mutex model_mutex_;
  StateObserverModel model_;
  ObserverState state_;

  std::string navigator_topic_;
  std::string force_topic_;
  std::string imu_topic_;
  std::string aruco_pose_topic_;
  std::string depth_pose_topic_;
  std::string predicted_pose_topic_;
  std::string predicted_odometry_topic_;
  std::string model_topic_;
  std::string freeze_service_name_;
  std::string reset_service_name_;
  std::string save_service_name_;
  std::string load_service_name_;
  std::string learn_service_name_;
  std::string predict_service_name_;
  std::string world_frame_id_;
  std::string base_frame_id_;
  std::string model_file_;

  Vector6 navigation_pose_covariance_{Vector6::Zero()};
  Vector6 process_noise_{Vector6::Zero()};
  AxisFeedbackMask navigation_feedback_axes_{false, false, true, true, true, true};

  double navigator_timeout_{0.5};
  double force_timeout_{0.5};
  double imu_timeout_{0.5};
  double aruco_pose_timeout_{0.5};
  double depth_pose_timeout_{0.5};
  double forgetting_factor_{0.995};
  double min_excitation_{1e-3};
  double initial_estimator_covariance_{1e3};
  double model_publish_period_{1.0};
  double max_prediction_dt_{0.1};
  double weight_{112.8};
  double buoyancy_{114.8};
  bool use_imu_linear_acceleration_{false};

  std::atomic<bool> controller_active_{false};
  std::atomic<int> mode_{static_cast<int>(ObserverMode::LEARN)};
  std::atomic<bool> parameters_frozen_{false};
  std::atomic<int64_t> last_navigator_time_ns_{0};
  std::atomic<int64_t> last_force_time_ns_{0};
  std::atomic<int64_t> last_imu_time_ns_{0};
  std::atomic<int64_t> last_aruco_pose_time_ns_{0};
  std::atomic<int64_t> last_depth_pose_time_ns_{0};
  std::atomic<int64_t> last_state_time_ns_{0};
  std::atomic<int64_t> last_model_publish_time_ns_{0};
};

}  // namespace sura_controllers::auv
