#ifndef SURA_CONTROLLERS__CONTROLLER_ARBITRATOR_HPP_
#define SURA_CONTROLLERS__CONTROLLER_ARBITRATOR_HPP_

#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sura_msgs/msg/sura_pose_command.hpp"
#include "sura_msgs/msg/sura_velocity_command.hpp"
#include "sura_msgs/msg/sura_wrench_command.hpp"
#include "sura_msgs/srv/clear_controller_intents.hpp"

namespace sura_controllers
{

class ControllerArbitrator : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ControllerArbitrator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  enum class IntentType
  {
    Velocity,
    Pose,
    Wrench
  };

  using VelocityMsg = geometry_msgs::msg::Twist;
  using PoseMsg = geometry_msgs::msg::PoseStamped;
  using WrenchMsg = geometry_msgs::msg::Wrench;
  using IntentPayload = std::variant<VelocityMsg, PoseMsg, WrenchMsg>;

  struct ControllerRoute
  {
    IntentType type;
    std::string topic;
  };

  struct Intent
  {
    std::string requester;
    std::string controller;
    IntentType type;
    uint8_t priority;
    rclcpp::Time stamp;
    uint64_t sequence;
    IntentPayload payload;
  };

  std::string namespacedTopic(const std::string & suffix) const;
  std::string makeIntentKey(const std::string & requester, const std::string & controller) const;
  std::string makeWinnerKey(const Intent & intent) const;
  static std::string typeName(IntentType type);
  static std::optional<IntentType> parseType(const std::string & type);

  bool isActive();
  bool validateCommon(
    const std::string & requester,
    const std::string & controller,
    uint8_t priority,
    IntentType type);
  bool isFinite(const VelocityMsg & msg) const;
  bool isFinite(const PoseMsg & msg) const;
  bool isFinite(const WrenchMsg & msg) const;
  bool isFinite(double value) const;

  void handleVelocityCommand(const sura_msgs::msg::SuraVelocityCommand::SharedPtr msg);
  void handlePoseCommand(const sura_msgs::msg::SuraPoseCommand::SharedPtr msg);
  void handleWrenchCommand(const sura_msgs::msg::SuraWrenchCommand::SharedPtr msg);
  void storeIntent(Intent intent);

  void handleClearControllerIntents(
    const std::shared_ptr<sura_msgs::srv::ClearControllerIntents::Request> request,
    std::shared_ptr<sura_msgs::srv::ClearControllerIntents::Response> response);

  void arbitrationTick();
  void removeExpiredIntents(const rclcpp::Time & now);
  std::optional<Intent> selectWinnerLocked() const;
  bool hasPositionHoldVelocityModeLocked() const;
  void publishWinner(const Intent & intent, bool position_hold_velocity_mode);
  void publishZeroIfNeeded(const std::optional<Intent> & previous_winner);

  void cleanupResources();

  std::string robot_namespace_;
  std::string body_velocity_controller_name_{"body_velocity"};
  std::string position_hold_controller_name_{"position_hold"};
  std::string position_hold_temporary_controller_name_{"position_hold_temporary"};
  std::string position_hold_reposition_controller_name_{"position_hold_reposition"};
  std::string position_hold_feedforward_topic_;
  std::string position_hold_reposition_feedforward_topic_;
  double arbitration_frequency_hz_{10.0};
  double command_timeout_s_{0.15};
  uint64_t sequence_counter_{0};

  std::unordered_map<std::string, ControllerRoute> routes_;
  std::unordered_map<std::string, Intent> intents_;
  std::optional<Intent> last_published_winner_;
  bool zero_published_after_idle_{true};
  mutable std::mutex mutex_;

  rclcpp::Subscription<sura_msgs::msg::SuraVelocityCommand>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sura_msgs::msg::SuraPoseCommand>::SharedPtr pose_sub_;
  rclcpp::Subscription<sura_msgs::msg::SuraWrenchCommand>::SharedPtr wrench_sub_;
  rclcpp::Service<sura_msgs::srv::ClearControllerIntents>::SharedPtr clear_service_;
  rclcpp::TimerBase::SharedPtr arbitration_timer_;

  std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<VelocityMsg>::SharedPtr>
  velocity_publishers_;
  rclcpp_lifecycle::LifecyclePublisher<VelocityMsg>::SharedPtr position_hold_feedforward_pub_;
  rclcpp_lifecycle::LifecyclePublisher<VelocityMsg>::SharedPtr
    position_hold_reposition_feedforward_pub_;
  std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<PoseMsg>::SharedPtr>
  pose_publishers_;
  std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<WrenchMsg>::SharedPtr>
  wrench_publishers_;
};

}  // namespace sura_controllers

#endif  // SURA_CONTROLLERS__CONTROLLER_ARBITRATOR_HPP_