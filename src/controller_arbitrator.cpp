#include "sura_controllers/controller_arbitrator.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace sura_controllers
{
namespace
{
constexpr uint8_t kMinPriority = 1;
constexpr uint8_t kMaxPriority = 100;

std::string stripSlashes(std::string value)
{
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}
}  // namespace

ControllerArbitrator::ControllerArbitrator(const rclcpp::NodeOptions & options)
: LifecycleNode("controller_arbitrator", options)
{
}

ControllerArbitrator::CallbackReturn ControllerArbitrator::on_configure(
  const rclcpp_lifecycle::State &)
{
  robot_namespace_ = stripSlashes(declare_parameter<std::string>("robot_namespace", "sura"));
  arbitration_frequency_hz_ = declare_parameter<double>("arbitration_frequency_hz", 10.0);
  command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.15);
  body_velocity_controller_name_ =
    declare_parameter<std::string>("body_velocity_controller_name", "body_velocity");
  position_hold_controller_name_ =
    declare_parameter<std::string>("position_hold_controller_name", "position_hold");
  position_hold_temporary_controller_name_ = declare_parameter<std::string>(
    "position_hold_temporary_controller_name", "position_hold_temporary");
  position_hold_reposition_controller_name_ = declare_parameter<std::string>(
    "position_hold_reposition_controller_name", "position_hold_reposition");
  controller_manager_switch_service_ = declare_parameter<std::string>(
    "controller_manager_switch_service",
    namespacedTopic("controller/controller_manager/switch_controller"));
  const auto position_hold_feedforward_topic_suffix = declare_parameter<std::string>(
    "position_hold_feedforward_topic_suffix", "controller/position_hold/feedforward");
  const auto position_hold_reposition_feedforward_topic_suffix = declare_parameter<std::string>(
    "position_hold_reposition_feedforward_topic_suffix",
    "controller/position_hold/reposition_feedforward");

  if (arbitration_frequency_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "Invalid arbitration_frequency_hz, using 10.0 Hz");
    arbitration_frequency_hz_ = 10.0;
  }
  if (command_timeout_s_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "Invalid command_timeout_s, using 0.15 s");
    command_timeout_s_ = 0.15;
  }

  std::vector<std::string> controllers = declare_parameter<std::vector<std::string>>(
    "controllers",
    {"body_velocity", "position_hold", "mpc_4dof", "body_force", "stabilize", "depth_hold"});

  for (const auto & controller : controllers) {
    const std::string prefix = "controller_routes." + controller + ".";
    const std::string default_type =
      controller == "body_velocity" ? "velocity" :
      (controller == "position_hold" || controller == "mpc_4dof" ? "pose" : "wrench");
    const std::string default_topic =
      controller == "body_velocity" ? "controller/body_velocity/setpoint" :
      controller == "position_hold" ? "controller/position_hold/setpoint" :
      controller == "mpc_4dof" ? "controller/mpc_4dof/setpoint" :
      controller == "body_force" ? "controller/body_force/command" :
      controller == "stabilize" ? "controller/stabilize/feedforward" :
      controller == "depth_hold" ? "controller/depth_hold/feedforward" :
      "controller/" + controller + "/command";

    const auto type = parseType(declare_parameter<std::string>(prefix + "type", default_type));
    const auto topic_suffix = declare_parameter<std::string>(prefix + "topic_suffix", default_topic);

    if (!type) {
      RCLCPP_ERROR(get_logger(), "Invalid type for controller '%s'", controller.c_str());
      return CallbackReturn::ERROR;
    }
    if (topic_suffix.empty()) {
      RCLCPP_ERROR(get_logger(), "Empty topic_suffix for controller '%s'", controller.c_str());
      return CallbackReturn::ERROR;
    }

    const auto topic = namespacedTopic(topic_suffix);
    routes_[controller] = ControllerRoute{*type, topic};
    if (*type == IntentType::Velocity) {
      velocity_publishers_[controller] = create_publisher<VelocityMsg>(topic, rclcpp::SystemDefaultsQoS());
    } else if (*type == IntentType::Pose) {
      pose_publishers_[controller] = create_publisher<PoseMsg>(topic, rclcpp::SystemDefaultsQoS());
    } else {
      wrench_publishers_[controller] = create_publisher<WrenchMsg>(topic, rclcpp::SystemDefaultsQoS());
    }
    RCLCPP_INFO(
      get_logger(),
      "Controller route: %s type=%s topic=%s",
      controller.c_str(),
      typeName(*type).c_str(),
      topic.c_str());
  }

  position_hold_feedforward_topic_ = namespacedTopic(position_hold_feedforward_topic_suffix);
  position_hold_reposition_feedforward_topic_ =
    namespacedTopic(position_hold_reposition_feedforward_topic_suffix);

  position_hold_feedforward_pub_ = create_publisher<VelocityMsg>(
    position_hold_feedforward_topic_,
    rclcpp::SystemDefaultsQoS());
  position_hold_reposition_feedforward_pub_ = create_publisher<VelocityMsg>(
    position_hold_reposition_feedforward_topic_,
    rclcpp::SystemDefaultsQoS());

  routes_[position_hold_temporary_controller_name_] =
    ControllerRoute{IntentType::Velocity, position_hold_feedforward_topic_};
  routes_[position_hold_reposition_controller_name_] =
    ControllerRoute{IntentType::Velocity, position_hold_reposition_feedforward_topic_};

  RCLCPP_INFO(
    get_logger(),
    "Position hold temporary velocity controller '%s' publishes to '%s'",
    position_hold_temporary_controller_name_.c_str(),
    position_hold_feedforward_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Position hold reposition velocity controller '%s' publishes to '%s'",
    position_hold_reposition_controller_name_.c_str(),
    position_hold_reposition_feedforward_topic_.c_str());

  velocity_sub_ = create_subscription<sura_msgs::msg::SuraVelocityCommand>(
    namespacedTopic("controller/arbitrator/velocity"),
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ControllerArbitrator::handleVelocityCommand, this, std::placeholders::_1));
  pose_sub_ = create_subscription<sura_msgs::msg::SuraPoseCommand>(
    namespacedTopic("controller/arbitrator/pose"),
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ControllerArbitrator::handlePoseCommand, this, std::placeholders::_1));
  wrench_sub_ = create_subscription<sura_msgs::msg::SuraWrenchCommand>(
    namespacedTopic("controller/arbitrator/wrench"),
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ControllerArbitrator::handleWrenchCommand, this, std::placeholders::_1));
  clear_service_ = create_service<sura_msgs::srv::ClearControllerIntents>(
    namespacedTopic("controller/arbitrator/clear_controller_intents"),
    std::bind(
      &ControllerArbitrator::handleClearControllerIntents,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  interlock_service_ = create_service<sura_msgs::srv::ControllerInterlock>(
    namespacedTopic("controller/arbitrator/controller_interlock"),
    std::bind(
      &ControllerArbitrator::handleControllerInterlock,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  switch_service_ = create_service<controller_manager_msgs::srv::SwitchController>(
    namespacedTopic("controller/arbitrator/switch_controller"),
    std::bind(
      &ControllerArbitrator::handleSwitchController,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  switch_proxy_node_ =
    std::make_shared<rclcpp::Node>("controller_arbitrator_switch_proxy_client");
  controller_manager_switch_client_ =
    switch_proxy_node_->create_client<controller_manager_msgs::srv::SwitchController>(
      controller_manager_switch_service_);

  const auto period = std::chrono::duration<double>(1.0 / arbitration_frequency_hz_);
  arbitration_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ControllerArbitrator::arbitrationTick, this));
  arbitration_timer_->cancel();

  RCLCPP_INFO(
    get_logger(),
    "Controller arbitrator configured for robot namespace '%s'",
    robot_namespace_.c_str());
  return CallbackReturn::SUCCESS;
}

ControllerArbitrator::CallbackReturn ControllerArbitrator::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (auto & publisher : velocity_publishers_) {
    publisher.second->on_activate();
  }
  if (position_hold_feedforward_pub_) {
    position_hold_feedforward_pub_->on_activate();
  }
  if (position_hold_reposition_feedforward_pub_) {
    position_hold_reposition_feedforward_pub_->on_activate();
  }
  for (auto & publisher : pose_publishers_) {
    publisher.second->on_activate();
  }
  for (auto & publisher : wrench_publishers_) {
    publisher.second->on_activate();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    intents_.clear();
    last_published_winner_.reset();
    zero_published_after_idle_ = true;
  }
  if (arbitration_timer_) {
    arbitration_timer_->reset();
  }
  RCLCPP_INFO(get_logger(), "Controller arbitrator activated");
  return CallbackReturn::SUCCESS;
}

ControllerArbitrator::CallbackReturn ControllerArbitrator::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (arbitration_timer_) {
    arbitration_timer_->cancel();
  }
  std::optional<Intent> previous_winner;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous_winner = last_published_winner_;
    intents_.clear();
    last_published_winner_.reset();
    zero_published_after_idle_ = true;
  }
  publishZeroIfNeeded(previous_winner);
  for (auto & publisher : velocity_publishers_) {
    publisher.second->on_deactivate();
  }
  if (position_hold_feedforward_pub_) {
    position_hold_feedforward_pub_->on_deactivate();
  }
  if (position_hold_reposition_feedforward_pub_) {
    position_hold_reposition_feedforward_pub_->on_deactivate();
  }
  for (auto & publisher : pose_publishers_) {
    publisher.second->on_deactivate();
  }
  for (auto & publisher : wrench_publishers_) {
    publisher.second->on_deactivate();
  }
  RCLCPP_INFO(get_logger(), "Controller arbitrator deactivated");
  return CallbackReturn::SUCCESS;
}

ControllerArbitrator::CallbackReturn ControllerArbitrator::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

ControllerArbitrator::CallbackReturn ControllerArbitrator::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

std::string ControllerArbitrator::namespacedTopic(const std::string & suffix) const
{
  if (!suffix.empty() && suffix.front() == '/') {
    return suffix;
  }
  if (robot_namespace_.empty()) {
    return "/" + stripSlashes(suffix);
  }
  return "/" + robot_namespace_ + "/" + stripSlashes(suffix);
}

std::string ControllerArbitrator::makeIntentKey(
  const std::string & requester,
  const std::string & controller) const
{
  return requester + "\x1f" + controller;
}

std::string ControllerArbitrator::makeWinnerKey(const Intent & intent) const
{
  return makeIntentKey(intent.requester, intent.controller);
}

std::string ControllerArbitrator::typeName(IntentType type)
{
  switch (type) {
    case IntentType::Velocity:
      return "velocity";
    case IntentType::Pose:
      return "pose";
    case IntentType::Wrench:
      return "wrench";
  }
  return "unknown";
}

std::optional<ControllerArbitrator::IntentType> ControllerArbitrator::parseType(
  const std::string & type)
{
  if (type == "velocity") {
    return IntentType::Velocity;
  }
  if (type == "pose") {
    return IntentType::Pose;
  }
  if (type == "wrench") {
    return IntentType::Wrench;
  }
  return std::nullopt;
}

bool ControllerArbitrator::isActive()
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool ControllerArbitrator::validateCommon(
  const std::string & requester,
  const std::string & controller,
  uint8_t priority,
  IntentType type)
{
  if (!isActive()) {
    return false;
  }
  if (requester.empty() || controller.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Rejecting intent with empty requester or controller");
    return false;
  }
  if (priority < kMinPriority || priority > kMaxPriority) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Rejecting intent with priority outside [1, 100]");
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (isControllerBlockedLocked(controller)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Rejecting intent for blocked controller '%s': controller interlock active",
        controller.c_str());
      return false;
    }
  }
  const auto route = routes_.find(controller);
  if (route == routes_.end()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Rejecting intent for unknown controller '%s'",
      controller.c_str());
    return false;
  }
  if (route->second.type != type) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Rejecting %s intent for controller '%s' configured as %s",
      typeName(type).c_str(),
      controller.c_str(),
      typeName(route->second.type).c_str());
    return false;
  }
  return true;
}

bool ControllerArbitrator::isControllerBlockedLocked(const std::string & controller) const
{
  for (const auto & interlock : controller_interlocks_) {
    if (interlock.second.find(controller) != interlock.second.end()) {
      return true;
    }
  }
  return false;
}

bool ControllerArbitrator::isFinite(double value) const
{
  return std::isfinite(value);
}

bool ControllerArbitrator::isFinite(const VelocityMsg & msg) const
{
  return isFinite(msg.linear.x) && isFinite(msg.linear.y) && isFinite(msg.linear.z) &&
         isFinite(msg.angular.x) && isFinite(msg.angular.y) && isFinite(msg.angular.z);
}

bool ControllerArbitrator::isFinite(const PoseMsg & msg) const
{
  const auto & position = msg.pose.position;
  const auto & orientation = msg.pose.orientation;
  return isFinite(position.x) && isFinite(position.y) && isFinite(position.z) &&
         isFinite(orientation.x) && isFinite(orientation.y) && isFinite(orientation.z) &&
         isFinite(orientation.w);
}

bool ControllerArbitrator::isFinite(const WrenchMsg & msg) const
{
  return isFinite(msg.force.x) && isFinite(msg.force.y) && isFinite(msg.force.z) &&
         isFinite(msg.torque.x) && isFinite(msg.torque.y) && isFinite(msg.torque.z);
}

void ControllerArbitrator::handleVelocityCommand(
  const sura_msgs::msg::SuraVelocityCommand::SharedPtr msg)
{
  if (!validateCommon(msg->requester, msg->controller, msg->priority, IntentType::Velocity)) {
    return;
  }
  if (!isFinite(msg->velocity)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejecting non-finite velocity intent");
    return;
  }
  storeIntent(Intent{
    msg->requester,
    msg->controller,
    IntentType::Velocity,
    msg->priority,
    now(),
    ++sequence_counter_,
    msg->velocity});
}

void ControllerArbitrator::handlePoseCommand(const sura_msgs::msg::SuraPoseCommand::SharedPtr msg)
{
  if (!validateCommon(msg->requester, msg->controller, msg->priority, IntentType::Pose)) {
    return;
  }
  if (!isFinite(msg->pose)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejecting non-finite pose intent");
    return;
  }
  storeIntent(Intent{
    msg->requester,
    msg->controller,
    IntentType::Pose,
    msg->priority,
    now(),
    ++sequence_counter_,
    msg->pose});
}

void ControllerArbitrator::handleWrenchCommand(
  const sura_msgs::msg::SuraWrenchCommand::SharedPtr msg)
{
  if (!validateCommon(msg->requester, msg->controller, msg->priority, IntentType::Wrench)) {
    return;
  }
  if (!isFinite(msg->wrench)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejecting non-finite wrench intent");
    return;
  }
  storeIntent(Intent{
    msg->requester,
    msg->controller,
    IntentType::Wrench,
    msg->priority,
    now(),
    ++sequence_counter_,
    msg->wrench});
}

void ControllerArbitrator::storeIntent(Intent intent)
{
  std::lock_guard<std::mutex> lock(mutex_);
  intents_[makeIntentKey(intent.requester, intent.controller)] = std::move(intent);
  zero_published_after_idle_ = false;
}

void ControllerArbitrator::handleClearControllerIntents(
  const std::shared_ptr<sura_msgs::srv::ClearControllerIntents::Request> request,
  std::shared_ptr<sura_msgs::srv::ClearControllerIntents::Response> response)
{
  if (request->controller.empty()) {
    response->success = false;
    response->message = "controller cannot be empty";
    response->cleared_count = 0;
    return;
  }

  std::optional<Intent> previous_winner;
  uint32_t cleared_count = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous_winner = last_published_winner_;
    for (auto it = intents_.begin(); it != intents_.end(); ) {
      if (it->second.controller == request->controller) {
        it = intents_.erase(it);
        ++cleared_count;
      } else {
        ++it;
      }
    }
    if (last_published_winner_ && last_published_winner_->controller == request->controller) {
      last_published_winner_.reset();
      zero_published_after_idle_ = true;
    }
  }

  if (previous_winner && previous_winner->controller == request->controller) {
    publishZeroIfNeeded(previous_winner);
  }

  response->success = true;
  response->cleared_count = cleared_count;
  response->message = cleared_count == 0 ?
    "No matching intents to clear" :
    "Cleared intents for controller " + request->controller;
}

void ControllerArbitrator::handleControllerInterlock(
  const std::shared_ptr<sura_msgs::srv::ControllerInterlock::Request> request,
  std::shared_ptr<sura_msgs::srv::ControllerInterlock::Response> response)
{
  using ControllerInterlock = sura_msgs::srv::ControllerInterlock;

  if (request->command == ControllerInterlock::Request::SET) {
    if (request->reason.empty()) {
      response->success = false;
      response->message = "reason cannot be empty";
      return;
    }

    std::optional<Intent> previous_winner;
    uint32_t cleared_count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      previous_winner = last_published_winner_;
      if (request->enabled) {
        auto & blocked = controller_interlocks_[request->reason];
        blocked.clear();
        for (const auto & controller : request->blocked_controllers) {
          if (!controller.empty()) {
            blocked.insert(controller);
          }
        }
      } else {
        controller_interlocks_.erase(request->reason);
      }

      for (auto it = intents_.begin(); it != intents_.end(); ) {
        if (isControllerBlockedLocked(it->second.controller)) {
          it = intents_.erase(it);
          ++cleared_count;
        } else {
          ++it;
        }
      }
      if (last_published_winner_ &&
        isControllerBlockedLocked(last_published_winner_->controller))
      {
        last_published_winner_.reset();
        zero_published_after_idle_ = true;
      }
    }

    bool previous_blocked = false;
    if (previous_winner) {
      std::lock_guard<std::mutex> lock(mutex_);
      previous_blocked = isControllerBlockedLocked(previous_winner->controller);
    }
    if (previous_winner && previous_blocked) {
      publishZeroIfNeeded(previous_winner);
    }

    response->success = true;
    response->message = request->enabled ?
      "Controller interlock enabled. cleared_intents=" + std::to_string(cleared_count) :
      "Controller interlock disabled";
  } else if (request->command == ControllerInterlock::Request::QUERY) {
    response->success = true;
    response->message = "Controller interlock state";
  } else {
    response->success = false;
    response->message = "Unknown controller interlock command";
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::set<std::string> blocked_controllers;
    std::string reasons;
    for (const auto & interlock : controller_interlocks_) {
      if (!reasons.empty()) {
        reasons += ",";
      }
      reasons += interlock.first;
      blocked_controllers.insert(interlock.second.begin(), interlock.second.end());
    }
    response->enabled = !controller_interlocks_.empty();
    response->reason = reasons;
    response->blocked_controllers.assign(
      blocked_controllers.begin(),
      blocked_controllers.end());
  }
}

void ControllerArbitrator::handleSwitchController(
  const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & controller : request->activate_controllers) {
      if (isControllerBlockedLocked(controller)) {
        response->ok = false;
        RCLCPP_WARN(
          get_logger(),
          "Rejecting switch request: controller '%s' is blocked by safety interlock",
          controller.c_str());
        return;
      }
    }
  }

  if (!controller_manager_switch_client_->wait_for_service(std::chrono::milliseconds(200))) {
    response->ok = false;
    RCLCPP_WARN(
      get_logger(),
      "Controller manager switch service '%s' is not available",
      controller_manager_switch_service_.c_str());
    return;
  }

  auto forward_request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>(*request);
  auto future = controller_manager_switch_client_->async_send_request(forward_request);

  const auto result = rclcpp::spin_until_future_complete(
    switch_proxy_node_,
    future,
    std::chrono::seconds(2));

  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    response->ok = false;
    RCLCPP_ERROR(get_logger(), "Controller manager switch request timed out");
    return;
  }

  response->ok = future.get()->ok;
}

void ControllerArbitrator::arbitrationTick()
{
  if (!isActive()) {
    return;
  }

  std::optional<Intent> winner;
  std::optional<Intent> previous_winner;
  bool should_zero_previous = false;
  bool position_hold_velocity_mode = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous_winner = last_published_winner_;
    removeExpiredIntents(now());
    position_hold_velocity_mode = hasPositionHoldVelocityModeLocked();
    winner = selectWinnerLocked();
    if (winner) {
      zero_published_after_idle_ = false;
      if (last_published_winner_ && makeWinnerKey(*last_published_winner_) != makeWinnerKey(*winner)) {
        should_zero_previous = true;
      }
      last_published_winner_ = winner;
    } else {
      should_zero_previous = !zero_published_after_idle_;
      last_published_winner_.reset();
      zero_published_after_idle_ = true;
    }
  }

  if (should_zero_previous) {
    publishZeroIfNeeded(previous_winner);
  }
  if (winner) {
    publishWinner(*winner, position_hold_velocity_mode);
  }
}

void ControllerArbitrator::removeExpiredIntents(const rclcpp::Time & now_time)
{
  const auto timeout = rclcpp::Duration::from_seconds(command_timeout_s_);
  for (auto it = intents_.begin(); it != intents_.end(); ) {
    const auto & intent = it->second;
    if (intent.type != IntentType::Pose && (now_time - intent.stamp) > timeout) {
      it = intents_.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<ControllerArbitrator::Intent> ControllerArbitrator::selectWinnerLocked() const
{
  std::optional<Intent> winner;
  for (const auto & item : intents_) {
    const auto & candidate = item.second;
    if (!winner ||
      candidate.priority > winner->priority ||
      (candidate.priority == winner->priority && candidate.sequence > winner->sequence))
    {
      winner = candidate;
    }
  }
  return winner;
}

bool ControllerArbitrator::hasPositionHoldVelocityModeLocked() const
{
  for (const auto & item : intents_) {
    const auto & intent = item.second;
    if (intent.controller == position_hold_controller_name_ ||
      intent.controller == position_hold_temporary_controller_name_ ||
      intent.controller == position_hold_reposition_controller_name_)
    {
      return true;
    }
  }
  return false;
}

void ControllerArbitrator::publishWinner(
  const Intent & intent,
  bool position_hold_velocity_mode)
{
  if (intent.type == IntentType::Velocity) {
    const auto & velocity = std::get<VelocityMsg>(intent.payload);

    if (intent.controller == position_hold_reposition_controller_name_) {
      if (position_hold_reposition_feedforward_pub_) {
        position_hold_reposition_feedforward_pub_->publish(velocity);
      }
      return;
    }

    if (intent.controller == position_hold_temporary_controller_name_) {
      if (position_hold_feedforward_pub_) {
        position_hold_feedforward_pub_->publish(velocity);
      }
      return;
    }

    if (intent.controller == body_velocity_controller_name_ && position_hold_velocity_mode) {
      if (position_hold_feedforward_pub_) {
        position_hold_feedforward_pub_->publish(velocity);
      }
      return;
    }

    const auto publisher = velocity_publishers_.find(intent.controller);
    if (publisher != velocity_publishers_.end()) {
      publisher->second->publish(velocity);
    }
  } else if (intent.type == IntentType::Pose) {
    const auto publisher = pose_publishers_.find(intent.controller);
    if (publisher != pose_publishers_.end()) {
      publisher->second->publish(std::get<PoseMsg>(intent.payload));
    }
  } else {
    const auto publisher = wrench_publishers_.find(intent.controller);
    if (publisher != wrench_publishers_.end()) {
      publisher->second->publish(std::get<WrenchMsg>(intent.payload));
    }
  }
}

void ControllerArbitrator::publishZeroIfNeeded(const std::optional<Intent> & previous_winner)
{
  if (!previous_winner) {
    return;
  }
  if (previous_winner->type == IntentType::Velocity) {
    if (previous_winner->controller == position_hold_reposition_controller_name_) {
      if (position_hold_reposition_feedforward_pub_) {
        position_hold_reposition_feedforward_pub_->publish(VelocityMsg{});
      }
      return;
    }

    if (previous_winner->controller == position_hold_temporary_controller_name_) {
      if (position_hold_feedforward_pub_) {
        position_hold_feedforward_pub_->publish(VelocityMsg{});
      }
      return;
    }

    const auto publisher = velocity_publishers_.find(previous_winner->controller);
    if (publisher != velocity_publishers_.end()) {
      publisher->second->publish(VelocityMsg{});
    }

    if (previous_winner->controller == body_velocity_controller_name_) {
      if (position_hold_feedforward_pub_) {
        position_hold_feedforward_pub_->publish(VelocityMsg{});
      }
      if (position_hold_reposition_feedforward_pub_) {
        position_hold_reposition_feedforward_pub_->publish(VelocityMsg{});
      }
    }
  } else if (previous_winner->type == IntentType::Wrench) {
    const auto publisher = wrench_publishers_.find(previous_winner->controller);
    if (publisher != wrench_publishers_.end()) {
      publisher->second->publish(WrenchMsg{});
    }
  }
}

void ControllerArbitrator::cleanupResources()
{
  if (arbitration_timer_) {
    arbitration_timer_->cancel();
  }
  velocity_sub_.reset();
  pose_sub_.reset();
  wrench_sub_.reset();
  clear_service_.reset();
  interlock_service_.reset();
  switch_service_.reset();
  controller_manager_switch_client_.reset();
  switch_proxy_node_.reset();
  arbitration_timer_.reset();
  velocity_publishers_.clear();
  position_hold_feedforward_pub_.reset();
  position_hold_reposition_feedforward_pub_.reset();
  pose_publishers_.clear();
  wrench_publishers_.clear();
  routes_.clear();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    intents_.clear();
    last_published_winner_.reset();
    zero_published_after_idle_ = true;
    controller_interlocks_.clear();
  }
}

}  // namespace sura_controllers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sura_controllers::ControllerArbitrator>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
