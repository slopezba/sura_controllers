#include "sura_controllers/auv/state_observer_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sura_controllers::auv
{

StateObserverModel::StateObserverModel()
{
  for (auto & parameters : parameters_) {
    parameters.setZero();
  }

  rigid_body_inertia_ <<
    11.5, 11.5, 11.5, 0.16, 0.16, 0.16;

  const Vector6 effective_inertia =
    (Vector6() << 17.0, 24.2, 26.07, 0.28, 0.28, 0.28).finished();
  const Vector6 linear_damping =
    (Vector6() << 4.03, 6.22, 5.18, 0.07, 0.07, 0.07).finished();
  const Vector6 quadratic_damping =
    (Vector6() << 18.18, 21.66, 36.99, 1.55, 1.55, 1.55).finished();
  const Vector6 static_wrench = Vector6::Zero();

  setEffectiveInertia(effective_inertia);
  setLinearDamping(linear_damping);
  setQuadraticDamping(quadratic_damping);
  setStaticWrench(static_wrench);
  resetEstimator(1e3);
  resetStatistics();
}

void StateObserverModel::setRigidBodyInertia(const Vector6 & rigid_body_inertia)
{
  rigid_body_inertia_ = rigid_body_inertia;
}

void StateObserverModel::setEffectiveInertia(const Vector6 & effective_inertia)
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    parameters_[axis](0) = effective_inertia(static_cast<Eigen::Index>(axis));
    clampAxis(axis);
  }
}

void StateObserverModel::setLinearDamping(const Vector6 & linear_damping)
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    parameters_[axis](1) = linear_damping(static_cast<Eigen::Index>(axis));
    clampAxis(axis);
  }
}

void StateObserverModel::setQuadraticDamping(const Vector6 & quadratic_damping)
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    parameters_[axis](2) = quadratic_damping(static_cast<Eigen::Index>(axis));
    clampAxis(axis);
  }
}

void StateObserverModel::setStaticWrench(const Vector6 & static_wrench)
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    parameters_[axis](3) = static_wrench(static_cast<Eigen::Index>(axis));
    clampAxis(axis);
  }
}

void StateObserverModel::setLimits(const Limits & limits)
{
  limits_ = limits;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    clampAxis(axis);
  }
}

void StateObserverModel::resetEstimator(double initial_covariance)
{
  const double safe_covariance = std::max(1e-9, initial_covariance);
  for (auto & covariance : covariance_) {
    covariance = ParameterCovariance::Identity() * safe_covariance;
  }
}

void StateObserverModel::resetStatistics()
{
  residual_sum_squares_.fill(0.0);
  axis_sample_counts_.fill(0);
  sample_count_ = 0;
}

const StateObserverModel::Vector6 & StateObserverModel::rigidBodyInertia() const
{
  return rigid_body_inertia_;
}

StateObserverModel::Vector6 StateObserverModel::effectiveInertia() const
{
  Vector6 values;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    values(static_cast<Eigen::Index>(axis)) = parameters_[axis](0);
  }
  return values;
}

StateObserverModel::Vector6 StateObserverModel::addedMass() const
{
  return effectiveInertia() - rigid_body_inertia_;
}

StateObserverModel::Vector6 StateObserverModel::linearDamping() const
{
  Vector6 values;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    values(static_cast<Eigen::Index>(axis)) = parameters_[axis](1);
  }
  return values;
}

StateObserverModel::Vector6 StateObserverModel::quadraticDamping() const
{
  Vector6 values;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    values(static_cast<Eigen::Index>(axis)) = parameters_[axis](2);
  }
  return values;
}

StateObserverModel::Vector6 StateObserverModel::staticWrench() const
{
  Vector6 values;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    values(static_cast<Eigen::Index>(axis)) = parameters_[axis](3);
  }
  return values;
}

StateObserverModel::Vector6 StateObserverModel::effectiveInertiaVariance() const
{
  Vector6 values;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    values(static_cast<Eigen::Index>(axis)) = covariance_[axis](0, 0);
  }
  return values;
}

StateObserverModel::Vector6 StateObserverModel::linearDampingVariance() const
{
  Vector6 values;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    values(static_cast<Eigen::Index>(axis)) = covariance_[axis](1, 1);
  }
  return values;
}

StateObserverModel::Vector6 StateObserverModel::quadraticDampingVariance() const
{
  Vector6 values;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    values(static_cast<Eigen::Index>(axis)) = covariance_[axis](2, 2);
  }
  return values;
}

StateObserverModel::Vector6 StateObserverModel::staticWrenchVariance() const
{
  Vector6 values;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    values(static_cast<Eigen::Index>(axis)) = covariance_[axis](3, 3);
  }
  return values;
}

StateObserverModel::Vector6 StateObserverModel::residualRms() const
{
  Vector6 values;
  values.setZero();
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const auto count = axis_sample_counts_[axis];
    if (count > 0) {
      values(static_cast<Eigen::Index>(axis)) =
        std::sqrt(residual_sum_squares_[axis] / static_cast<double>(count));
    }
  }
  return values;
}

std::uint32_t StateObserverModel::sampleCount() const
{
  return sample_count_;
}

StateObserverModel::Vector6 StateObserverModel::predictAcceleration(
  const Vector6 & body_wrench,
  const Vector6 & body_velocity,
  const Vector6 & restoring_wrench) const
{
  Vector6 acceleration;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const Eigen::Index index = static_cast<Eigen::Index>(axis);
    const double velocity = body_velocity(index);
    const double damping =
      parameters_[axis](1) * velocity +
      parameters_[axis](2) * velocity * std::abs(velocity);
    const double static_wrench = parameters_[axis](3);
    const double inertia = std::max(parameters_[axis](0), limits_.min_effective_inertia);
    acceleration(index) =
      (body_wrench(index) - restoring_wrench(index) - damping - static_wrench) / inertia;
  }
  return acceleration;
}

bool StateObserverModel::updateAxis(
  std::size_t axis,
  double body_wrench,
  double body_velocity,
  double body_acceleration,
  double restoring_wrench,
  double forgetting_factor,
  double min_excitation)
{
  if (axis >= kAxisCount) {
    return false;
  }

  if (
    !isFinite(body_wrench) || !isFinite(body_velocity) ||
    !isFinite(body_acceleration) || !isFinite(restoring_wrench))
  {
    return false;
  }

  const double safe_forgetting_factor = clamp(forgetting_factor, 0.90, 1.0);
  const double quadratic_term =
    std::clamp(
      body_velocity * std::abs(body_velocity),
      -5.0,
      5.0);

  const ParameterVector regressor(
    body_acceleration,
    body_velocity,
    quadratic_term,
    1.0);

  const double y = body_wrench - restoring_wrench;
  if (regressor.norm() < min_excitation || std::abs(y) < min_excitation) {
    return false;
  }

  auto & covariance = covariance_[axis];
  auto & parameters = parameters_[axis];
  const ParameterVector covariance_regressor = covariance * regressor;
  const double denominator =
    safe_forgetting_factor + (regressor.transpose() * covariance_regressor)(0, 0);

  if (!isFinite(denominator) || std::abs(denominator) < 1e-12) {
    return false;
  }

  ParameterVector gain = covariance_regressor / denominator;

  // Gain clamp
  constexpr double max_gain = 0.05;

  for (Eigen::Index i = 0; i < gain.size(); ++i) {
    gain(i) = std::clamp(gain(i), -max_gain, max_gain);
  }
  
  double residual = y - regressor.dot(parameters);

  // Residual clamp
  constexpr double max_residual = 50.0;
  residual = std::clamp(residual, -max_residual, max_residual);
  ParameterVector parameter_update = gain * residual;

  // Limit parameter update speed
  constexpr double max_parameter_step = 0.05;

  for (Eigen::Index i = 0; i < parameter_update.size(); ++i) {
    parameter_update(i) =
      std::clamp(
        parameter_update(i),
        -max_parameter_step,
        max_parameter_step);
  }

  parameters += parameter_update;
  covariance =
    (covariance - gain * regressor.transpose() * covariance) / safe_forgetting_factor;
  
  // Covariance clamp
  constexpr double max_covariance = 1e6;

  for (Eigen::Index r = 0; r < covariance.rows(); ++r) {
    for (Eigen::Index c = 0; c < covariance.cols(); ++c) {
      covariance(r, c) =
        std::clamp(
          covariance(r, c),
          -max_covariance,
          max_covariance);
    }
  }  
  covariance = 0.5 * (covariance + covariance.transpose());
  clampAxis(axis);

  residual_sum_squares_[axis] += residual * residual;
  ++axis_sample_counts_[axis];
  if (sample_count_ < std::numeric_limits<std::uint32_t>::max()) {
    ++sample_count_;
  }

  return true;
}

void StateObserverModel::update(
  const Vector6 & body_wrench,
  const Vector6 & body_velocity,
  const Vector6 & body_acceleration,
  const Vector6 & restoring_wrench,
  double forgetting_factor,
  double min_excitation)
{
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    const Eigen::Index index = static_cast<Eigen::Index>(axis);
    updateAxis(
      axis,
      body_wrench(index),
      body_velocity(index),
      body_acceleration(index),
      restoring_wrench(index),
      forgetting_factor,
      min_excitation);
  }
}

double StateObserverModel::clamp(double value, double lower, double upper)
{
  if (lower > upper) {
    return value;
  }
  return std::clamp(value, lower, upper);
}

bool StateObserverModel::isFinite(double value)
{
  return std::isfinite(value);
}

void StateObserverModel::clampAxis(std::size_t axis)
{
  auto & parameters = parameters_[axis];
  parameters(0) = clamp(
    parameters(0), limits_.min_effective_inertia, limits_.max_effective_inertia);
  parameters(1) = clamp(
    parameters(1), limits_.min_linear_damping, limits_.max_linear_damping);
  parameters(2) = clamp(
    parameters(2), limits_.min_quadratic_damping, limits_.max_quadratic_damping);
  parameters(3) = clamp(
    parameters(3), limits_.min_static_wrench, limits_.max_static_wrench);
}

}  // namespace sura_controllers::auv
