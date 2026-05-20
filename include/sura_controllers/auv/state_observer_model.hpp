#pragma once

#include <array>
#include <cstdint>
#include <cstddef>

#include <Eigen/Dense>

namespace sura_controllers::auv
{

class StateObserverModel
{
public:
  static constexpr std::size_t kAxisCount = 6;

  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using ParameterVector = Eigen::Matrix<double, 4, 1>;
  using ParameterCovariance = Eigen::Matrix<double, 4, 4>;

  struct Limits
  {
    double min_effective_inertia{1e-3};
    double max_effective_inertia{1e6};
    double min_linear_damping{0.0};
    double max_linear_damping{1e6};
    double min_quadratic_damping{0.0};
    double max_quadratic_damping{1e6};
    double min_static_wrench{-1e6};
    double max_static_wrench{1e6};
  };

  StateObserverModel();

  void setRigidBodyInertia(const Vector6 & rigid_body_inertia);
  void setEffectiveInertia(const Vector6 & effective_inertia);
  void setLinearDamping(const Vector6 & linear_damping);
  void setQuadraticDamping(const Vector6 & quadratic_damping);
  void setStaticWrench(const Vector6 & static_wrench);
  void setLimits(const Limits & limits);

  void resetEstimator(double initial_covariance);
  void resetStatistics();

  const Vector6 & rigidBodyInertia() const;
  Vector6 effectiveInertia() const;
  Vector6 addedMass() const;
  Vector6 linearDamping() const;
  Vector6 quadraticDamping() const;
  Vector6 staticWrench() const;
  Vector6 effectiveInertiaVariance() const;
  Vector6 linearDampingVariance() const;
  Vector6 quadraticDampingVariance() const;
  Vector6 staticWrenchVariance() const;
  Vector6 residualRms() const;

  std::uint32_t sampleCount() const;

  Vector6 predictAcceleration(
    const Vector6 & body_wrench,
    const Vector6 & body_velocity,
    const Vector6 & restoring_wrench) const;

  bool updateAxis(
    std::size_t axis,
    double body_wrench,
    double body_velocity,
    double body_acceleration,
    double restoring_wrench,
    double forgetting_factor,
    double min_excitation);

  void update(
    const Vector6 & body_wrench,
    const Vector6 & body_velocity,
    const Vector6 & body_acceleration,
    const Vector6 & restoring_wrench,
    double forgetting_factor,
    double min_excitation);

private:
  static double clamp(double value, double lower, double upper);
  static bool isFinite(double value);

  void clampAxis(std::size_t axis);

  Vector6 rigid_body_inertia_;
  std::array<ParameterVector, kAxisCount> parameters_;
  std::array<ParameterCovariance, kAxisCount> covariance_;
  std::array<double, kAxisCount> residual_sum_squares_;
  std::array<std::uint32_t, kAxisCount> axis_sample_counts_;
  std::uint32_t sample_count_{0};
  Limits limits_;
};

}  // namespace sura_controllers::auv
