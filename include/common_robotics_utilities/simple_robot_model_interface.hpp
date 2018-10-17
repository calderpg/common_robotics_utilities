#pragma once

#include <string>
#include <Eigen/Geometry>

#include <common_robotics_utilities/math.hpp>

namespace common_robotics_utilities
{
namespace simple_robot_model_interface
{
/// This is basically the absolute minimal robot model interface needed to do
/// motion planning and forward kinematics. It does not include link geometry.
template<typename Configuration,
         typename ConfigAlloc=std::allocator<Configuration>>
class SimpleRobotModelInterface
{
public:
  virtual ~SimpleRobotModelInterface() {}

  /// Clone the current robot model.
  virtual SimpleRobotModelInterface<Configuration,
                                    ConfigAlloc>* Clone() const = 0;

  /// Return the current position (i.e. joint values).
  virtual const Configuration& GetPosition() const = 0;

  /// Set a new position (i.e. joint values) and return the new value.
  virtual const Configuration& SetPosition(const Configuration& config) = 0;

  /// Get names of links in the robot.
  virtual std::vector<std::string> GetLinkNames() const = 0;

  /// Get the transform of the link with index @param link_index relative to
  /// world.
  virtual Eigen::Isometry3d GetLinkTransform(
      const int64_t link_index) const = 0;

  /// Get the transform of the link with name @param link_name relative to
  /// world.
  virtual Eigen::Isometry3d GetLinkTransform(
      const std::string& link_name) const = 0;

  /// Get the transforms of all links in the robot relative to world in the same
  /// order as returned by GetLinkNames().
  virtual math::VectorIsometry3d GetLinkTransforms() const = 0;

  /// Return a map of <name, transform> for all links in the robot relative to
  /// the world.
  virtual math::MapStringIsometry3d GetLinkTransformsMap() const = 0;

  /// Return C-space distance between @param config1 and @param config2.
  virtual double ComputeConfigurationDistance(
      const Configuration& config1, const Configuration& config2) const = 0;

  /// Return C-space absolute-value distance between @param config1 and @param
  /// config2 for each dimension of the C-space separately.
  Eigen::VectorXd ComputePerDimensionConfigurationDistance(
      const Configuration& config1, const Configuration& config2) const
  {
    return ComputePerDimensionConfigurationSignedDistance(
        config1, config2).cwiseAbs();
  }

  /// Return C-space signed distance between @param config1 and @param config2
  /// for each dimension of the C-space separately.
  virtual Eigen::VectorXd ComputePerDimensionConfigurationSignedDistance(
      const Configuration& config1, const Configuration& config2) const = 0;

  /// Return C-space distance between the current configuration and @param
  /// config.
  double ComputeConfigurationDistanceTo(const Configuration& config) const
  {
    return ComputeConfigurationDistance(GetPosition(), config);
  }

  /// Return C-space abolsute-value distance between the current configuration
  /// and @param config for each dimension of the C-space separately.
  Eigen::VectorXd ComputePerDimensionConfigurationDistanceTo(
      const Configuration& config) const
  {
    return ComputePerDimensionConfigurationDistance(GetPosition(), config);
  }

  /// Return C-space signed distance between the current configuration and
  /// @param config for each dimension of the C-space separately.
  Eigen::VectorXd ComputePerDimensionConfigurationSignedDistanceTo(
      const Configuration& config) const
  {
    return ComputePerDimensionConfigurationSignedDistance(
        GetPosition(), config);
  }

  /// Interpolate a configuration of the robot between @param start and @param
  /// end for the provided ratio @param ratio.
  virtual Configuration InterpolateBetweenConfigurations(
      const Configuration& start, const Configuration& end,
      const double ratio) const = 0;

  /// Average the provided set of configurations @param configurations.
  virtual Configuration AverageConfigurations(
      const std::vector<Configuration, ConfigAlloc>& configurations) const = 0;

  /// Compute the translation-only part of the Jacobian at the provided point
  /// @param link_relative_point on link @param link_name in the robot.
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic>
  ComputeLinkPointTranslationJacobian(
      const std::string& link_name,
      const Eigen::Vector4d& link_relative_point) const = 0;

  /// Compute the Jacobian at the provided point @param link_relative_point on
  /// link @param link_name in the robot.
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic>
  ComputeLinkPointJacobian(
      const std::string& link_name,
      const Eigen::Vector4d& link_relative_point) const = 0;
};
}  // namespace simple_robot_model_interface
}  // namespace common_robotics_utilities
