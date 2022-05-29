#pragma once

#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
namespace time_optimal_trajectory_parametrization
{
/// Stores a position+velocity+time point in a trajectory.
class PositionVelocityTimePoint
{
public:
  PositionVelocityTimePoint() {}

  PositionVelocityTimePoint(
      const Eigen::VectorXd& position, const Eigen::VectorXd& velocity,
      const double time)
      : position_(position), velocity_(velocity), time_(time)
  {
    if (position_.size() != velocity_.size())
    {
      throw std::invalid_argument("position_.size() != velocity_.size()");
    }
    if (time_ < 0.0)
    {
      throw std::invalid_argument("time must be >= 0");
    }
  }

  const Eigen::VectorXd& Position() const { return position_; }

  const Eigen::VectorXd& Velocity() const { return velocity_; }

  double Time() const { return time_; }

private:
  Eigen::VectorXd position_;
  Eigen::VectorXd velocity_;
  double time_ = 0.0;
};

/// Basic interface to a position+velocity trajectory.
class PositionVelocityTrajectoryInterface
{
public:
  virtual ~PositionVelocityTrajectoryInterface() {}

  virtual std::unique_ptr<PositionVelocityTrajectoryInterface>
  Clone() const = 0;

  virtual double Duration() const = 0;

  virtual PositionVelocityTimePoint GetPositionVelocityTimePoint(
      const double time) const = 0;
};

/// Parametrize the provided @param path with the provided velocity and
/// acceleration limits into a trajectory using Time-Optimal Trajectory
/// Parametrization.
std::unique_ptr<PositionVelocityTrajectoryInterface> ParametrizePathTOTP(
    const std::vector<Eigen::VectorXd>& path,
    const Eigen::VectorXd& max_velocity,
    const Eigen::VectorXd& max_acceleration,
    const double max_deviation = 0.0, const double timestep = 0.001);
}  // namespace time_optimal_trajectory_parametrization
}  // namespace common_robotics_utilities
