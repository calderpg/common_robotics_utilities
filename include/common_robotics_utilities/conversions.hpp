#pragma once

#include <vector>

#include <Eigen/Geometry>

namespace common_robotics_utilities
{
namespace conversions
{
Eigen::Quaterniond QuaternionFromRPY(const double R,
                                     const double P,
                                     const double Y);

// URDF RPY IS ACTUALLY APPLIED Y*P*R
Eigen::Quaterniond QuaternionFromUrdfRPY(const double R,
                                         const double P,
                                         const double Y);

// Returns XYZ Euler angles
Eigen::Vector3d EulerAnglesFromRotationMatrix(
    const Eigen::Matrix3d& rot_matrix);

// Returns XYZ Euler angles
Eigen::Vector3d EulerAnglesFromQuaternion(const Eigen::Quaterniond& quat);

// Returns XYZ Euler angles
Eigen::Vector3d EulerAnglesFromIsometry3d(const Eigen::Isometry3d& trans);

Eigen::Isometry3d TransformFromXYZRPY(const double x,
                                      const double y,
                                      const double z,
                                      const double roll,
                                      const double pitch,
                                      const double yaw);

Eigen::Isometry3d TransformFromRPY(const Eigen::Vector3d& translation,
                                   const Eigen::Vector3d& rotation);

Eigen::Isometry3d TransformFromRPY(const Eigen::VectorXd& components);

// URDF RPY IS ACTUALLY APPLIED Y*P*R
Eigen::Isometry3d TransformFromUrdfXYZRPY(const double x,
                                          const double y,
                                          const double z,
                                          const double roll,
                                          const double pitch,
                                          const double yaw);

// URDF RPY IS ACTUALLY APPLIED Y*P*R
Eigen::Isometry3d TransformFromUrdfRPY(const Eigen::Vector3d& translation,
                                       const Eigen::Vector3d& rotation);

// URDF RPY IS ACTUALLY APPLIED Y*P*R
Eigen::Isometry3d TransformFromUrdfRPY(const Eigen::VectorXd& components);

Eigen::VectorXd TransformToRPY(const Eigen::Isometry3d& transform);

Eigen::Vector3d StdVectorDoubleToEigenVector3d(
    const std::vector<double>& vector);

Eigen::VectorXd StdVectorDoubleToEigenVectorXd(
    const std::vector<double>& vector);

std::vector<double> EigenVector3dToStdVectorDouble(
    const Eigen::Vector3d& point);

std::vector<double> EigenVectorXdToStdVectorDouble(
    const Eigen::VectorXd& eigen_vector);

// Takes <x, y, z, w> as is the ROS custom!
Eigen::Quaterniond StdVectorDoubleToEigenQuaterniond(
    const std::vector<double>& vector);

// Returns <x, y, z, w> as is the ROS custom!
std::vector<double> EigenQuaterniondToStdVectorDouble(
    const Eigen::Quaterniond& quat);
}  // namespace conversions
}  // namespace common_robotics_utilities
