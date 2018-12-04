#pragma once

#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

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

Eigen::Vector3d GeometryPointToEigenVector3d(
    const geometry_msgs::Point& point);

geometry_msgs::Point EigenVector3dToGeometryPoint(
    const Eigen::Vector3d& point);

Eigen::Vector4d GeometryPointToEigenVector4d(
    const geometry_msgs::Point& point);

geometry_msgs::Point EigenVector4dToGeometryPoint(
    const Eigen::Vector4d& point);

geometry_msgs::PointStamped EigenVector3dToGeometryPointStamped(
    const Eigen::Vector3d& point, const std::string& frame_id);

Eigen::Vector3d GeometryVector3ToEigenVector3d(
    const geometry_msgs::Vector3& vector);

geometry_msgs::Vector3 EigenVector3dToGeometryVector3(
    const Eigen::Vector3d& vector);

Eigen::Vector4d GeometryVector3ToEigenVector4d(
    const geometry_msgs::Vector3& vector);

geometry_msgs::Vector3 EigenVector4dToGeometryVector3(
    const Eigen::Vector4d& vector);

Eigen::Quaterniond GeometryQuaternionToEigenQuaterniond(
    const geometry_msgs::Quaternion& quat);

geometry_msgs::Quaternion EigenQuaterniondToGeometryQuaternion(
    const Eigen::Quaterniond& quat);

Eigen::Isometry3d GeometryPoseToEigenIsometry3d(
    const geometry_msgs::Pose& pose);

geometry_msgs::Pose EigenIsometry3dToGeometryPose(
    const Eigen::Isometry3d& transform);

geometry_msgs::PoseStamped EigenIsometry3dToGeometryPoseStamped(
    const Eigen::Isometry3d& transform, const std::string& frame_id);

Eigen::Isometry3d GeometryTransformToEigenIsometry3d(
    const geometry_msgs::Transform& transform);

geometry_msgs::Transform EigenIsometry3dToGeometryTransform(
    const Eigen::Isometry3d& transform);

geometry_msgs::TransformStamped EigenIsometry3dToGeometryTransformStamped(
    const Eigen::Isometry3d& transform, const std::string& frame_id,
    const std::string& child_frame_id);

Eigen::Matrix3Xd VectorGeometryPointToEigenMatrix3Xd(
    const std::vector<geometry_msgs::Point>& vector_geom);

std::vector<geometry_msgs::Point> EigenMatrix3XdToVectorGeometryPoint(
    const Eigen::Matrix3Xd& eigen_matrix);

std::vector<geometry_msgs::Point>
VectorEigenVector3dToVectorGeometryPoint(
    const common_robotics_utilities::math::VectorVector3d& vector_eigen);

common_robotics_utilities::math::VectorVector3d
VectorGeometryPointToVectorEigenVector3d(
    const std::vector<geometry_msgs::Point>& vector_geom);

common_robotics_utilities::math::VectorVector3d
VectorGeometryVector3ToEigenVector3d(
    const std::vector<geometry_msgs::Vector3>& vector_geom);

common_robotics_utilities::math::VectorIsometry3d
VectorGeometryPoseToVectorIsometry3d(
    const std::vector<geometry_msgs::Pose>& vector_geom);

common_robotics_utilities::math::VectorIsometry3d
VectorGeometryPoseToVectorIsometry3d(
    const std::vector<geometry_msgs::Transform>& vector_geom);

std::vector<geometry_msgs::Pose> VectorIsometry3dToVectorGeometryPose(
    const common_robotics_utilities::math::VectorIsometry3d& vector_eigen);

std::vector<geometry_msgs::Transform> VectorIsometry3dToVectorGeometryTransform(
    const common_robotics_utilities::math::VectorIsometry3d& vector_eigen);
}  // namespace conversions
}  // namespace common_robotics_utilities
